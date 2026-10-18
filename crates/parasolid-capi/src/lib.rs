//! C ABI compiled into parasolid.dll. Callers free returned strings with
//! `parasolid_string_free` so the allocation stays on this crate's allocator.
//!
//! The library inspects, parses, and tessellates transmit files for the viewer.

#![allow(unsafe_code)]

mod mesh;

use std::ffi::{CStr, CString};
use std::fmt::Write as _;
use std::os::raw::{c_char, c_int};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};

use parasolid_core::brep::{map_xb_brep, map_xt_brep, BrepModel};
use parasolid_core::{
    parse_schema_catalog, parse_xb, parse_xt, BuiltinProfileRegistry, DocumentLimits, ErrorKind,
    FieldValue, InMemorySchemaProvider, InspectionLimits, ParseError, RawNode, SchemaCatalogLimits,
    SchemaKey, SchemaProviderResolution,
};

const OK: c_int = 0;
const UNSUPPORTED: c_int = 1;
const MALFORMED: c_int = 2;
const LIMIT: c_int = 3;
const IO: c_int = 4;
const PANIC: c_int = 5;
const INVALID: c_int = 6;

const FORMAT_XB: c_int = 1;
const FORMAT_XT: c_int = 2;

#[derive(Clone, Copy, PartialEq, Eq)]
enum SourceFormat {
    Binary,
    Text,
}

enum Fail {
    Parse(ParseError),
    Status(c_int, String),
}

impl Fail {
    fn status(&self) -> c_int {
        match self {
            Self::Parse(error) => status_of(error),
            Self::Status(status, _) => *status,
        }
    }

    fn message(&self) -> String {
        match self {
            Self::Parse(error) => error.to_string(),
            Self::Status(_, message) => message.clone(),
        }
    }
}

fn status_of(error: &ParseError) -> c_int {
    match error.kind() {
        ErrorKind::LimitExceeded | ErrorKind::InvalidLimit => LIMIT,
        ErrorKind::UnsupportedBinaryFormat
        | ErrorKind::UnsupportedSchemaFieldType
        | ErrorKind::UnsupportedBaseSchemaType
        | ErrorKind::UnsupportedBuiltinSchemaKey
        | ErrorKind::BuiltinProfileUncoveredType
        | ErrorKind::UnsupportedUserFields
        | ErrorKind::MissingBaseSchema
        | ErrorKind::UnknownBaseSchemaType => UNSUPPORTED,
        _ => MALFORMED,
    }
}

fn cstring(text: String) -> CString {
    let bytes: Vec<u8> = text.into_bytes().into_iter().filter(|byte| *byte != 0).collect();
    match CString::new(bytes) {
        Ok(value) => value,
        Err(error) => {
            let cleaned: Vec<u8> = error.into_vec().into_iter().filter(|byte| *byte != 0).collect();
            CString::new(cleaned).unwrap_or_default()
        }
    }
}

fn write_slot(slot: *mut *mut c_char, text: String) {
    if slot.is_null() {
        return;
    }
    unsafe { *slot = cstring(text).into_raw() };
}

fn clear_slot(slot: *mut *mut c_char) {
    if !slot.is_null() {
        unsafe { *slot = std::ptr::null_mut() };
    }
}

fn bytes<'a>(ptr: *const u8, len: usize) -> Result<&'a [u8], c_int> {
    if ptr.is_null() {
        if len == 0 {
            Ok(&[])
        } else {
            Err(INVALID)
        }
    } else {
        Ok(unsafe { std::slice::from_raw_parts(ptr, len) })
    }
}

fn c_str<'a>(ptr: *const c_char) -> Result<&'a str, c_int> {
    if ptr.is_null() {
        return Err(INVALID);
    }
    let text = unsafe { CStr::from_ptr(ptr) };
    text.to_str().map_err(|_| INVALID)
}

fn json_escape(text: &str) -> String {
    let mut out = String::new();
    for ch in text.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            control if (control as u32) < 0x20 => {
                let _ = write!(out, "\\u{value:04x}", value = control as u32);
            }
            other => out.push(other),
        }
    }
    out
}

fn file_name(path: &str) -> &str {
    path.rsplit(['/', '\\']).next().unwrap_or(path)
}

fn path_format(path: &str) -> Option<SourceFormat> {
    let ext = std::path::Path::new(file_name(path)).extension()?;
    if ext.eq_ignore_ascii_case("x_b") || ext.eq_ignore_ascii_case("xb") {
        Some(SourceFormat::Binary)
    } else if ext.eq_ignore_ascii_case("x_t") || ext.eq_ignore_ascii_case("xt") {
        Some(SourceFormat::Text)
    } else {
        None
    }
}

fn detect_format(path: Option<&str>, data: &[u8]) -> Result<SourceFormat, Fail> {
    if let Some(path) = path
        && let Some(format) = path_format(path)
    {
        return Ok(format);
    }
    let prefix = &data[..data.len().min(4096)];
    if prefix.starts_with(b"PS\0\0") || prefix.windows(5).any(|window| window == b"\nPS\0\0") {
        return Ok(SourceFormat::Binary);
    }
    if prefix.starts_with(b"T") || prefix.starts_with(b"**PART") {
        return Ok(SourceFormat::Text);
    }
    Err(Fail::Status(
        UNSUPPORTED,
        "cannot identify source encoding; pass x-b or x-t".into(),
    ))
}

fn format_from_code(code: c_int, path: Option<&str>, data: &[u8]) -> Result<SourceFormat, Fail> {
    match code {
        0 => detect_format(path, data),
        FORMAT_XB => Ok(SourceFormat::Binary),
        FORMAT_XT => Ok(SourceFormat::Text),
        _ => Err(Fail::Status(UNSUPPORTED, "unknown format code".into())),
    }
}

fn format_code(format: SourceFormat) -> c_int {
    match format {
        SourceFormat::Binary => FORMAT_XB,
        SourceFormat::Text => FORMAT_XT,
    }
}

fn limits() -> (DocumentLimits, InspectionLimits) {
    let document = DocumentLimits::default();
    let inspection = InspectionLimits {
        max_file_size: document.max_file_size,
        max_string_bytes: document.max_string_bytes,
    };
    (document, inspection)
}

fn read_path(path: &str) -> Result<Vec<u8>, Fail> {
    let (document, _) = limits();
    let meta = std::fs::metadata(path).map_err(|error| Fail::Status(IO, error.to_string()))?;
    if meta.len() > u64::try_from(document.max_file_size).unwrap_or(u64::MAX) {
        return Err(Fail::Status(
            LIMIT,
            "file_size exceeds the configured limit".into(),
        ));
    }
    std::fs::read(path).map_err(|error| Fail::Status(IO, error.to_string()))
}

fn inspect_json(data: &[u8], format: SourceFormat) -> Result<String, Fail> {
    let (_, inspection) = limits();
    match format {
        SourceFormat::Binary => {
            let header = parasolid_core::inspect_xb(data, inspection).map_err(Fail::Parse)?;
            Ok(format!(
                "{{\"format\":\"binary\",\"schema_key\":\"{schema}\",\"modeller_version\":\"{modeller}\",\"user_field_size\":{user_field},\"file_size\":{file_size}}}",
                schema = json_escape(&header.schema_key),
                modeller = json_escape(&header.modeller_version),
                user_field = header.user_field_size,
                file_size = header.file_size,
            ))
        }
        SourceFormat::Text => {
            let header = parasolid_core::inspect_xt(data, inspection).map_err(Fail::Parse)?;
            Ok(format!(
                "{{\"format\":\"text\",\"schema_key\":\"{schema}\",\"modeller_version\":\"{modeller}\",\"user_field_size\":{user_field},\"file_size\":{file_size}}}",
                schema = json_escape(&header.schema_key),
                modeller = json_escape(&header.modeller_version),
                user_field = header.user_field_size,
                file_size = header.file_size,
            ))
        }
    }
}

fn unsupported_key(schema_key: &str) -> Fail {
    Fail::Status(
        UNSUPPORTED,
        format!("no built-in profile supports schema key {schema_key}"),
    )
}

fn push_file(candidates: &mut Vec<PathBuf>, dir: impl AsRef<Path>, name: &str) {
    candidates.push(dir.as_ref().join(name));
}

fn push_child_schemas(candidates: &mut Vec<PathBuf>, root: &Path, name: &str) {
    let Ok(entries) = std::fs::read_dir(root) else {
        return;
    };
    let mut dirs: Vec<PathBuf> = entries
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .collect();
    dirs.sort();
    for dir in dirs.iter().rev() {
        let Some(folder) = dir.file_name().and_then(|value| value.to_str()) else {
            continue;
        };
        if folder == "schema" || folder.starts_with("schema_") {
            push_file(candidates, dir, name);
        }
    }
}

/// Exact `sch_<id>.sch_txt` locations. Built-in profiles do not cover every
/// embedded-base key. `SolidWorks` 2024 files name base `13006` and carry the
/// delta in the transmit stream.
fn catalog_candidates(schema_id: &str) -> Vec<PathBuf> {
    let name = format!("sch_{schema_id}.sch_txt");
    let mut candidates = Vec::new();
    if let Ok(dir) = std::env::var("PARASOLID_SCHEMA_DIR") {
        push_file(&mut candidates, dir, &name);
    }
    if let Ok(exe) = std::env::current_exe()
        && let Some(dir) = exe.parent()
    {
        push_file(&mut candidates, dir.join("schema"), &name);
        push_file(&mut candidates, dir, &name);
    }
    #[cfg(windows)]
    {
        push_child_schemas(
            &mut candidates,
            Path::new(r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS"),
            &name,
        );
        let autodesk = Path::new(r"C:\Program Files\Common Files\Autodesk Shared\Components");
        if let Ok(years) = std::fs::read_dir(autodesk) {
            let mut year_dirs: Vec<PathBuf> = years
                .flatten()
                .map(|entry| entry.path())
                .filter(|path| path.is_dir())
                .collect();
            year_dirs.sort();
            for year in year_dirs.iter().rev() {
                let Ok(versions) = std::fs::read_dir(year) else {
                    continue;
                };
                let mut version_dirs: Vec<PathBuf> = versions
                    .flatten()
                    .map(|entry| entry.path())
                    .filter(|path| path.is_dir())
                    .collect();
                version_dirs.sort();
                for version in version_dirs.iter().rev() {
                    push_file(&mut candidates, version.join("schema"), &name);
                }
            }
        }
    }
    candidates
}

fn provider_from_catalog(path: &Path, schema_id: &str) -> Result<InMemorySchemaProvider, Fail> {
    let bytes = std::fs::read(path).map_err(|error| Fail::Status(IO, error.to_string()))?;
    let catalog =
        parse_schema_catalog(&bytes, SchemaCatalogLimits::default()).map_err(Fail::Parse)?;
    if catalog.schema_id != schema_id {
        return Err(Fail::Status(
            UNSUPPORTED,
            format!(
                "schema catalog {} declares {}, expected {schema_id}",
                path.display(),
                catalog.schema_id
            ),
        ));
    }
    let mut provider = InMemorySchemaProvider::new();
    for definition in catalog.definitions {
        provider.insert(schema_id, definition);
    }
    Ok(provider)
}

fn embedded_provider(schema_key: &str, base: &str) -> Result<InMemorySchemaProvider, Fail> {
    let mut last_error = None;
    for path in catalog_candidates(base) {
        if !path.is_file() {
            continue;
        }
        match provider_from_catalog(&path, base) {
            Ok(provider) => return Ok(provider),
            Err(error) => last_error = Some(error),
        }
    }
    if let Some(error) = last_error {
        return Err(error);
    }
    Err(Fail::Status(
        UNSUPPORTED,
        format!("schema catalog sch_{base}.sch_txt was not found for {schema_key}"),
    ))
}

fn field<'a>(node: &'a RawNode, name: &str) -> Option<&'a parasolid_core::RawField> {
    node.fields.iter().find(|item| item.definition.name == name)
}

fn pointer(node: &RawNode, name: &str) -> Option<u32> {
    match field(node, name).and_then(|item| item.values.first()) {
        Some(FieldValue::PointerIndex(index)) if *index != 0 => Some(*index),
        _ => None,
    }
}

fn node_identifier(node: &RawNode) -> i32 {
    match field(node, "node_id").and_then(|item| item.values.first()) {
        Some(FieldValue::Integer(Some(value))) => *value,
        _ => -1,
    }
}

fn transform_matrix(node: Option<&RawNode>) -> [[f64; 4]; 4] {
    let Some(node) = node else {
        return identity_matrix();
    };
    let mut rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    if let Some(field) = field(node, "rotation_matrix") {
        let values = field
            .values
            .iter()
            .filter_map(|value| match value {
                FieldValue::Double(Some(item)) => Some(*item),
                _ => None,
            })
            .collect::<Vec<_>>();
        if values.len() == 9 {
            rotation.copy_from_slice(&values);
        }
    }
    let translation = match field(node, "translation_vector").and_then(|item| item.values.first()) {
        Some(FieldValue::Vector([Some(x), Some(y), Some(z)])) => [*x, *y, *z],
        _ => [0.0, 0.0, 0.0],
    };
    let scale = match field(node, "scale").and_then(|item| item.values.first()) {
        Some(FieldValue::Double(Some(value))) if *value != 0.0 => *value,
        _ => 1.0,
    };
    // XT applies scale to the whole rigid motion: p' = s(Rp + t).
    [
        [rotation[0] * scale, rotation[1] * scale, rotation[2] * scale, translation[0] * scale],
        [rotation[3] * scale, rotation[4] * scale, rotation[5] * scale, translation[1] * scale],
        [rotation[6] * scale, rotation[7] * scale, rotation[8] * scale, translation[2] * scale],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn identity_matrix() -> [[f64; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn multiply(left: [[f64; 4]; 4], right: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for row in 0..4 {
        for column in 0..4 {
            out[row][column] = left[row][0] * right[0][column]
                + left[row][1] * right[1][column]
                + left[row][2] * right[2][column]
                + left[row][3] * right[3][column];
        }
    }
    out
}

#[allow(clippy::cast_possible_truncation)]
fn coin_matrix(matrix: [[f64; 4]; 4]) -> [f32; 16] {
    [
        matrix[0][0] as f32,
        matrix[1][0] as f32,
        matrix[2][0] as f32,
        0.0,
        matrix[0][1] as f32,
        matrix[1][1] as f32,
        matrix[2][1] as f32,
        0.0,
        matrix[0][2] as f32,
        matrix[1][2] as f32,
        matrix[2][2] as f32,
        0.0,
        matrix[0][3] as f32,
        matrix[1][3] as f32,
        matrix[2][3] as f32,
        1.0,
    ]
}

struct InstanceNode {
    part: u32,
    part_type: u16,
    assembly: u32,
    node_id: i32,
    local: [[f64; 4]; 4],
}

fn assemble(nodes: &[RawNode], model: &BrepModel) -> Vec<mesh::Placement> {
    let mut at = std::collections::BTreeMap::<u32, usize>::new();
    for (index, node) in nodes.iter().enumerate() {
        at.insert(node.index, index);
    }
    let mut body_ordinal = std::collections::BTreeMap::<u32, u32>::new();
    for (ordinal, body) in model.bodies.iter().enumerate() {
        body_ordinal.insert(body.source.node_index, u32::try_from(ordinal).unwrap_or(u32::MAX));
    }
    let mut instances = Vec::new();
    let mut referenced = std::collections::BTreeSet::<u32>::new();
    for node in nodes {
        if node.node_type != 11 {
            continue;
        }
        let Some(part) = pointer(node, "part") else {
            continue;
        };
        let Some(part_index) = at.get(&part) else {
            continue;
        };
        let part_type = nodes[*part_index].node_type;
        if part_type == 10 {
            referenced.insert(part);
        }
        let local = transform_matrix(pointer(node, "transform").and_then(|index| at.get(&index)).map(|index| &nodes[*index]));
        instances.push(InstanceNode {
            part,
            part_type,
            assembly: pointer(node, "assembly").unwrap_or(0),
            node_id: node_identifier(node),
            local,
        });
    }
    if instances.is_empty() {
        return Vec::new();
    }
    let mut roots = Vec::new();
    for node in nodes {
        if node.node_type == 10 && !referenced.contains(&node.index) {
            roots.push(node.index);
        }
    }
    let mut placed = Vec::new();
    let visit = |assembly: u32, parent: [[f64; 4]; 4], depth: u32, placed: &mut Vec<mesh::Placement>| {
        fn walk(
            instances: &[InstanceNode],
            body_ordinal: &std::collections::BTreeMap<u32, u32>,
            assembly: u32,
            parent: [[f64; 4]; 4],
            depth: u32,
            placed: &mut Vec<mesh::Placement>,
        ) {
            if depth > 16 {
                return;
            }
            for instance in instances.iter().filter(|item| item.assembly == assembly) {
                let world = multiply(parent, instance.local);
                if instance.part_type == 12 {
                    let Some(body) = body_ordinal.get(&instance.part) else {
                        continue;
                    };
                    placed.push(mesh::Placement {
                        body: *body,
                        node_id: instance.node_id,
                        matrix: coin_matrix(world),
                    });
                } else if instance.part_type == 10 {
                    walk(instances, body_ordinal, instance.part, world, depth + 1, placed);
                }
            }
        }
        walk(&instances, &body_ordinal, assembly, parent, depth, placed);
    };
    if roots.is_empty() {
        visit(0, identity_matrix(), 0, &mut placed);
    } else {
        for root in roots {
            visit(root, identity_matrix(), 0, &mut placed);
        }
    }
    placed
}

fn load_brep(data: &[u8], format: SourceFormat) -> Result<parasolid_core::brep::BrepModel, Fail> {
    Ok(load_scene(data, format)?.0)
}

fn load_scene(data: &[u8], format: SourceFormat) -> Result<(BrepModel, Vec<mesh::Placement>), Fail> {
    let (document_limits, inspection) = limits();
    match format {
        SourceFormat::Binary => {
            let header = parasolid_core::inspect_xb(data, inspection).map_err(Fail::Parse)?;
            let key = SchemaKey::parse(&header.schema_key).map_err(Fail::Parse)?;
            let registry = BuiltinProfileRegistry::compiled().map_err(Fail::Parse)?;
            if let Some(provider) = registry.provider_for_key(&key) {
                let document = parse_xb(data, &provider, document_limits).map_err(Fail::Parse)?;
                if document.schema_provider == SchemaProviderResolution::CallerSupplied {
                    return Err(unsupported_key(&header.schema_key));
                }
                let model = map_xb_brep(&document).map_err(Fail::Parse)?;
                let placements = assemble(&document.nodes, &model);
                return Ok((model, placements));
            }
            let Some(base) = key.base() else {
                return Err(unsupported_key(&header.schema_key));
            };
            let provider = embedded_provider(&header.schema_key, base)?;
            let document = parse_xb(data, &provider, document_limits).map_err(Fail::Parse)?;
            let model = map_xb_brep(&document).map_err(Fail::Parse)?;
            let placements = assemble(&document.nodes, &model);
            Ok((model, placements))
        }
        SourceFormat::Text => {
            let header = parasolid_core::inspect_xt(data, inspection).map_err(Fail::Parse)?;
            let key = SchemaKey::parse(&header.schema_key).map_err(Fail::Parse)?;
            let registry = BuiltinProfileRegistry::compiled().map_err(Fail::Parse)?;
            if let Some(provider) = registry.provider_for_key(&key) {
                let document = parse_xt(data, &provider, document_limits).map_err(Fail::Parse)?;
                if document.schema_provider == SchemaProviderResolution::CallerSupplied {
                    return Err(unsupported_key(&header.schema_key));
                }
                let model = map_xt_brep(&document).map_err(Fail::Parse)?;
                let placements = assemble(&document.nodes, &model);
                return Ok((model, placements));
            }
            let Some(base) = key.base() else {
                return Err(unsupported_key(&header.schema_key));
            };
            let provider = embedded_provider(&header.schema_key, base)?;
            let document = parse_xt(data, &provider, document_limits).map_err(Fail::Parse)?;
            let model = map_xt_brep(&document).map_err(Fail::Parse)?;
            let placements = assemble(&document.nodes, &model);
            Ok((model, placements))
        }
    }
}

fn summary_json(data: &[u8], format: SourceFormat) -> Result<String, Fail> {
    let model = load_brep(data, format)?;
    Ok(format!(
        "{{\"format\":\"{format}\",\"schema_key\":\"{schema}\",\"complete\":{complete},\"bodies\":{bodies},\"faces\":{faces},\"edges\":{edges},\"vertices\":{vertices},\"diagnostics\":{diagnostics}}}",
        format = model.source_format.as_str(),
        schema = json_escape(&model.schema_key),
        complete = if model.complete { "true" } else { "false" },
        bodies = model.bodies.len(),
        faces = model.faces.len(),
        edges = model.edges.len(),
        vertices = model.vertices.len(),
        diagnostics = model.diagnostics.len(),
    ))
}

fn finish(result: Result<String, Fail>, out_json: *mut *mut c_char, out_err: *mut *mut c_char) -> c_int {
    match result {
        Ok(json) => {
            write_slot(out_json, json);
            clear_slot(out_err);
            OK
        }
        Err(error) => {
            clear_slot(out_json);
            let status = error.status();
            write_slot(out_err, error.message());
            status
        }
    }
}

fn run(
    result: Result<String, Fail>,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    if let Ok(status) = catch_unwind(AssertUnwindSafe(|| finish(result, out_json, out_err))) {
        status
    } else {
        clear_slot(out_json);
        write_slot(out_err, "parasolid panicked".into());
        PANIC
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr().cast()
}

#[unsafe(no_mangle)]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn parasolid_string_free(value: *mut c_char) {
    if value.is_null() {
        return;
    }
    let _ = catch_unwind(AssertUnwindSafe(|| unsafe { drop(CString::from_raw(value)) }));
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_format_from_bytes(data: *const u8, len: usize) -> c_int {
    let Ok(data) = bytes(data, len) else {
        return 0;
    };
    detect_format(None, data).map_or(0, format_code)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_format_from_extension(ext: *const c_char) -> c_int {
    let Ok(ext) = c_str(ext) else {
        return 0;
    };
    let name = ext.trim_start_matches('.').to_ascii_lowercase();
    match name.as_str() {
        "x_b" | "xb" => FORMAT_XB,
        "x_t" | "xt" => FORMAT_XT,
        _ => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_format_from_path(path: *const c_char) -> c_int {
    let Ok(path) = c_str(path) else {
        return 0;
    };
    path_format(path).map_or(0, format_code)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_inspect(
    path: *const c_char,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    let path = match c_str(path) {
        Ok(path) => path,
        Err(status) => {
            clear_slot(out_json);
            write_slot(out_err, "path is null or not utf-8".into());
            return status;
        }
    };
    let result = (|| {
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        inspect_json(&data, format)
    })();
    run(result, out_json, out_err)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_inspect_bytes(
    data: *const u8,
    len: usize,
    format: c_int,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    let data = match bytes(data, len) {
        Ok(data) => data,
        Err(status) => {
            clear_slot(out_json);
            write_slot(out_err, "bytes pointer is null".into());
            return status;
        }
    };
    let result = (|| {
        let format = format_from_code(format, None, data)?;
        inspect_json(data, format)
    })();
    run(result, out_json, out_err)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_summary(
    path: *const c_char,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    let path = match c_str(path) {
        Ok(path) => path,
        Err(status) => {
            clear_slot(out_json);
            write_slot(out_err, "path is null or not utf-8".into());
            return status;
        }
    };
    let result = (|| {
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        summary_json(&data, format)
    })();
    run(result, out_json, out_err)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_summary_bytes(
    data: *const u8,
    len: usize,
    format: c_int,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    let data = match bytes(data, len) {
        Ok(data) => data,
        Err(status) => {
            clear_slot(out_json);
            write_slot(out_err, "bytes pointer is null".into());
            return status;
        }
    };
    let result = (|| {
        let format = format_from_code(format, None, data)?;
        summary_json(data, format)
    })();
    run(result, out_json, out_err)
}

fn mesh_bytes(data: &[u8], format: SourceFormat) -> Result<Vec<u8>, Fail> {
    let (model, placements) = load_scene(data, format)?;
    if model.bodies.is_empty() {
        return Err(Fail::Status(MALFORMED, "document contains no bodies".into()));
    }
    mesh::tessellate_placed(&model, &placements).map_err(|message| Fail::Status(LIMIT, message))
}

fn take_bytes(bytes: Vec<u8>) -> (*mut u8, usize) {
    let boxed = bytes.into_boxed_slice();
    let len = boxed.len();
    (Box::into_raw(boxed).cast::<u8>(), len)
}

fn write_bytes(slot: *mut *mut u8, size: *mut usize, bytes: Vec<u8>) {
    if slot.is_null() || size.is_null() {
        return;
    }
    let (ptr, len) = take_bytes(bytes);
    unsafe {
        *slot = ptr;
        *size = len;
    }
}

fn clear_bytes(slot: *mut *mut u8, size: *mut usize) {
    if !slot.is_null() {
        unsafe { *slot = std::ptr::null_mut() };
    }
    if !size.is_null() {
        unsafe { *size = 0 };
    }
}

fn finish_mesh(result: Result<Vec<u8>, Fail>, out_bytes: *mut *mut u8, out_size: *mut usize, out_err: *mut *mut c_char) -> c_int {
    if out_size.is_null() {
        clear_bytes(out_bytes, out_size);
        write_slot(out_err, "mesh size pointer is null".into());
        return INVALID;
    }
    match result {
        Ok(bytes) => {
            write_bytes(out_bytes, out_size, bytes);
            clear_slot(out_err);
            OK
        }
        Err(error) => {
            clear_bytes(out_bytes, out_size);
            let status = error.status();
            write_slot(out_err, error.message());
            status
        }
    }
}

fn run_mesh(result: Result<Vec<u8>, Fail>, out_bytes: *mut *mut u8, out_size: *mut usize, out_err: *mut *mut c_char) -> c_int {
    if let Ok(status) = catch_unwind(AssertUnwindSafe(|| finish_mesh(result, out_bytes, out_size, out_err))) {
        status
    } else {
        clear_bytes(out_bytes, out_size);
        write_slot(out_err, "parasolid panicked".into());
        PANIC
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_mesh(
    path: *const c_char,
    out_bytes: *mut *mut u8,
    out_size: *mut usize,
    out_err: *mut *mut c_char,
) -> c_int {
    let path = match c_str(path) {
        Ok(path) => path,
        Err(status) => {
            clear_bytes(out_bytes, out_size);
            write_slot(out_err, "path is null or not utf-8".into());
            return status;
        }
    };
    let result = (|| {
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        mesh_bytes(&data, format)
    })();
    run_mesh(result, out_bytes, out_size, out_err)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_mesh_bytes(
    data: *const u8,
    len: usize,
    format: c_int,
    out_bytes: *mut *mut u8,
    out_size: *mut usize,
    out_err: *mut *mut c_char,
) -> c_int {
    let data = match bytes(data, len) {
        Ok(data) => data,
        Err(status) => {
            clear_bytes(out_bytes, out_size);
            write_slot(out_err, "bytes pointer is null".into());
            return status;
        }
    };
    let result = (|| {
        let format = format_from_code(format, None, data)?;
        mesh_bytes(data, format)
    })();
    run_mesh(result, out_bytes, out_size, out_err)
}

#[unsafe(no_mangle)]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn parasolid_bytes_free(bytes: *mut u8, size: usize) {
    if bytes.is_null() || size == 0 {
        return;
    }
    let _ = catch_unwind(AssertUnwindSafe(|| unsafe {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(bytes, size)));
    }));
}

#[cfg(test)]
mod tests {
    use super::{
        parasolid_format_from_bytes, parasolid_format_from_extension, parasolid_inspect,
        parasolid_inspect_bytes, parasolid_string_free, parasolid_summary_bytes, parasolid_version,
        FORMAT_XB, FORMAT_XT, MALFORMED, OK,
    };
    use std::ffi::CStr;

    #[test]
    fn version_is_nonempty() {
        let version = unsafe { CStr::from_ptr(parasolid_version()) };
        assert!(version.to_bytes().starts_with(b"0."));
    }

    #[test]
    fn extension_and_signature_select_binary() {
        assert_eq!(parasolid_format_from_extension(c".x_b".as_ptr()), FORMAT_XB);
        assert_eq!(parasolid_format_from_extension(c"xt".as_ptr()), FORMAT_XT);
        let bytes = b"PS\0\0";
        assert_eq!(parasolid_format_from_bytes(bytes.as_ptr(), bytes.len()), FORMAT_XB);
    }

    #[test]
    fn garbage_inspect_reports_an_error() {
        let bytes = b"not a parasolid file";
        let mut json = std::ptr::null_mut();
        let mut err = std::ptr::null_mut();
        let status = parasolid_inspect_bytes(bytes.as_ptr(), bytes.len(), 0, &raw mut json, &raw mut err);
        assert_ne!(status, 0);
        assert!(json.is_null());
        assert!(!err.is_null());
        parasolid_string_free(err);
        parasolid_string_free(std::ptr::null_mut());
    }

    #[test]
    fn truncated_binary_is_malformed() {
        let bytes = b"PS\0\0";
        let mut json = std::ptr::null_mut();
        let mut err = std::ptr::null_mut();
        let status =
            parasolid_summary_bytes(bytes.as_ptr(), bytes.len(), FORMAT_XB, &raw mut json, &raw mut err);
        assert_eq!(status, MALFORMED);
        assert!(json.is_null());
        parasolid_string_free(err);
    }

    #[test]
    fn synthetic_text_file_inspects() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../corpus/generated/synthetic/release-v1/values.x_t\0"
        );
        let mut json = std::ptr::null_mut();
        let mut err = std::ptr::null_mut();
        let status = parasolid_inspect(path.as_ptr().cast(), &raw mut json, &raw mut err);
        let message = if err.is_null() {
            String::new()
        } else {
            let text = unsafe { CStr::from_ptr(err) }.to_string_lossy().into_owned();
            parasolid_string_free(err);
            text
        };
        assert_eq!(status, OK, "{message}");
        let body = unsafe { CStr::from_ptr(json) }.to_string_lossy().into_owned();
        parasolid_string_free(json);
        assert!(body.contains("\"format\":\"text\""));
        assert!(body.contains("SCH_"));
    }

    #[test]
    fn filament_housing_meshes() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../tests/3d/Global_FilamentHousing.x_t"
        );
        let data = std::fs::read(path).unwrap_or_default();
        assert!(!data.is_empty(), "missing fixture");
        let model = match super::load_brep(&data, super::SourceFormat::Text) {
            Ok(model) => model,
            Err(error) => {
                let message = error.message();
                assert!(
                    message.contains("was not found"),
                    "{message}"
                );
                return;
            }
        };
        let detail = format!(
            "bodies {} faces {} complete {}",
            model.bodies.len(),
            model.faces.len(),
            model.complete
        );
        assert!(!model.bodies.is_empty(), "{detail}");
        assert!(!model.faces.is_empty(), "{detail}");
        let mesh = match super::mesh_bytes(&data, super::SourceFormat::Text) {
            Ok(mesh) => mesh,
            Err(error) => {
                let message = error.message();
                assert!(message.contains("was not found"), "{message}");
                return;
            }
        };
        let triangles = mesh_triangles(&mesh);
        assert!(triangles > 0, "{detail} triangles {triangles}");
        let shift = max_translation(&mesh);
        assert!(shift > 0.2, "{detail} translation {shift}");
    }


    fn mesh_triangles(mesh: &[u8]) -> u32 {
        if mesh.len() < 16 {
            return 0;
        }
        let bodies = u32::from_le_bytes([mesh[12], mesh[13], mesh[14], mesh[15]]);
        let mut offset = 16_usize;
        let mut total = 0_u32;
        for _ in 0..bodies {
            if offset + 40 > mesh.len() {
                return total;
            }
            let count = u32::from_le_bytes([
                mesh[offset + 12],
                mesh[offset + 13],
                mesh[offset + 14],
                mesh[offset + 15],
            ]);
            total = total.saturating_add(count);
            let tri_bytes = usize::try_from(count).unwrap_or(0).saturating_mul(72);
            offset = offset.saturating_add(40).saturating_add(tri_bytes);
        }
        total
    }

    fn max_translation(mesh: &[u8]) -> f32 {
        if mesh.len() < 20 || u32::from_le_bytes([mesh[4], mesh[5], mesh[6], mesh[7]]) != 2 {
            return 0.0;
        }
        let bodies = u32::from_le_bytes([mesh[12], mesh[13], mesh[14], mesh[15]]);
        let mut offset = 16_usize;
        for _ in 0..bodies {
            if offset + 40 > mesh.len() {
                return 0.0;
            }
            let count = u32::from_le_bytes([
                mesh[offset + 12],
                mesh[offset + 13],
                mesh[offset + 14],
                mesh[offset + 15],
            ]);
            let tri_bytes = usize::try_from(count).unwrap_or(0).saturating_mul(72);
            offset = offset.saturating_add(40).saturating_add(tri_bytes);
        }
        if offset + 4 > mesh.len() {
            return 0.0;
        }
        let copies = u32::from_le_bytes([
            mesh[offset],
            mesh[offset + 1],
            mesh[offset + 2],
            mesh[offset + 3],
        ]);
        offset += 4;
        let mut max_shift = 0.0_f32;
        for _ in 0..copies {
            if offset + 8 + 64 > mesh.len() {
                break;
            }
            offset += 8;
            for axis in 0..3 {
                let start = offset + (12 + axis) * 4;
                let value = f32::from_le_bytes([
                    mesh[start],
                    mesh[start + 1],
                    mesh[start + 2],
                    mesh[start + 3],
                ]);
                max_shift = max_shift.max(value.abs());
            }
            offset += 64;
        }
        max_shift
    }

}
