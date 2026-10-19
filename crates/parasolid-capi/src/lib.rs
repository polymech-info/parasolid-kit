//! C ABI compiled into parasolid.dll. Callers free returned strings with
//! `parasolid_string_free` so the allocation stays on this crate's allocator.
//!
//! The library inspects, parses, and tessellates transmit files for the viewer.

#![allow(unsafe_code)]

mod mesh;

use std::collections::{HashMap, HashSet};
use std::ffi::{CStr, CString};
use std::fmt::Write as _;
use std::os::raw::{c_char, c_int};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::SystemTime;

use parasolid_core::brep::{map_xb_brep, map_xt_brep, BrepModel};
use parasolid_core::{
    parse_schema_catalog, parse_xb, parse_xt, BuiltinProfileRegistry, DocumentLimits, ErrorKind,
    FieldValue, InMemorySchemaProvider, InspectionLimits, ParseError, RawNode, SchemaCatalogLimits,
    SchemaKey, SchemaProvider, SchemaProviderResolution, TypeDefinition,
};

const OK: c_int = 0;
const UNSUPPORTED: c_int = 1;
const MALFORMED: c_int = 2;
const LIMIT: c_int = 3;
const IO: c_int = 4;
const PANIC: c_int = 5;
const INVALID: c_int = 6;
const PARTIAL: c_int = 7;

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

/// # Safety
/// `ptr` must be valid for `len` bytes when it is non-null. `len` may be 0 with a null pointer.
unsafe fn bytes<'a>(ptr: *const u8, len: usize) -> Result<&'a [u8], c_int> {
    let max = DocumentLimits::default().max_file_size;
    if len > isize::MAX as usize {
        return Err(INVALID);
    }
    if len > max {
        return Err(LIMIT);
    }
    if ptr.is_null() {
        return if len == 0 { Ok(&[]) } else { Err(INVALID) };
    }
    Ok(unsafe { std::slice::from_raw_parts(ptr, len) })
}

/// # Safety
/// `ptr` must be null or a NUL-terminated UTF-8 string no longer than 1 MiB.
unsafe fn c_str<'a>(ptr: *const c_char) -> Result<&'a str, c_int> {
    if ptr.is_null() {
        return Err(INVALID);
    }
    let text = unsafe { CStr::from_ptr(ptr) };
    if text.to_bytes().len() > isize::MAX as usize {
        return Err(INVALID);
    }
    if text.to_bytes().len() > 1024 * 1024 {
        return Err(LIMIT);
    }
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

fn push_layout(candidates: &mut Vec<PathBuf>, dir: &Path, name: &str) {
    push_file(candidates, dir.join("schema"), name);
    push_file(candidates, dir, name);
}

/// Directory of the loaded DLL. Catalog files are not part of this crate.
/// A host may place `sch_*.sch_txt` beside the DLL or under its `schema` folder.
#[cfg(windows)]
fn loaded_library_dir() -> Option<PathBuf> {
    use std::os::windows::ffi::OsStringExt;

    unsafe extern "system" {
        fn GetModuleHandleExW(flags: u32, address: *const u16, module: *mut *mut std::ffi::c_void) -> i32;
        fn GetModuleFileNameW(module: *mut std::ffi::c_void, filename: *mut u16, size: u32) -> u32;
    }

    const FROM_ADDRESS: u32 = 0x4;
    const UNCHANGED_REFCOUNT: u32 = 0x2;
    let mut module = std::ptr::null_mut();
    let anchor = loaded_library_dir as *const () as *const u16;
    if unsafe { GetModuleHandleExW(FROM_ADDRESS | UNCHANGED_REFCOUNT, anchor, &mut module) } == 0 {
        return None;
    }
    let mut buffer = vec![0_u16; 32_768];
    let length = unsafe { GetModuleFileNameW(module, buffer.as_mut_ptr(), buffer.len() as u32) };
    if length == 0 || length as usize >= buffer.len() {
        return None;
    }
    let text = std::ffi::OsString::from_wide(&buffer[..length as usize]);
    Path::new(&text).parent().map(Path::to_path_buf)
}

/// Exact `sch_<id>.sch_txt` locations. Built-in profiles do not cover every
/// embedded-base key. Files such as SolidWorks base `13006` need the catalog
/// shipped beside Tanit or this DLL.
fn catalog_candidates(schema_id: &str) -> Vec<PathBuf> {
    let name = format!("sch_{schema_id}.sch_txt");
    let mut candidates = Vec::new();
    if let Ok(dir) = std::env::var("PARASOLID_SCHEMA_DIR") {
        push_file(&mut candidates, dir, &name);
    }
    if let Ok(exe) = std::env::current_exe()
        && let Some(dir) = exe.parent()
    {
        push_layout(&mut candidates, dir, &name);
    }
    #[cfg(windows)]
    if let Some(dir) = loaded_library_dir() {
        push_layout(&mut candidates, &dir, &name);
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

struct SharedCatalog(Arc<InMemorySchemaProvider>);

impl SchemaProvider for SharedCatalog {
    fn contains_schema(&self, schema: &str) -> bool {
        self.0.contains_schema(schema)
    }

    fn type_definition(&self, schema: &str, node_type: u16) -> Option<&TypeDefinition> {
        self.0.type_definition(schema, node_type)
    }
}

struct CachedCatalog {
    modified: Option<SystemTime>,
    len: u64,
    provider: Arc<InMemorySchemaProvider>,
}

fn catalog_cache() -> &'static Mutex<HashMap<PathBuf, CachedCatalog>> {
    static CACHE: OnceLock<Mutex<HashMap<PathBuf, CachedCatalog>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

fn cached_catalog(path: &Path, schema_id: &str) -> Result<SharedCatalog, Fail> {
    let meta = std::fs::metadata(path).map_err(|error| Fail::Status(IO, error.to_string()))?;
    let modified = meta.modified().ok();
    let len = meta.len();
    {
        let cache = catalog_cache().lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(hit) = cache.get(path)
            && hit.modified == modified
            && hit.len == len
        {
            return Ok(SharedCatalog(Arc::clone(&hit.provider)));
        }
    }
    let provider = Arc::new(provider_from_catalog(path, schema_id)?);
    let mut cache = catalog_cache().lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    if let Some(hit) = cache.get(path)
        && hit.modified == modified
        && hit.len == len
    {
        return Ok(SharedCatalog(Arc::clone(&hit.provider)));
    }
    cache.insert(
        path.to_path_buf(),
        CachedCatalog {
            modified,
            len,
            provider: Arc::clone(&provider),
        },
    );
    Ok(SharedCatalog(provider))
}

fn embedded_provider(schema_key: &str, base: &str) -> Result<SharedCatalog, Fail> {
    let mut last_error = None;
    for path in catalog_candidates(base) {
        if !path.is_file() {
            continue;
        }
        match cached_catalog(&path, base) {
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

struct NodeRefs {
    part: Option<u32>,
    assembly: Option<u32>,
    transform: Option<u32>,
    node_id: i32,
}

fn pointer_index(values: &[FieldValue]) -> Option<u32> {
    match values.first() {
        Some(FieldValue::PointerIndex(index)) if *index != 0 => Some(*index),
        _ => None,
    }
}

/// One pass over the instance fields. Name checks stay, but each node is scanned once.
fn node_refs(node: &RawNode) -> NodeRefs {
    let mut refs = NodeRefs {
        part: None,
        assembly: None,
        transform: None,
        node_id: -1,
    };
    for item in &node.fields {
        match item.definition.name.as_str() {
            "part" => refs.part = pointer_index(&item.values),
            "assembly" => refs.assembly = pointer_index(&item.values),
            "transform" => refs.transform = pointer_index(&item.values),
            "node_id" => {
                if let Some(FieldValue::Integer(Some(value))) = item.values.first() {
                    refs.node_id = *value;
                }
            }
            _ => {}
        }
    }
    refs
}

#[allow(clippy::cast_possible_truncation)]
fn finite_component(value: f64) -> bool {
    value.is_finite() && (value as f32).is_finite()
}

/// `Some(None)` is an absent transform: identity, and the instance stays placed.
/// `None` means the transform node is not usable, so the body is listed as unplaced.
fn instance_transform(node: Option<&RawNode>) -> Option<Option<[[f64; 4]; 4]>> {
    let Some(node) = node else {
        return Some(None);
    };
    if node.definition.name != "TRANSFORM" {
        return None;
    }
    transform_matrix(node).map(Some)
}

fn transform_matrix(node: &RawNode) -> Option<[[f64; 4]; 4]> {
    let mut rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    if let Some(field) = field(node, "rotation_matrix") {
        let mut values = [0.0; 9];
        let mut count = 0usize;
        for value in &field.values {
            if count == 9 {
                return None;
            }
            match value {
                FieldValue::Double(Some(item)) if finite_component(*item) => {
                    values[count] = *item;
                    count += 1;
                }
                _ => return None,
            }
        }
        if count != 9 {
            return None;
        }
        rotation = values;
    }
    let translation = match field(node, "translation_vector").and_then(|item| item.values.first()) {
        None => [0.0, 0.0, 0.0],
        Some(FieldValue::Vector([Some(x), Some(y), Some(z)]))
            if finite_component(*x) && finite_component(*y) && finite_component(*z) =>
        {
            [*x, *y, *z]
        }
        _ => return None,
    };
    let scale = match field(node, "scale").and_then(|item| item.values.first()) {
        None | Some(FieldValue::Double(None)) => 1.0,
        Some(FieldValue::Double(Some(value))) if finite_component(*value) && *value != 0.0 => *value,
        _ => return None,
    };
    // XT applies scale to the whole rigid motion: p' = s(Rp + t).
    let matrix = [
        [rotation[0] * scale, rotation[1] * scale, rotation[2] * scale, translation[0] * scale],
        [rotation[3] * scale, rotation[4] * scale, rotation[5] * scale, translation[1] * scale],
        [rotation[6] * scale, rotation[7] * scale, rotation[8] * scale, translation[2] * scale],
        [0.0, 0.0, 0.0, 1.0],
    ];
    matrix_drawable(matrix).then_some(matrix)
}

fn identity_matrix() -> [[f64; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn matrix_drawable(matrix: [[f64; 4]; 4]) -> bool {
    matrix.iter().flatten().copied().all(finite_component)
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

#[derive(Clone, Copy)]
struct InstanceNode {
    /// Assembly or body node this instance points at.
    part: u32,
    part_type: u16,
    assembly: u32,
    node_id: i32,
    /// `None` with `invalid` means the transform could not be used.
    /// `Some(identity)` is a real identity placement.
    local: Option<[[f64; 4]; 4]>,
    body: Option<u32>,
    invalid: bool,
    missing_part: bool,
}

struct Assembly {
    placed: Vec<mesh::Placement>,
    unplaced: Vec<mesh::Placement>,
    standalone: bool,
    missing: u32,
    cycles: u32,
    truncated: bool,
}

const MAX_DEPTH: u32 = 16;
const MAX_INSTANCES: usize = 1_000_000;

fn assemble(nodes: &[RawNode], model: &BrepModel) -> Assembly {
    let mut at = HashMap::<u32, usize>::new();
    for (index, node) in nodes.iter().enumerate() {
        at.insert(node.index, index);
    }
    let mut body_ordinal = HashMap::<u32, u32>::new();
    for (ordinal, body) in model.bodies.iter().enumerate() {
        body_ordinal.insert(body.source.node_index, u32::try_from(ordinal).unwrap_or(u32::MAX));
    }
    let mut instances = Vec::new();
    let mut referenced = HashSet::<u32>::new();
    for node in nodes {
        if node.node_type != 11 {
            continue;
        }
        let refs = node_refs(node);
        let assembly = refs.assembly.unwrap_or(0);
        let node_id = refs.node_id;
        let Some(part) = refs.part else {
            instances.push(InstanceNode {
                part: 0,
                part_type: 0,
                assembly,
                node_id,
                local: None,
                body: None,
                invalid: false,
                missing_part: true,
            });
            continue;
        };
        let Some(&part_index) = at.get(&part) else {
            instances.push(InstanceNode {
                part,
                part_type: 0,
                assembly,
                node_id,
                local: None,
                body: None,
                invalid: false,
                missing_part: true,
            });
            continue;
        };
        let part_type = nodes[part_index].node_type;
        if part_type == 10 {
            referenced.insert(part);
        }
        let transform = refs.transform.and_then(|index| at.get(&index).map(|slot| &nodes[*slot]));
        let (local, invalid) = match instance_transform(transform) {
            Some(None) => (Some(identity_matrix()), false),
            Some(Some(matrix)) => (Some(matrix), false),
            None => (None, true),
        };
        let body = if part_type == 12 { body_ordinal.get(&part).copied() } else { None };
        instances.push(InstanceNode {
            part,
            part_type,
            assembly,
            node_id,
            local,
            body,
            invalid,
            missing_part: part_type != 10 && part_type != 12,
        });
    }
    let mut roots = Vec::new();
    for node in nodes {
        if node.node_type == 10 && !referenced.contains(&node.index) {
            roots.push(node.index);
        }
    }
    layout_instances(instances, roots)
}

fn layout_instances(instances: Vec<InstanceNode>, roots: Vec<u32>) -> Assembly {
    if instances.is_empty() {
        return Assembly {
            placed: Vec::new(),
            unplaced: Vec::new(),
            standalone: true,
            missing: 0,
            cycles: 0,
            truncated: false,
        };
    }
    let mut children = HashMap::<u32, Vec<usize>>::new();
    for (index, instance) in instances.iter().enumerate() {
        children.entry(instance.assembly).or_default().push(index);
    }
    let mut walk = Walk {
        instances: &instances,
        children: &children,
        placed: Vec::new(),
        unplaced: Vec::new(),
        stack: Vec::new(),
        missing: 0,
        cycles: 0,
        truncated: false,
    };
    if roots.is_empty() {
        walk.descend(0, identity_matrix(), false, 0);
    } else {
        for root in roots {
            walk.descend(root, identity_matrix(), false, 0);
        }
    }
    Assembly {
        placed: walk.placed,
        unplaced: walk.unplaced,
        standalone: false,
        missing: walk.missing,
        cycles: walk.cycles,
        truncated: walk.truncated,
    }
}

struct Walk<'a> {
    instances: &'a [InstanceNode],
    children: &'a HashMap<u32, Vec<usize>>,
    placed: Vec<mesh::Placement>,
    unplaced: Vec<mesh::Placement>,
    stack: Vec<u32>,
    missing: u32,
    cycles: u32,
    truncated: bool,
}

impl Walk<'_> {
    fn descend(&mut self, assembly: u32, parent: [[f64; 4]; 4], force_unplaced: bool, depth: u32) {
        if depth > MAX_DEPTH {
            self.truncated = true;
            return;
        }
        if self.stack.contains(&assembly) {
            self.cycles = self.cycles.saturating_add(1);
            return;
        }
        self.stack.push(assembly);
        let indexes = self.children.get(&assembly).cloned().unwrap_or_default();
        for index in indexes {
            if self.placed.len().saturating_add(self.unplaced.len()) >= MAX_INSTANCES {
                self.truncated = true;
                break;
            }
            let Some(instance) = self.instances.get(index).copied() else {
                continue;
            };
            let bad = force_unplaced || instance.invalid || instance.local.is_none();
            let local = instance.local.unwrap_or_else(identity_matrix);
            let world = multiply(parent, local);
            let bad = bad || !matrix_drawable(world);
            self.place(instance, if bad { identity_matrix() } else { world }, bad, depth);
        }
        let _ = self.stack.pop();
    }

    fn place(&mut self, instance: InstanceNode, world: [[f64; 4]; 4], bad: bool, depth: u32) {
        if instance.missing_part {
            self.missing = self.missing.saturating_add(1);
            return;
        }
        if instance.part_type == 12 {
            let Some(body) = instance.body else {
                self.missing = self.missing.saturating_add(1);
                return;
            };
            let placement = mesh::Placement {
                body,
                node_id: instance.node_id,
                matrix: coin_matrix(world),
            };
            if bad {
                self.unplaced.push(placement);
            } else {
                self.placed.push(placement);
            }
            return;
        }
        if instance.part_type == 10 {
            self.descend(instance.part, world, bad, depth.saturating_add(1));
            return;
        }
        self.missing = self.missing.saturating_add(1);
    }
}

fn load_brep(data: &[u8], format: SourceFormat) -> Result<BrepModel, Fail> {
    Ok(load_mapped(data, format)?.0)
}

fn load_scene(data: &[u8], format: SourceFormat) -> Result<(BrepModel, Assembly), Fail> {
    let (model, nodes) = load_mapped(data, format)?;
    let assembly = assemble(&nodes, &model);
    Ok((model, assembly))
}

fn load_mapped(data: &[u8], format: SourceFormat) -> Result<(BrepModel, Vec<RawNode>), Fail> {
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
                return Ok((model, document.nodes));
            }
            let Some(base) = key.base() else {
                return Err(unsupported_key(&header.schema_key));
            };
            let provider = embedded_provider(&header.schema_key, base)?;
            let document = parse_xb(data, &provider, document_limits).map_err(Fail::Parse)?;
            let model = map_xb_brep(&document).map_err(Fail::Parse)?;
            Ok((model, document.nodes))
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
                return Ok((model, document.nodes));
            }
            let Some(base) = key.base() else {
                return Err(unsupported_key(&header.schema_key));
            };
            let provider = embedded_provider(&header.schema_key, base)?;
            let document = parse_xt(data, &provider, document_limits).map_err(Fail::Parse)?;
            let model = map_xt_brep(&document).map_err(Fail::Parse)?;
            Ok((model, document.nodes))
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
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
    work: impl FnOnce() -> Result<String, Fail>,
) -> c_int {
    clear_slot(out_json);
    clear_slot(out_err);
    match catch_unwind(AssertUnwindSafe(work)) {
        Ok(result) => finish(result, out_json, out_err),
        Err(_) => {
            clear_slot(out_json);
            write_slot(out_err, "parasolid panicked".into());
            PANIC
        }
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

fn input_fail(status: c_int) -> Fail {
    let message = if status == LIMIT {
        "input exceeds the size limit"
    } else {
        "input pointer is null or not utf-8"
    };
    Fail::Status(status, message.into())
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_format_from_bytes(data: *const u8, len: usize) -> c_int {
    let Ok(data) = (unsafe { bytes(data, len) }) else {
        return 0;
    };
    detect_format(None, data).map_or(0, format_code)
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_format_from_extension(ext: *const c_char) -> c_int {
    let Ok(ext) = (unsafe { c_str(ext) }) else {
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
    let Ok(path) = (unsafe { c_str(path) }) else {
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
    run(out_json, out_err, || {
        let path = unsafe { c_str(path) }.map_err(input_fail)?;
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        inspect_json(&data, format)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_inspect_bytes(
    data: *const u8,
    len: usize,
    format: c_int,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    run(out_json, out_err, || {
        let data = unsafe { bytes(data, len) }.map_err(input_fail)?;
        let format = format_from_code(format, None, data)?;
        inspect_json(data, format)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_summary(
    path: *const c_char,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    run(out_json, out_err, || {
        let path = unsafe { c_str(path) }.map_err(input_fail)?;
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        summary_json(&data, format)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_summary_bytes(
    data: *const u8,
    len: usize,
    format: c_int,
    out_json: *mut *mut c_char,
    out_err: *mut *mut c_char,
) -> c_int {
    run(out_json, out_err, || {
        let data = unsafe { bytes(data, len) }.map_err(input_fail)?;
        let format = format_from_code(format, None, data)?;
        summary_json(data, format)
    })
}

struct MeshProduct {
    bytes: Vec<u8>,
    report: Option<String>,
}

fn mesh_report(scene: &Assembly, drawn: &mesh::DrawnMesh) -> Option<String> {
    let partial = drawn.faces_approximated > 0
        || drawn.faces_omitted > 0
        || !scene.unplaced.is_empty()
        || scene.missing > 0
        || scene.cycles > 0
        || scene.truncated
        || drawn.truncated
        || drawn.bodies_rendered < drawn.bodies_total;
    if !partial {
        return None;
    }
    let (placed, total) = if scene.standalone {
        (drawn.bodies_rendered, drawn.bodies_total)
    } else {
        let placed = u32::try_from(scene.placed.len()).unwrap_or(u32::MAX);
        let unplaced = u32::try_from(scene.unplaced.len()).unwrap_or(u32::MAX);
        (placed, placed.saturating_add(unplaced))
    };
    let unplaced = u32::try_from(scene.unplaced.len()).unwrap_or(u32::MAX);
    let mut text = format!(
        "status: partial\nbodies rendered: {rendered}/{total_bodies}\nfaces approximated: {approximated}\nfaces omitted: {omitted}\ninstances placed: {placed}/{total}\ninstances unplaced: {unplaced}",
        rendered = drawn.bodies_rendered,
        total_bodies = drawn.bodies_total,
        approximated = drawn.faces_approximated,
        omitted = drawn.faces_omitted,
    );
    if scene.missing > 0 {
        let _ = write!(text, "\ninstances missing: {}", scene.missing);
    }
    if scene.cycles > 0 {
        let _ = write!(text, "\nassembly cycles stopped: {}", scene.cycles);
    }
    if scene.truncated || drawn.truncated {
        text.push_str("\ntruncated: depth, instance, or triangle limit");
    }
    Some(text)
}

fn view_report(scene: &Assembly, view: &mesh::ViewMesh) -> Option<String> {
    let partial = view.other_faces > 0
        || view.faces_omitted > 0
        || !scene.unplaced.is_empty()
        || scene.missing > 0
        || scene.cycles > 0
        || scene.truncated
        || view.truncated;
    if !partial {
        return None;
    }
    let (placed, total) = if scene.standalone {
        (view.bodies, view.bodies)
    } else {
        let placed = u32::try_from(scene.placed.len()).unwrap_or(u32::MAX);
        let unplaced = u32::try_from(scene.unplaced.len()).unwrap_or(u32::MAX);
        (placed, placed.saturating_add(unplaced))
    };
    let unplaced = u32::try_from(scene.unplaced.len()).unwrap_or(u32::MAX);
    let mut text = format!(
        "status: partial\nmesher: native occt\nbodies rendered: {rendered}/{total_bodies}\nanalytic faces: {analytic}\nother faces: {other}\nfaces omitted: {omitted}\ninstances placed: {placed}/{total}\ninstances unplaced: {unplaced}",
        rendered = view.bodies,
        total_bodies = view.bodies,
        analytic = view.analytic_faces,
        other = view.other_faces,
        omitted = view.faces_omitted,
    );
    if scene.missing > 0 {
        let _ = write!(text, "\ninstances missing: {}", scene.missing);
    }
    if scene.cycles > 0 {
        let _ = write!(text, "\nassembly cycles stopped: {}", scene.cycles);
    }
    if scene.truncated || view.truncated {
        text.push_str("\ntruncated: depth, instance, or triangle limit");
    }
    Some(text)
}

fn view_bytes(data: &[u8], format: SourceFormat) -> Result<MeshProduct, Fail> {
    let (model, scene) = load_scene(data, format)?;
    if model.bodies.is_empty() {
        return Err(Fail::Status(MALFORMED, "document contains no bodies".into()));
    }
    let view = mesh::view_scene(
        &model,
        &mesh::MeshRequest {
            placements: &scene.placed,
            unplaced: &scene.unplaced,
            standalone: scene.standalone,
        },
    );
    let report = view_report(&scene, &view);
    Ok(MeshProduct { bytes: view.bytes, report })
}

fn mesh_bytes(data: &[u8], format: SourceFormat) -> Result<MeshProduct, Fail> {
    let (model, scene) = load_scene(data, format)?;
    if model.bodies.is_empty() {
        return Err(Fail::Status(MALFORMED, "document contains no bodies".into()));
    }
    let drawn = mesh::tessellate_placed(
        &model,
        &mesh::MeshRequest {
            placements: &scene.placed,
            unplaced: &scene.unplaced,
            standalone: scene.standalone,
        },
    );
    let report = mesh_report(&scene, &drawn);
    Ok(MeshProduct { bytes: drawn.bytes, report })
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

fn finish_mesh(
    result: Result<MeshProduct, Fail>,
    out_bytes: *mut *mut u8,
    out_size: *mut usize,
    out_err: *mut *mut c_char,
) -> c_int {
    if out_bytes.is_null() || out_size.is_null() {
        clear_bytes(out_bytes, out_size);
        write_slot(out_err, "mesh output pointer is null".into());
        return INVALID;
    }
    match result {
        Ok(product) => {
            write_bytes(out_bytes, out_size, product.bytes);
            if let Some(report) = product.report {
                write_slot(out_err, report);
                PARTIAL
            } else {
                clear_slot(out_err);
                OK
            }
        }
        Err(error) => {
            clear_bytes(out_bytes, out_size);
            let status = error.status();
            write_slot(out_err, error.message());
            status
        }
    }
}

fn run_mesh(
    out_bytes: *mut *mut u8,
    out_size: *mut usize,
    out_err: *mut *mut c_char,
    work: impl FnOnce() -> Result<MeshProduct, Fail>,
) -> c_int {
    if out_bytes.is_null() || out_size.is_null() {
        clear_bytes(out_bytes, out_size);
        write_slot(out_err, "mesh output pointer is null".into());
        return INVALID;
    }
    clear_bytes(out_bytes, out_size);
    clear_slot(out_err);
    match catch_unwind(AssertUnwindSafe(work)) {
        Ok(result) => finish_mesh(result, out_bytes, out_size, out_err),
        Err(_) => {
            clear_bytes(out_bytes, out_size);
            write_slot(out_err, "parasolid panicked".into());
            PANIC
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_mesh(
    path: *const c_char,
    out_bytes: *mut *mut u8,
    out_size: *mut usize,
    out_err: *mut *mut c_char,
) -> c_int {
    run_mesh(out_bytes, out_size, out_err, || {
        let path = unsafe { c_str(path) }.map_err(input_fail)?;
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        mesh_bytes(&data, format)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn parasolid_view(
    path: *const c_char,
    out_bytes: *mut *mut u8,
    out_size: *mut usize,
    out_err: *mut *mut c_char,
) -> c_int {
    run_mesh(out_bytes, out_size, out_err, || {
        let path = unsafe { c_str(path) }.map_err(input_fail)?;
        let data = read_path(path)?;
        let format = detect_format(Some(path), &data)?;
        view_bytes(&data, format)
    })
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
    run_mesh(out_bytes, out_size, out_err, || {
        let data = unsafe { bytes(data, len) }.map_err(input_fail)?;
        let format = format_from_code(format, None, data)?;
        mesh_bytes(data, format)
    })
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
        parasolid_bytes_free, parasolid_format_from_bytes, parasolid_format_from_extension,
        parasolid_inspect, parasolid_inspect_bytes, parasolid_mesh_bytes, parasolid_string_free,
        parasolid_summary_bytes, parasolid_version, FORMAT_XB, FORMAT_XT, INVALID, LIMIT, MALFORMED,
        OK, PANIC,
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

    fn plugin_schema() -> Option<&'static str> {
        let dir = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../dist/win-x64/plugins/3d_viewer/schema"
        );
        if std::path::Path::new(dir).is_dir() { Some(dir) } else { None }
    }

    fn use_plugin_schema() -> bool {
        let Some(dir) = plugin_schema() else {
            return false;
        };
        unsafe { std::env::set_var("PARASOLID_SCHEMA_DIR", dir) };
        true
    }

    #[test]
    fn null_mesh_outputs_are_invalid() {
        let byte = [0u8];
        let status = parasolid_mesh_bytes(byte.as_ptr(), byte.len(), 0, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut());
        assert_eq!(status, INVALID);
    }

    #[test]
    fn oversized_input_is_a_limit() {
        let byte = [0u8];
        let mut json = std::ptr::null_mut();
        let mut err = std::ptr::null_mut();
        let status = parasolid_inspect_bytes(byte.as_ptr(), 256 * 1024 * 1024 + 1, 0, &raw mut json, &raw mut err);
        assert_eq!(status, LIMIT);
        assert!(json.is_null());
        parasolid_string_free(err);
    }

    #[test]
    #[allow(clippy::indexing_slicing)]
    fn mesh_panic_is_contained() {
        let mut bytes = std::ptr::null_mut();
        let mut size = 99usize;
        let mut err = std::ptr::null_mut();
        let status = super::run_mesh(&raw mut bytes, &raw mut size, &raw mut err, || {
            let items = [0u8];
            let index = std::hint::black_box(1usize);
            let _ = items[index];
            Ok(super::MeshProduct { bytes: Vec::new(), report: None })
        });
        assert_eq!(status, PANIC);
        assert!(bytes.is_null());
        assert_eq!(size, 0);
        parasolid_string_free(err);
        parasolid_bytes_free(bytes, size);
    }

    #[test]
    fn bad_transform_stays_unplaced_and_a_cycle_keeps_its_sibling() {
        let placed = super::InstanceNode {
            part: 100,
            part_type: 12,
            assembly: 1,
            node_id: 7,
            local: Some(super::identity_matrix()),
            body: Some(0),
            invalid: false,
            missing_part: false,
        };
        let cycle = super::InstanceNode {
            part: 1,
            part_type: 10,
            assembly: 1,
            node_id: 8,
            local: Some(super::identity_matrix()),
            body: None,
            invalid: false,
            missing_part: false,
        };
        let unplaced = super::InstanceNode {
            part: 101,
            part_type: 12,
            assembly: 1,
            node_id: 9,
            local: None,
            body: Some(1),
            invalid: true,
            missing_part: false,
        };
        let scene = super::layout_instances(vec![placed, cycle, unplaced], vec![1]);
        assert_eq!(scene.placed.len(), 1);
        assert_eq!(scene.placed[0].body, 0);
        assert_eq!(scene.placed[0].node_id, 7);
        assert_eq!(scene.unplaced.len(), 1);
        assert_eq!(scene.unplaced[0].body, 1);
        assert_eq!(scene.unplaced[0].node_id, 9);
        assert_eq!(scene.cycles, 1);
        assert!(!scene.standalone);
        assert!(scene.placed[0].matrix[0].abs() > 0.5);
        assert!(scene.unplaced[0].matrix[0].abs() > 0.5);
    }

    #[test]
    fn schema_catalog_is_reused() {
        let Some(dir) = plugin_schema() else {
            eprintln!("schema folder is absent; catalog cache was not checked");
            return;
        };
        let path = std::path::Path::new(dir).join("sch_13006.sch_txt");
        if !path.is_file() {
            eprintln!("sch_13006.sch_txt is absent; catalog cache was not checked");
            return;
        }
        let first = match super::cached_catalog(&path, "13006") {
            Ok(provider) => provider,
            Err(error) => {
                assert!(error.message().is_empty(), "{}", error.message());
                return;
            }
        };
        let second = match super::cached_catalog(&path, "13006") {
            Ok(provider) => provider,
            Err(error) => {
                assert!(error.message().is_empty(), "{}", error.message());
                return;
            }
        };
        assert!(std::sync::Arc::ptr_eq(&first.0, &second.0));
    }

    #[test]
    fn printhead_meshes() {
        if !use_plugin_schema() {
            eprintln!("schema folder is absent; printhead mesh was not checked");
            return;
        }
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../tests/3d/Global-Printhead-SM25.x_t"
        );
        let data = std::fs::read(path).unwrap_or_default();
        if data.is_empty() {
            eprintln!("printhead fixture is absent; mesh was not checked");
            return;
        }
        let mesh = match super::mesh_bytes(&data, super::SourceFormat::Text) {
            Ok(mesh) => mesh,
            Err(error) => {
                assert!(error.message().is_empty(), "{}", error.message());
                return;
            }
        };
        assert!(mesh_triangles(&mesh.bytes) > 0);
        let report = mesh.report.unwrap_or_default();
        assert!(report.contains("bodies rendered: 48/48"), "{report}");
        assert!(report.contains("instances placed: 54/54"), "{report}");
    }

    #[test]
    fn filament_housing_meshes() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../tests/3d/Global_FilamentHousing.x_t"
        );
        let data = std::fs::read(path).unwrap_or_default();
        if data.is_empty() || !use_plugin_schema() {
            eprintln!("housing fixture or schema folder is absent; mesh was not checked");
            return;
        }
        let mesh = match super::mesh_bytes(&data, super::SourceFormat::Text) {
            Ok(mesh) => mesh,
            Err(error) => {
                assert!(error.message().is_empty(), "{}", error.message());
                return;
            }
        };
        let triangles = mesh_triangles(&mesh.bytes);
        assert!(triangles > 0, "triangles {triangles}");
        let shift = max_translation(&mesh.bytes);
        assert!(shift > 0.2, "translation {shift}");
        let report = mesh.report.unwrap_or_default();
        assert!(report.contains("status: partial") || report.is_empty(), "{report}");
    }

    #[test]
    fn mould_body_two_keeps_its_placement() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../../tests/3d/Mould-650-15.x_t");
        let data = std::fs::read(path).unwrap_or_default();
        if data.is_empty() || !use_plugin_schema() {
            eprintln!("mould fixture or schema folder is absent; placement was not checked");
            return;
        }
        let mesh = match super::mesh_bytes(&data, super::SourceFormat::Text) {
            Ok(mesh) => mesh,
            Err(error) => {
                assert!(error.message().is_empty(), "{}", error.message());
                return;
            }
        };
        let copies = instance_records(&mesh.bytes);
        assert_eq!(copies.len(), 55, "{}", mesh.report.clone().unwrap_or_default());
        let mut matrix = [0.0_f32; 16];
        let mut saw = false;
        for (ordinal, _node, item) in &copies {
            if *ordinal == 2 {
                matrix = *item;
                saw = true;
                break;
            }
        }
        assert!(saw, "body 2 instance missing");
        let expected = [0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.002, 0.0, 1.0];
        for (got, want) in matrix.iter().zip(expected) {
            assert!((got - want).abs() < 1.0e-5, "matrix {got} wanted {want}");
        }
        let origin = apply_row(&matrix, [0.0, 0.0, 0.0]);
        assert!((origin[1] - 0.002).abs() < 1.0e-5 && origin[0].abs() < 1.0e-5 && origin[2].abs() < 1.0e-5);
        let axis = apply_row(&matrix, [1.0, 0.0, 0.0]);
        assert!(axis[0].abs() < 1.0e-4 && (axis[1] - 0.002).abs() < 1.0e-4 && (axis[2] - 1.0).abs() < 1.0e-4);
    }

    fn apply_row(matrix: &[f32; 16], point: [f32; 3]) -> [f32; 3] {
        [
            point[0] * matrix[0] + point[1] * matrix[4] + point[2] * matrix[8] + matrix[12],
            point[0] * matrix[1] + point[1] * matrix[5] + point[2] * matrix[9] + matrix[13],
            point[0] * matrix[2] + point[1] * matrix[6] + point[2] * matrix[10] + matrix[14],
        ]
    }

    fn instance_records(mesh: &[u8]) -> Vec<(u32, i32, [f32; 16])> {
        if mesh.len() < 20 || u32::from_le_bytes([mesh[4], mesh[5], mesh[6], mesh[7]]) != 2 {
            return Vec::new();
        }
        let bodies = u32::from_le_bytes([mesh[12], mesh[13], mesh[14], mesh[15]]);
        let mut offset = 16_usize;
        for _ in 0..bodies {
            if offset + 40 > mesh.len() {
                return Vec::new();
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
            return Vec::new();
        }
        let copies = u32::from_le_bytes([mesh[offset], mesh[offset + 1], mesh[offset + 2], mesh[offset + 3]]);
        offset += 4;
        let mut records = Vec::new();
        for _ in 0..copies {
            if offset + 8 + 64 > mesh.len() {
                break;
            }
            let ordinal = u32::from_le_bytes([mesh[offset], mesh[offset + 1], mesh[offset + 2], mesh[offset + 3]]);
            let node = i32::from_le_bytes([mesh[offset + 4], mesh[offset + 5], mesh[offset + 6], mesh[offset + 7]]);
            offset += 8;
            let mut matrix = [0.0_f32; 16];
            for (slot, value) in matrix.iter_mut().enumerate() {
                let start = offset + slot * 4;
                *value = f32::from_le_bytes([mesh[start], mesh[start + 1], mesh[start + 2], mesh[start + 3]]);
            }
            offset += 64;
            records.push((ordinal, node, matrix));
        }
        records
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
