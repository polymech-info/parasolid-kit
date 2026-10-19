# PolyMech notes

These notes cover only the parasolid-kit additions used by [Tanit](https://tanit.polymech.info/user/3bb4cfbf-318b-44d3-a9d3-35680e738421/pages/tanit-chat-next). Tanit is a native Coin viewer. It links `parasolid.dll` and does not run this package's Python viewer or call Python at draw time.

A pull request to parasolid-kit can take the conversion fixes. The Coin container, the PSM preview, and the CMake packaging exist so Tanit can draw a transmit file. They are not required by the kit's parse, check, or STEP contract.

## Added in this package

| Piece | Role |
|---|---|
| `crates/parasolid-capi` | `parasolid.dll` (`cdylib` + `staticlib`). Crate name `parasolid`. `publish = false`. |
| `include/parasolid.h` | C ABI: inspect, summary, mesh, and the free functions. |
| `CMakeLists.txt` | `cargo build -p parasolid-capi --locked`, then copies the DLL and import library. The comment that the library does not emit a mesh is stale: `parasolid_mesh` does. |
| `crates/parasolid-capi/src/mesh.rs` | `parasolid_view` writes planes, cylinders, cones, spheres, tori, NURBS, offsets, spun surfaces, and rolling-ball blends, plus trim loops, for the C++ mesher. The stitch triangles stay on each face and are used only when that face cannot be built. |
| `crates/parasolid-capi/src/lib.rs` (`assemble`, `transform_matrix`, `coin_matrix`) | Instance walk and the Coin matrix. |
| `src/parasolid_kit/interop/occt/scene_mesh.py` | Development mesh. OCCT triangles in source units, empty instance trailer. |
| `src/parasolid_kit/interop/occt/parametric.py` | Analytic sphere pcurve. A refused face still aborts `build()`. Open shells stay available to the preview. |

Status codes are `PARASOLID_OK`, `UNSUPPORTED`, `MALFORMED`, `LIMIT`, `IO`, `PANIC`, `INVALID`. A panic inside the mesh call becomes `PARASOLID_PANIC` and the string `"parasolid panicked"`. Strings from the DLL are freed with `parasolid_string_free`. Mesh bytes are freed with `parasolid_bytes_free`.

Schema `13006` (the SolidWorks files below) is not a built-in profile. `parasolid.dll` loads `sch_<id>.sch_txt` from `PARASOLID_SCHEMA_DIR`, from `schema\` beside Tanit, or from `schema\` beside the DLL. It does not search a CAD install. The catalogs copied for the viewer live in the 3D plugin's `schema` folder. `scene_mesh.py` still defaults to a SolidWorks `schema_18` when regenerating a fixture. A sibling `.occt.psm` is drawn without reading a catalog. The kit repository does not vendor the files.

## Problems and what fixed them

### The viewer cannot host the Python API

Tanit is C++. The kit's public surface was Python plus `parasolid-core`.

`parasolid_inspect`, `parasolid_summary`, `parasolid_mesh`, and `parasolid_view` expose the Rust parser. Summary is JSON counts. `parasolid_view` is the buffer Tanit meshes with its own OCCT. `parasolid_mesh` is the stitch fallback. Neither calls Python.

### Parts sat on the world origin

Drawing each body once, at identity, drops the assembly. The transmit stores instances separately from the bodies.

`assemble` walks instance nodes (node type 11). An instance whose part is an assembly (node type 10) is another level. An instance whose part is a body (node type 12) becomes one placed copy. The world matrix is the parent chain times the local transform. Depth stops at 16.

The local transform is the Parasolid one, `p' = s(Rp + t)`. These files have scale 1. The rotation is `rotation_matrix` in row order. The translation is `translation_vector`. A status flag that means "translation and rotation are present" is not a pivot, and `perspective_vector` on the checked transforms is null.

`coin_matrix` stores that column-vector matrix as Coin rows: three axis columns, then the translation in floats 12–14. Coin multiplies as `v' = v * M`. The same layout is what Tanit's STEP path writes from `gp_Trsf`. Transposing the matrix turns a flat stack into plates that cross.

Checked on `Mould-650-15.x_t`. Body 2 is about 0.631 × 0.016 × 0.632 m, thin in Y. Its instance is `R·(x,y,z) = (-y, -z, x)` plus `(0, 0.002, 0)`, so `p' = (-y, 0.002 − z, x)`. The plate lies in XZ. The other large plates stay parallel to it. The 15 mm rails stay rails. A camera rotation cannot make those plates perpendicular. A 9–11 mm gap on some small hardware is still unexplained. Bounds do not say whether those placements are wrong.

### Shaded faces need a real surface mesh

The PSM preview is a boundary mesh. A patterned face with six loops keeps two of them. A cylinder or NURBS face with one loop becomes a flat polygon and one normal, so the viewer cannot shade it. That is why `Global_FilamentHousing.x_t` looked unshaded and why its three repeated parts were only partly there.

The preview is intentionally not a trimmed-surface tessellator. Shaded faces come from OCCT.

`scene_mesh_bytes` builds with `ParametricTopologyBuilder` in millimetres (the unit the housing spheres validate in) and scales triangle coordinates back to the source unit. The placement trailer is not rewritten. The writer ends with an instance count of 0. Tanit replaces that word with the trailer from `parasolid_mesh`, so the same 72 matrices apply to the OCCT triangles.

On the housing, bodies 10 and 11 are the repeated patterned parts (planes and cylinders, faces with up to six loops, three copies each, about 160 mm apart). The boundary mesh had 1,075 triangles each. The OCCT mesh has 5,704. The frame sheets and the base grew the same way.

### One sphere still refuses the housing

Null-curve edges on a sphere were getting a projected pcurve, and `FixMissingSeam` then repaired a curve that was already off the surface. `V ≈ 1.49` is a latitude in radians, not a failed conversion.

`_analytic_sphere_pcurve` uses `ElSLib.Parameters` and checks the trim with `ElSLib.SphereValue`. A miss raises before `FixMissingSeam`. A pole is bridged on that latitude.

`build()` still raises `RefusedFaces` and does not assemble the document. The housing now fails on one face: sphere face 798, one loop, body 3 (a part about 9.5 mm across), `BRepCheck_UnorientableShape`. The other 889 faces convert.

`scene_mesh` catches that refusal and meshes the faces already in `face_shapes`, as a compound, without sewing. Face 798 is absent from the preview. `to_occt` is unchanged and still rejects the document. SameParameter is not cleared, and per-face copies of shared edges are not the fix.

### An open shell is still a failed solid

Housing shells that do not close record `occt.solid_not_closed`. The faces stay in `rendered_bodies` with the sheet kind. Forcing those boundaries into a solid is a different change and is not done here. The mould bodies that do close export as solids. The housing preview is an open shell plus the one omitted sphere.

### A flat ellipse dragged one sample to its far tip

`sample_ellipse` used `atan2(y, x)` on the raw frame. That is the circle parameter. An ellipse parameter is `atan2(y / minor, x / major)`. On `Global-LCM.x_t` body 17, four edges sit on an ellipse of major radius 2.29 m and minor radius 0.04 m. The raw angle evaluated near the major-axis tip, about 1.45 m from the edge, and only the first sample was put back on the vertex. Dividing by the two radii keeps the samples on the arc. A partial arc also pins its last sample to the end vertex.

### One bad edge used to drop the whole mesh

`build()` stopped at the first edge that threw, so no face was available to tessellate. `Global-LCM.x_t` has 5,343 edges and 76 of them fail: a UV trim off its stored point by at most 0.10 mm, a line vertex off the curve by about 1e-6 mm, or a parameter curve on a surface the converter does not take. Those edges are recorded as `occt.edge_refused`. Face construction continues. `RefusedFaces` is still raised and the document is not assembled, so `to_occt` still fails. The preview meshes the faces that did convert.

On that file the stitched body 17 is 1.57 m long in one axis because of the ellipse tip. The OCCT mesh of the same body is 0.160 × 0.241 × 0.199 m. Two small parts, bodies 6 and 9, about 5 mm across, have no converted faces and are absent from the preview.

## PSM buffer

Little-endian, magic `0x314D5350` (`PSM1`), version 2.

- `u32` flags. Bit 0 is set when the mapped model is complete. Complete means the parse mapped the source B-rep. It does not mean the preview is a solid or a CAD mesh.
- `u32` body count.
- Each body: `u32` ordinal, `u32` kind (1 solid, 2 wire, 3 sheet, 6 general), `i32` node id (`-1` if absent), `u32` triangle count, `f32` min[3], `f32` max[3], then that many corners of `f32` position[3] and normal[3].
- `u32` instance count, then each instance: `u32` body ordinal, `i32` node id, 16 `f32` Coin matrix.

An empty instance list is drawn as nothing. If the assembly walk finds no instances, the mesh call writes one identity placement per body. `scene_mesh.py` always writes a count of 0; the caller splices the real trailer.

Caps in the preview: 400,000 triangles, 96 samples on an edge.

## Where Tanit uses it

The product surface is [Tanit](https://tanit.polymech.info/user/3bb4cfbf-318b-44d3-a9d3-35680e738421/pages/tanit-chat-next). The call site is the native 3D viewer, not this repository:

`src/win/3d_viewer/formats/read_ps.cpp` calls `parasolid_mesh` for `.x_t`, `.x_b`, `.xt`, and `.xb`. If `<transmit>.occt.psm` is a version-2 PSM, that file supplies the triangles and the kit trailer supplies the instances. The Coin node is `SoMatrixTransform` with the 16 floats stored as documented above. There is no extra root rotation on this path.

`parasolid_mesh` returns `PARASOLID_PARTIAL` (7) when the buffer is still drawable. `*out_err` is a short report (`bodies rendered`, `faces approximated`, `faces omitted`, `instances placed`, `instances unplaced`). One bad face is omitted or approximated; an open shell is drawn; a bad transform is listed after the placed instances as an unplaced body at identity, and the viewer parents those under **Unplaced parts**. A cycle stops that branch. A depth, instance, or triangle limit returns the scene collected so far. A null output pointer, an over-size buffer, or a non-finite matrix component still rejects the call. The matrix product is unchanged: `p' = s(Rp + t)`.

`parasolid_summary` maps the model and stops. Placement expansion runs only for `parasolid_mesh`. Each body is tessellated once; instance records point at that body. External `sch_*.sch_txt` providers are cached by path, modification time, and size. Built-in profiles already live in a process-wide registry. The viewer builds one Coin mesh node per body and parents every copy of that node under its own `SoMatrixTransform`.

Files used while checking:

- `tests/3d/Mould-650-15.x_t` and `Mould-650-15.x_t.occt.psm`. Placements describe one Y-thin sandwich. The OCCT mesh is what shades it.
- `tests/3d/Global-LCM.x_t` and `Global-LCM.x_t.occt.psm`. 41 bodies, 2,104 faces, 89 instances. The OCCT mesh shades the faces and drops the ellipse spike. Bodies 6 and 9 are missing.
- `tests/3d/Nema-23-PX57-Coupling.x_t` and `Nema-23-PX57-Coupling.x_t.occt.psm`. 4 bodies, 191 faces, 4 instances. Every face converted. Body 2 is the coupling housing: planes, cylinders, cones, and two tori. The boundary stitch was what twisted it and left faces open.
- `tests/3d/Global-Printhead-SM25.x_t`. Rolling-ball blends store unset `thumb_weight` values as null. Those are the neutral weight 1. A present non-finite weight is still rejected. There is no `.occt.psm` for this file. Tanit builds those surfaces in C++, including NURBS, offsets, spun surfaces, and rolling-ball blends, and uses the stitch only for a face that build refuses.

## Pull request scope

Reasonable kit changes:

- Analytic sphere pcurves, rejected before `FixMissingSeam` when `SphereValue` misses.
- `RefusedFaces` leaving `build()` unassembled, and `occt.solid_not_closed` keeping the open shell available without reporting a solid.
- A C ABI for inspect and summary, if the project wants a non-Python parser entry.

Poor fit for the kit's stated contract (read the transmit, preserve it, convert and export STEP):

- The PSM preview and its Coin row layout.
- `scene_mesh.py` and the "mesh the faces that converted" path. That path is a Tanit fixture. It does not change `to_occt`.
- CMake rules whose output is `parasolid.dll` next to the Tanit plugin.

The viewer path is the Rust parser, `parasolid_view`, and C++ OCCT face construction in `read_ps.cpp`. Placements stay the Coin matrices from the trailer. A sibling `.occt.psm` is still loaded when one is already next to the file. Python is not on this path.
