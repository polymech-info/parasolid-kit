//! Preview tessellation for the native viewer. Planar faces keep their boundary.
//! Two-loop faces (walls, cones, fillets) are stitched along the trim curves.
//! A spherical face is the boundary disk pushed out onto the sphere.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use parasolid_core::brep::{
    Body, BodyKind, BrepId, BrepModel, CurveGeometry, CurveKind, Sense, SurfaceKind, Vector3,
};

const MAGIC: u32 = 0x314D_5350;
const VERSION: u32 = 2;
const MAX_TRIANGLES: usize = 400_000;
const MAX_EDGE_SAMPLES: usize = 96;
const COIN_IDENTITY: [f32; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// One placed copy of a tessellated body. `matrix` is the Coin row-vector form:
/// the three axis columns, then the translation in the last row.
pub(crate) struct Placement {
    pub body: u32,
    pub node_id: i32,
    pub matrix: [f32; 16],
}

#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn tessellate(model: &BrepModel) -> Result<Vec<u8>, String> {
    tessellate_placed(model, &[])
}

pub(crate) fn tessellate_placed(model: &BrepModel, placements: &[Placement]) -> Result<Vec<u8>, String> {
    let mut bodies = Vec::with_capacity(model.bodies.len());
    for (ordinal, body) in model.bodies.iter().enumerate() {
        bodies.push(tessellate_body(model, body, u32::try_from(ordinal).unwrap_or(u32::MAX))?);
    }
    let mut out = Vec::new();
    write_u32(&mut out, MAGIC);
    write_u32(&mut out, VERSION);
    write_u32(&mut out, u32::from(model.complete));
    write_u32(&mut out, u32::try_from(bodies.len()).unwrap_or(u32::MAX));
    for body in &bodies {
        write_u32(&mut out, body.ordinal);
        write_u32(&mut out, body.kind);
        write_i32(&mut out, body.node_id);
        write_u32(&mut out, u32::try_from(body.triangles.len()).unwrap_or(u32::MAX));
        for value in body.bounds {
            write_f32(&mut out, value);
        }
        for tri in &body.triangles {
            for corner in &tri.position {
                write_f32(&mut out, corner[0]);
                write_f32(&mut out, corner[1]);
                write_f32(&mut out, corner[2]);
            }
            for normal in &tri.normal {
                write_f32(&mut out, normal[0]);
                write_f32(&mut out, normal[1]);
                write_f32(&mut out, normal[2]);
            }
        }
    }
    let generated;
    let records = if placements.is_empty() {
        generated = identity_placements(model);
        generated.as_slice()
    } else {
        placements
    };
    write_u32(&mut out, u32::try_from(records.len()).unwrap_or(u32::MAX));
    for record in records {
        write_u32(&mut out, record.body);
        write_i32(&mut out, record.node_id);
        for value in record.matrix {
            write_f32(&mut out, value);
        }
    }
    Ok(out)
}

fn identity_placements(model: &BrepModel) -> Vec<Placement> {
    model
        .bodies
        .iter()
        .enumerate()
        .map(|(index, body)| Placement {
            body: u32::try_from(index).unwrap_or(u32::MAX),
            node_id: body.source.node_id.unwrap_or(-1),
            matrix: COIN_IDENTITY,
        })
        .collect()
}

struct BodyMesh {
    ordinal: u32,
    kind: u32,
    node_id: i32,
    bounds: [f32; 6],
    triangles: Vec<Triangle>,
}

struct Triangle {
    position: [[f32; 3]; 3],
    normal: [[f32; 3]; 3],
}

struct Sink<'a> {
    triangles: &'a mut Vec<Triangle>,
    bounds: &'a mut Bounds,
}

struct Bounds {
    min: [f64; 3],
    max: [f64; 3],
    any: bool,
}

impl Bounds {
    fn new() -> Self {
        Self {
            min: [0.0, 0.0, 0.0],
            max: [0.0, 0.0, 0.0],
            any: false,
        }
    }

    fn add(&mut self, point: Vector3) {
        let value = [point.x, point.y, point.z];
        if !value.iter().all(|item| item.is_finite()) {
            return;
        }
        if self.any {
            for (index, item) in value.iter().enumerate() {
                self.min[index] = self.min[index].min(*item);
                self.max[index] = self.max[index].max(*item);
            }
        } else {
            self.min = value;
            self.max = value;
            self.any = true;
        }
    }

    fn pack(&self) -> [f32; 6] {
        if self.any {
            [
                self.min[0] as f32,
                self.min[1] as f32,
                self.min[2] as f32,
                self.max[0] as f32,
                self.max[1] as f32,
                self.max[2] as f32,
            ]
        } else {
            [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
        }
    }
}

fn tessellate_body(model: &BrepModel, body: &Body, ordinal: u32) -> Result<BodyMesh, String> {
    let mut triangles = Vec::new();
    let mut bounds = Bounds::new();
    {
        let mut sink = Sink {
            triangles: &mut triangles,
            bounds: &mut bounds,
        };
        for face_id in face_ids(model, body) {
            tessellate_face(model, face_id, &mut sink)?;
        }
        for vertex_id in &body.vertices {
            if let Some(point) = vertex_position(model, *vertex_id) {
                bounds.add(point);
            }
        }
        for edge_id in &body.edges {
            if let Some(edge) = at(&model.edges, *edge_id) {
                if let Some(point) = edge.start_vertex.and_then(|id| vertex_position(model, id)) {
                    bounds.add(point);
                }
                if let Some(point) = edge.end_vertex.and_then(|id| vertex_position(model, id)) {
                    bounds.add(point);
                }
            }
        }
    }
    Ok(BodyMesh {
        ordinal,
        kind: match body.kind {
            BodyKind::Solid => 1,
            BodyKind::Wire => 2,
            BodyKind::Sheet => 3,
            BodyKind::General => 6,
        },
        node_id: body.source.node_id.unwrap_or(-1),
        bounds: bounds.pack(),
        triangles,
    })
}

fn face_ids(model: &BrepModel, body: &Body) -> Vec<BrepId> {
    let mut ids = Vec::new();
    for region_id in &body.regions {
        let Some(region) = at(&model.regions, *region_id) else {
            continue;
        };
        for shell_id in &region.shells {
            let Some(shell) = at(&model.shells, *shell_id) else {
                continue;
            };
            for face_id in shell.front_faces.iter().chain(&shell.back_faces) {
                if !ids.contains(face_id) {
                    ids.push(*face_id);
                }
            }
        }
    }
    ids
}

fn tessellate_face(model: &BrepModel, face_id: BrepId, sink: &mut Sink<'_>) -> Result<(), String> {
    let Some(face) = at(&model.faces, face_id) else {
        return Ok(());
    };
    let mut loops = Vec::new();
    for loop_id in &face.loops {
        if let Some(poly) = loop_polyline(model, *loop_id)
            && poly.len() >= 3
        {
            loops.push(poly);
        }
    }
    if loops.is_empty() {
        return Ok(());
    }
    let surface = face.surface.and_then(|id| at(&model.surfaces, id));
    let sense = sense_sign(face.sense) * surface.map_or(1.0, |item| sense_sign(item.sense));
    let kind = surface.map(|item| &item.kind);
    match kind {
        Some(SurfaceKind::Plane { point, normal, x_axis }) => {
            let y_axis = cross(normalize(*normal).unwrap_or(*normal), normalize(*x_axis).unwrap_or(*x_axis));
            triangulate_plane(
                &loops,
                *point,
                *normal,
                normalize(*x_axis).unwrap_or(*x_axis),
                y_axis,
                sense,
                kind,
                sink,
            )
        }
        Some(SurfaceKind::Sphere { center, radius, .. }) if loops.len() == 1 && *radius > 0.0 => {
            sphere_cap(&loops[0], *center, *radius, sense, sink)
        }
        _ if loops.len() >= 2 => stitch_loops(&loops[0], &loops[1], kind, sense, sink),
        _ => {
            let normal = kind
                .and_then(|item| surface_normal(item, sense, loops[0][0]))
                .unwrap_or_else(|| polygon_normal(&loops[0]));
            triangulate_polygon(&loops[0], normal, 1.0, kind, sink)
        }
    }
}

fn loop_polyline(model: &BrepModel, loop_id: BrepId) -> Option<Vec<Vector3>> {
    let loop_value = at(&model.loops, loop_id)?;
    let mut poly = Vec::new();
    for half_id in &loop_value.half_edges {
        let half = at(&model.half_edges, *half_id)?;
        if half.dummy {
            continue;
        }
        let Some(edge_id) = half.edge else {
            continue;
        };
        let edge = at(&model.edges, edge_id)?;
        let follow = half.sense != Sense::Negative;
        let (from_id, to_id) = if follow {
            (edge.start_vertex, edge.end_vertex)
        } else {
            (edge.end_vertex, edge.start_vertex)
        };
        let from = from_id.and_then(|id| vertex_position(model, id));
        let to = to_id.and_then(|id| vertex_position(model, id));
        let curve = edge
            .curve
            .or(half.curve)
            .and_then(|id| at(&model.curves, id));
        let mut samples = sample_curve(model, curve, from, to, follow);
        if samples.is_empty()
            && let Some(point) = from
        {
            samples.push(point);
        }
        if samples.last().is_some_and(|point| to.is_some_and(|end| near(*point, end))) {
            samples.pop();
        }
        poly.append(&mut samples);
    }
    dedup(&mut poly);
    (poly.len() >= 3).then_some(poly)
}

fn sample_curve(
    model: &BrepModel,
    curve: Option<&CurveGeometry>,
    from: Option<Vector3>,
    to: Option<Vector3>,
    follow: bool,
) -> Vec<Vector3> {
    let Some(curve) = curve else {
        return from.into_iter().collect();
    };
    let ccw = follow == (curve.sense != Sense::Negative);
    match &curve.kind {
        CurveKind::Circle { center, normal, x_axis, radius } => sample_ellipse(
            &EllipseFrame { center: *center, normal: *normal, x_axis: *x_axis, major: *radius, minor: *radius },
            from,
            to,
            ccw,
        ),
        CurveKind::Ellipse {
            center,
            normal,
            x_axis,
            major_radius,
            minor_radius,
        } => sample_ellipse(
            &EllipseFrame {
                center: *center,
                normal: *normal,
                x_axis: *x_axis,
                major: *major_radius,
                minor: *minor_radius,
            },
            from,
            to,
            ccw,
        ),
        CurveKind::Trimmed { basis_curve, .. } => {
            let basis = at(&model.curves, *basis_curve);
            sample_curve(model, basis, from, to, follow)
        }
        CurveKind::Nurbs(nurbs) => sample_nurbs_curve(nurbs, from, to),
        CurveKind::Intersection { chart_points, .. } => sample_chart(chart_points, from, to),
        _ => from.into_iter().collect(),
    }
}

#[derive(Clone, Copy)]
struct EllipseFrame {
    center: Vector3,
    normal: Vector3,
    x_axis: Vector3,
    major: f64,
    minor: f64,
}

fn sample_ellipse(frame_in: &EllipseFrame, from: Option<Vector3>, to: Option<Vector3>, ccw: bool) -> Vec<Vector3> {
    let EllipseFrame { center, normal, x_axis, major, minor } = *frame_in;
    let Some(frame) = orthonormal(normal, x_axis) else {
        return from.into_iter().collect();
    };
    if major.abs() < 1e-12 || minor.abs() < 1e-12 {
        return from.into_iter().collect();
    }
    let angle_of = |point: Vector3| -> f64 {
        let delta = sub(point, center);
        delta.dot_axis(frame.1).atan2(delta.dot_axis(frame.0))
    };
    let full = match (from, to) {
        (Some(start), Some(end)) => near(start, end),
        _ => true,
    };
    let (mut start, mut end) = if full {
        (0.0, std::f64::consts::TAU)
    } else {
        (angle_of(from.unwrap_or(center)), angle_of(to.unwrap_or(center)))
    };
    if ccw {
        while end <= start {
            end += std::f64::consts::TAU;
        }
    } else {
        while start <= end {
            start += std::f64::consts::TAU;
        }
        std::mem::swap(&mut start, &mut end);
    }
    let span = end - start;
    let steps = ((span / 0.18).ceil() as usize).clamp(2, MAX_EDGE_SAMPLES);
    let mut points = Vec::with_capacity(steps);
    for step in 0..steps {
        let angle = start + span * (step as f64) / (steps as f64);
        points.push(add(
            center,
            add(scale(frame.0, major * angle.cos()), scale(frame.1, minor * angle.sin())),
        ));
    }
    if let Some(start_point) = from
        && !full
        && let Some(first) = points.first_mut()
    {
        *first = start_point;
    }
    points
}

fn sample_nurbs_curve(curve: &parasolid_core::brep::NurbsCurve, from: Option<Vector3>, to: Option<Vector3>) -> Vec<Vector3> {
    let knots = expand_knots(&curve.knots, &curve.knot_multiplicities);
    let degree = usize::from(curve.degree);
    if degree == 0 || knots.len() <= degree + 1 || curve.control_vertices.is_empty() {
        return from.into_iter().collect();
    }
    let u0 = knots[degree];
    let u1 = knots[knots.len() - degree - 1];
    if !u0.is_finite() || !u1.is_finite() || u1 <= u0 {
        return from.into_iter().collect();
    }
    let count = 24.min(MAX_EDGE_SAMPLES);
    let mut samples = Vec::with_capacity(count);
    for step in 0..count {
        let u = u0 + (u1 - u0) * (step as f64) / ((count - 1) as f64);
        if let Some(point) = eval_nurbs_curve(curve, &knots, degree, u) {
            samples.push(point);
        }
    }
    trim_samples(samples, from, to)
}

fn sample_chart(chart: &[Vector3], from: Option<Vector3>, to: Option<Vector3>) -> Vec<Vector3> {
    trim_samples(chart.to_vec(), from, to)
}

fn trim_samples(mut samples: Vec<Vector3>, from: Option<Vector3>, to: Option<Vector3>) -> Vec<Vector3> {
    if samples.is_empty() {
        return from.into_iter().collect();
    }
    if let (Some(start), Some(end)) = (from, to) {
        let start_at = nearest(&samples, start);
        let end_at = nearest(&samples, end);
        if start_at <= end_at {
            samples = samples[start_at..end_at].to_vec();
        } else {
            samples = samples[end_at..start_at].to_vec();
            samples.reverse();
        }
        if samples.first().is_none_or(|point| !near(*point, start)) {
            samples.insert(0, start);
        }
    }
    samples
}

#[allow(clippy::too_many_arguments)]
fn triangulate_plane(
    loops: &[Vec<Vector3>],
    origin: Vector3,
    normal: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    sense: f64,
    kind: Option<&SurfaceKind>,
    sink: &mut Sink<'_>,
) -> Result<(), String> {
    let oriented = scale(normalize(normal).unwrap_or(normal), sense);
    let mut projected = loops
        .iter()
        .map(|loop_points| {
            loop_points
                .iter()
                .map(|point| {
                    let delta = sub(*point, origin);
                    [delta.dot_axis(x_axis), delta.dot_axis(y_axis)]
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let Some(outer_at) = projected
        .iter()
        .enumerate()
        .max_by(|left, right| signed_area(left.1).abs().total_cmp(&signed_area(right.1).abs()))
        .map(|(index, _)| index)
    else {
        return Ok(());
    };
    let mut outer = projected.swap_remove(outer_at);
    let mut outer_3d = loops[outer_at].clone();
    let mut holes_3d = loops.to_vec();
    holes_3d.remove(outer_at);
    if signed_area(&outer) < 0.0 {
        outer.reverse();
        outer_3d.reverse();
    }
    for hole_3d in holes_3d {
        let mut hole: Vec<[f64; 2]> = hole_3d
            .iter()
            .map(|point| {
                let delta = sub(*point, origin);
                [delta.dot_axis(x_axis), delta.dot_axis(y_axis)]
            })
            .collect();
        if signed_area(&hole) > 0.0 {
            hole.reverse();
        }
        if let Some(merged) = bridge(&outer, &outer_3d, &hole, &hole_3d) {
            outer = merged.0;
            outer_3d = merged.1;
        }
    }
    emit_indexed(&outer_3d, &ear_clip(&outer), oriented, kind, sink)
}

fn triangulate_polygon(
    points: &[Vector3],
    normal: Vector3,
    sense: f64,
    kind: Option<&SurfaceKind>,
    sink: &mut Sink<'_>,
) -> Result<(), String> {
    if points.len() < 3 {
        return Ok(());
    }
    let origin = points[0];
    let x_axis = normalize(sub(points[1], origin)).unwrap_or(Vector3 { x: 1.0, y: 0.0, z: 0.0 });
    let y_axis = cross(normalize(normal).unwrap_or(normal), x_axis);
    triangulate_plane(
        &[points.to_vec()],
        origin,
        normal,
        x_axis,
        y_axis,
        sense,
        kind,
        sink,
    )
}

fn sphere_cap(loop_points: &[Vector3], center: Vector3, radius: f64, sense: f64, sink: &mut Sink<'_>) -> Result<(), String> {
    let normal = polygon_normal(loop_points);
    let mut flat = Vec::new();
    {
        let mut bounds = Bounds::new();
        let mut scratch = Sink {
            triangles: &mut flat,
            bounds: &mut bounds,
        };
        triangulate_polygon(loop_points, normal, 1.0, None, &mut scratch)?;
    }
    for _ in 0..2 {
        flat = subdivide_radial(&flat, center, radius);
    }
    for tri in flat {
        let positions = tri.position.map(f32_to_vec);
        let mid = scale(add(add(positions[0], positions[1]), positions[2]), 1.0 / 3.0);
        let want = scale(normalize(sub(mid, center)).unwrap_or(sub(mid, center)), sense);
        let corners = positions;
        let shaded = corners.map(|point| {
            let direction = normalize(sub(point, center)).unwrap_or(want);
            if dot(direction, want) < 0.0 {
                scale(direction, -1.0)
            } else {
                direction
            }
        });
        emit_triangle(corners, want, Some(shaded), sink)?;
    }
    Ok(())
}

fn stitch_loops(
    left: &[Vector3],
    right: &[Vector3],
    kind: Option<&SurfaceKind>,
    sense: f64,
    sink: &mut Sink<'_>,
) -> Result<(), String> {
    let count = left.len().max(right.len()).clamp(8, 64);
    let a = resample_closed(left, count);
    let b = align_loop(&a, &resample_closed(right, count));
    if a.len() != b.len() || a.len() < 2 {
        return Ok(());
    }
    for index in 0..a.len() {
        let next = (index + 1) % a.len();
        let quad = [a[index], a[next], b[next], b[index]];
        let centroid = scale(add(add(quad[0], quad[1]), add(quad[2], quad[3])), 0.25);
        let want = kind
            .and_then(|item| surface_normal(item, sense, centroid))
            .unwrap_or_else(|| polygon_normal(quad.as_ref()));
        emit_triangle([quad[0], quad[1], quad[2]], want, corner_normals(kind, want, [quad[0], quad[1], quad[2]]), sink)?;
        emit_triangle([quad[0], quad[2], quad[3]], want, corner_normals(kind, want, [quad[0], quad[2], quad[3]]), sink)?;
    }
    Ok(())
}

fn align_loop(reference: &[Vector3], other: &[Vector3]) -> Vec<Vector3> {
    let count = reference.len();
    if count == 0 || count != other.len() {
        return other.to_vec();
    }
    let mut best = f64::MAX;
    let mut best_reverse = false;
    let mut best_shift = 0_usize;
    for reverse in [false, true] {
        for shift in 0..count {
            let mut cost = 0.0;
            #[allow(clippy::needless_range_loop)]
            for index in 0..count {
                let at = if reverse {
                    (shift + count - index) % count
                } else {
                    (shift + index) % count
                };
                cost += distance_sq(reference[index], other[at]);
            }
            if cost < best {
                best = cost;
                best_reverse = reverse;
                best_shift = shift;
            }
        }
    }
    (0..count)
        .map(|index| {
            let at = if best_reverse {
                (best_shift + count - index) % count
            } else {
                (best_shift + index) % count
            };
            other[at]
        })
        .collect()
}

fn resample_closed(points: &[Vector3], count: usize) -> Vec<Vector3> {
    if points.len() < 2 || count < 2 {
        return points.to_vec();
    }
    let mut lengths = Vec::with_capacity(points.len() + 1);
    lengths.push(0.0);
    let mut total = 0.0;
    for index in 0..points.len() {
        let next = points[(index + 1) % points.len()];
        total += distance(points[index], next);
        lengths.push(total);
    }
    if total < 1e-12 {
        return points.to_vec();
    }
    let mut out = Vec::with_capacity(count);
    for step in 0..count {
        let target = total * (step as f64) / (count as f64);
        let span = lengths.iter().position(|length| *length >= target).unwrap_or(lengths.len() - 1);
        let upper = span.max(1);
        let lower = upper - 1;
        let span_length = lengths[upper] - lengths[lower];
        let mix = if span_length > 1e-12 {
            (target - lengths[lower]) / span_length
        } else {
            0.0
        };
        let from = points[lower % points.len()];
        let to = points[upper % points.len()];
        out.push(add(from, scale(sub(to, from), mix)));
    }
    out
}

fn ear_clip(points: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let count = points.len();
    if count < 3 {
        return Vec::new();
    }
    if count == 3 {
        return vec![[0, 1, 2]];
    }
    let ccw = signed_area(points) >= 0.0;
    let mut prev: Vec<usize> = (0..count).map(|index| if index == 0 { count - 1 } else { index - 1 }).collect();
    let mut next: Vec<usize> = (0..count).map(|index| (index + 1) % count).collect();
    let mut alive = count;
    let mut cursor = 0usize;
    let mut tris = Vec::new();
    let mut guard = count * count;
    while alive > 3 && guard > 0 {
        guard -= 1;
        let a = prev[cursor];
        let b = cursor;
        let c = next[cursor];
        if is_ear(points, &next, a, b, c, ccw, alive) {
            tris.push([a, b, c]);
            next[a] = c;
            prev[c] = a;
            alive -= 1;
            cursor = c;
        } else {
            cursor = c;
        }
    }
    if alive == 3 {
        tris.push([prev[cursor], cursor, next[cursor]]);
    }
    tris
}

fn is_ear(points: &[[f64; 2]], next: &[usize], a: usize, b: usize, c: usize, ccw: bool, alive: usize) -> bool {
    let cross = (points[b][0] - points[a][0]) * (points[c][1] - points[a][1])
        - (points[b][1] - points[a][1]) * (points[c][0] - points[a][0]);
    let convex = if ccw { cross > 1e-14 } else { cross < -1e-14 };
    if !convex {
        return false;
    }
    let mut cursor = next[c];
    let mut seen = 0usize;
    while cursor != a && seen < alive {
        if cursor != b && point_in_triangle(points[cursor], points[a], points[b], points[c]) {
            return false;
        }
        cursor = next[cursor];
        seen += 1;
    }
    true
}

fn point_in_triangle(point: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let ab = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]);
    let bc = (c[0] - b[0]) * (point[1] - b[1]) - (c[1] - b[1]) * (point[0] - b[0]);
    let ca = (a[0] - c[0]) * (point[1] - c[1]) - (a[1] - c[1]) * (point[0] - c[0]);
    (ab >= -1e-12 && bc >= -1e-12 && ca >= -1e-12) || (ab <= 1e-12 && bc <= 1e-12 && ca <= 1e-12)
}

fn bridge(
    outer: &[[f64; 2]],
    outer_3d: &[Vector3],
    hole: &[[f64; 2]],
    hole_3d: &[Vector3],
) -> Option<(Vec<[f64; 2]>, Vec<Vector3>)> {
    if outer.len() < 3 || hole.len() < 3 || outer.len() != outer_3d.len() || hole.len() != hole_3d.len() {
        return None;
    }
    let mut hole_at = 0usize;
    for index in 1..hole.len() {
        if hole[index][0] > hole[hole_at][0] {
            hole_at = index;
        }
    }
    let mut outer_at = None;
    let mut best = f64::MAX;
    for index in 0..outer.len() {
        if !segment_clear(outer, index, hole[hole_at]) {
            continue;
        }
        let distance = (outer[index][0] - hole[hole_at][0]).powi(2) + (outer[index][1] - hole[hole_at][1]).powi(2);
        if distance < best {
            best = distance;
            outer_at = Some(index);
        }
    }
    let outer_at = outer_at?;
    let mut flat = Vec::with_capacity(outer.len() + hole.len() + 2);
    let mut solid = Vec::with_capacity(outer.len() + hole.len() + 2);
    flat.extend_from_slice(&outer[..=outer_at]);
    solid.extend_from_slice(&outer_3d[..=outer_at]);
    for step in 0..=hole.len() {
        let index = (hole_at + step) % hole.len();
        flat.push(hole[index]);
        solid.push(hole_3d[index]);
    }
    flat.push(outer[outer_at]);
    solid.push(outer_3d[outer_at]);
    flat.extend_from_slice(&outer[outer_at + 1..]);
    solid.extend_from_slice(&outer_3d[outer_at + 1..]);
    Some((flat, solid))
}

fn segment_clear(outer: &[[f64; 2]], from: usize, hole: [f64; 2]) -> bool {
    let start = outer[from];
    for index in 0..outer.len() {
        let next = (index + 1) % outer.len();
        if index == from || next == from {
            continue;
        }
        if segments_cross(start, hole, outer[index], outer[next]) {
            return false;
        }
    }
    true
}

fn segments_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    fn side(p: [f64; 2], q: [f64; 2], r: [f64; 2]) -> f64 {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    }
    let ab_c = side(a, b, c);
    let ab_d = side(a, b, d);
    let cd_a = side(c, d, a);
    let cd_b = side(c, d, b);
    ab_c * ab_d < 0.0 && cd_a * cd_b < 0.0
}

fn emit_indexed(
    points: &[Vector3],
    tris: &[[usize; 3]],
    want: Vector3,
    kind: Option<&SurfaceKind>,
    sink: &mut Sink<'_>,
) -> Result<(), String> {
    for tri in tris {
        let corners = [points.get(tri[0]), points.get(tri[1]), points.get(tri[2])];
        let [Some(a), Some(b), Some(c)] = corners else {
            continue;
        };
        let corners = [*a, *b, *c];
        emit_triangle(corners, want, corner_normals(kind, want, corners), sink)?;
    }
    Ok(())
}

fn corner_normals(kind: Option<&SurfaceKind>, want: Vector3, points: [Vector3; 3]) -> Option<[Vector3; 3]> {
    let kind = kind?;
    let mut normals = [
        surface_normal(kind, 1.0, points[0])?,
        surface_normal(kind, 1.0, points[1])?,
        surface_normal(kind, 1.0, points[2])?,
    ];
    if dot(normals[0], want) < 0.0 {
        normals = normals.map(|normal| scale(normal, -1.0));
    }
    Some(normals)
}

fn emit_triangle(
    mut corners: [Vector3; 3],
    want: Vector3,
    mut shaded: Option<[Vector3; 3]>,
    sink: &mut Sink<'_>,
) -> Result<(), String> {
    let mut normal = cross(sub(corners[1], corners[0]), sub(corners[2], corners[0]));
    if dot(normal, want) < 0.0 {
        corners.swap(1, 2);
        normal = scale(normal, -1.0);
        if let Some(normals) = &mut shaded {
            normals.swap(1, 2);
        }
    }
    let Some(normal) = normalize(normal) else {
        return Ok(());
    };
    if sink.triangles.len() >= MAX_TRIANGLES {
        return Err("parasolid mesh exceeds the triangle limit".into());
    }
    let position = corners.map(|point| [point.x as f32, point.y as f32, point.z as f32]);
    if position.iter().flatten().any(|value| !value.is_finite()) {
        return Ok(());
    }
    for point in corners {
        sink.bounds.add(point);
    }
    let fallback = [normal.x as f32, normal.y as f32, normal.z as f32];
    let stored = shaded.map_or([fallback, fallback, fallback], |normals| {
        normals.map(|item| {
            let item = normalize(item).unwrap_or(normal);
            [item.x as f32, item.y as f32, item.z as f32]
        })
    });
    sink.triangles.push(Triangle {
        position,
        normal: stored,
    });
    Ok(())
}

fn subdivide_radial(tris: &[Triangle], center: Vector3, radius: f64) -> Vec<Triangle> {
    let mut out = Vec::with_capacity(tris.len() * 4);
    for tri in tris {
        let corners = tri.position.map(f32_to_vec);
        let mid = [
            project_sphere(center, radius, scale(add(corners[0], corners[1]), 0.5)),
            project_sphere(center, radius, scale(add(corners[1], corners[2]), 0.5)),
            project_sphere(center, radius, scale(add(corners[2], corners[0]), 0.5)),
        ];
        let projected = corners.map(|point| project_sphere(center, radius, point));
        let pieces = [
            [projected[0], mid[0], mid[2]],
            [mid[0], projected[1], mid[1]],
            [mid[2], mid[1], projected[2]],
            [mid[0], mid[1], mid[2]],
        ];
        for piece in pieces {
            let normal = normalize(cross(sub(piece[1], piece[0]), sub(piece[2], piece[0]))).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            });
            let sign = if dot(normal, sub(piece[0], center)) < 0.0 {
                -1.0
            } else {
                1.0
            };
            let stored = piece.map(|point| {
            let direction = normalize(sub(point, center)).map_or(normal, |item| scale(item, sign));
                [direction.x as f32, direction.y as f32, direction.z as f32]
            });
            out.push(Triangle {
                position: piece.map(|point| [point.x as f32, point.y as f32, point.z as f32]),
                normal: stored,
            });
        }
    }
    out
}

fn project_sphere(center: Vector3, radius: f64, point: Vector3) -> Vector3 {
    let delta = sub(point, center);
    normalize(delta).map_or(point, |direction| add(center, scale(direction, radius.abs())))
}

fn eval_nurbs_curve(curve: &parasolid_core::brep::NurbsCurve, knots: &[f64], degree: usize, u: f64) -> Option<Vector3> {
    let mut weight_sum = 0.0;
    let mut point = Vector3 { x: 0.0, y: 0.0, z: 0.0 };
    for (index, control) in curve.control_vertices.iter().enumerate() {
        let basis = basis(index, degree, u, knots);
        if basis == 0.0 {
            continue;
        }
        let (position, weight) = control_point(control, curve.rational)?;
        let weighted = basis * weight;
        point = add(point, scale(position, weighted));
        weight_sum += weighted;
    }
    (weight_sum.abs() > 1e-15).then(|| scale(point, 1.0 / weight_sum))
}

fn basis(index: usize, degree: usize, u: f64, knots: &[f64]) -> f64 {
    if degree == 0 {
        let last = index + 1 == knots.len() - 1;
        return f64::from(u >= knots[index] && (u < knots.get(index + 1).copied().unwrap_or(u) || (last && u <= knots[index])));
    }
    let left_span = knots.get(index + degree).copied().unwrap_or(u) - knots.get(index).copied().unwrap_or(u);
    let right_span = knots.get(index + degree + 1).copied().unwrap_or(u) - knots.get(index + 1).copied().unwrap_or(u);
    let left = if left_span.abs() < 1e-15 {
        0.0
    } else {
        (u - knots[index]) / left_span * basis(index, degree - 1, u, knots)
    };
    let right = if right_span.abs() < 1e-15 {
        0.0
    } else {
        (knots[index + degree + 1] - u) / right_span * basis(index + 1, degree - 1, u, knots)
    };
    left + right
}

fn control_point(values: &[f64], rational: bool) -> Option<(Vector3, f64)> {
    if values.len() < 3 {
        return None;
    }
    let weight = if rational { *values.get(3)? } else { 1.0 };
    if !weight.is_finite() || weight.abs() < 1e-15 {
        return None;
    }
    Some((
        Vector3 {
            x: values[0] / weight,
            y: values[1] / weight,
            z: values[2] / weight,
        },
        weight,
    ))
}

fn expand_knots(knots: &[f64], multiplicity: &[u16]) -> Vec<f64> {
    let mut out = Vec::new();
    for (knot, count) in knots.iter().zip(multiplicity) {
        for _ in 0..*count {
            out.push(*knot);
        }
    }
    out
}

fn vertex_position(model: &BrepModel, id: BrepId) -> Option<Vector3> {
    let vertex = at(&model.vertices, id)?;
    Some(at(&model.points, vertex.point)?.position)
}

fn at<T>(items: &[T], id: BrepId) -> Option<&T> {
    items.get(usize::try_from(id).ok()?)
}

fn polygon_normal(points: &[Vector3]) -> Vector3 {
    let mut normal = Vector3 { x: 0.0, y: 0.0, z: 0.0 };
    for index in 0..points.len() {
        let next = points[(index + 1) % points.len()];
        normal = add(normal, cross(points[index], next));
    }
    normalize(normal).unwrap_or(Vector3 { x: 0.0, y: 0.0, z: 1.0 })
}

fn signed_area(points: &[[f64; 2]]) -> f64 {
    let mut area = 0.0;
    for index in 0..points.len() {
        let next = points[(index + 1) % points.len()];
        area += points[index][0] * next[1] - next[0] * points[index][1];
    }
    area * 0.5
}

fn nearest(points: &[Vector3], target: Vector3) -> usize {
    let mut best = 0usize;
    let mut distance = f64::MAX;
    for (index, point) in points.iter().enumerate() {
        let value = distance_sq(*point, target);
        if value < distance {
            distance = value;
            best = index;
        }
    }
    best
}

fn dedup(points: &mut Vec<Vector3>) {
    points.dedup_by(|left, right| near(*left, *right));
    if points.len() >= 2 && near(points[0], *points.last().unwrap_or(&points[0])) {
        points.pop();
    }
}

fn near(left: Vector3, right: Vector3) -> bool {
    distance_sq(left, right) < 1e-16
}

fn distance(left: Vector3, right: Vector3) -> f64 {
    distance_sq(left, right).sqrt()
}

fn distance_sq(left: Vector3, right: Vector3) -> f64 {
    let delta = sub(left, right);
    dot(delta, delta)
}

fn orthonormal(normal: Vector3, x_axis: Vector3) -> Option<(Vector3, Vector3)> {
    let z_axis = normalize(normal)?;
    let x_axis = normalize(sub(x_axis, scale(z_axis, dot(x_axis, z_axis))))?;
    Some((x_axis, cross(z_axis, x_axis)))
}

fn sense_sign(sense: Sense) -> f64 {
    sense.multiplier().unwrap_or(1.0)
}

fn surface_normal(kind: &SurfaceKind, sense: f64, point: Vector3) -> Option<Vector3> {
    let normal = match kind {
        SurfaceKind::Plane { normal, .. } => *normal,
        SurfaceKind::Cylinder { point: origin, axis, .. } => away_from_axis(*origin, *axis, point)?,
        SurfaceKind::Cone {
            point: origin,
            axis,
            sin_half_angle,
            cos_half_angle,
            ..
        } => {
            let axis = normalize(*axis)?;
            let radial = away_from_axis(*origin, axis, point)?;
            add(scale(radial, *cos_half_angle), scale(axis, *sin_half_angle))
        }
        SurfaceKind::Sphere { center, .. } => sub(point, *center),
        SurfaceKind::Torus {
            center,
            axis,
            major_radius,
            ..
        } => {
            let radial = away_from_axis(*center, *axis, point)?;
            sub(point, add(*center, scale(radial, *major_radius)))
        }
        SurfaceKind::BlendedEdge { .. }
        | SurfaceKind::BlendBoundary { .. }
        | SurfaceKind::Spun { .. }
        | SurfaceKind::Offset { .. }
        | SurfaceKind::Nurbs(_)
        | SurfaceKind::Unsupported { .. } => return None,
    };
    normalize(scale(normal, sense))
}

fn away_from_axis(origin: Vector3, axis: Vector3, point: Vector3) -> Option<Vector3> {
    let axis = normalize(axis)?;
    let delta = sub(point, origin);
    normalize(sub(delta, scale(axis, dot(delta, axis))))
}

fn f32_to_vec(value: [f32; 3]) -> Vector3 {
    Vector3 {
        x: f64::from(value[0]),
        y: f64::from(value[1]),
        z: f64::from(value[2]),
    }
}

trait DotAxis {
    fn dot_axis(self, axis: Vector3) -> f64;
}

impl DotAxis for Vector3 {
    fn dot_axis(self, axis: Vector3) -> f64 {
        dot(self, axis)
    }
}

fn add(left: Vector3, right: Vector3) -> Vector3 {
    Vector3 { x: left.x + right.x, y: left.y + right.y, z: left.z + right.z }
}

fn sub(left: Vector3, right: Vector3) -> Vector3 {
    Vector3 { x: left.x - right.x, y: left.y - right.y, z: left.z - right.z }
}

fn scale(value: Vector3, factor: f64) -> Vector3 {
    Vector3 { x: value.x * factor, y: value.y * factor, z: value.z * factor }
}

fn dot(left: Vector3, right: Vector3) -> f64 {
    left.x * right.x + left.y * right.y + left.z * right.z
}

fn cross(left: Vector3, right: Vector3) -> Vector3 {
    Vector3 {
        x: left.y * right.z - left.z * right.y,
        y: left.z * right.x - left.x * right.z,
        z: left.x * right.y - left.y * right.x,
    }
}

fn normalize(value: Vector3) -> Option<Vector3> {
    let length = dot(value, value).sqrt();
    (length.is_finite() && length > 1e-15).then(|| scale(value, 1.0 / length))
}

fn write_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn write_i32(out: &mut Vec<u8>, value: i32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn write_f32(out: &mut Vec<u8>, value: f32) {
    out.extend_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::tessellate;
    use parasolid_core::brep::{
        Body, BodyKind, BrepMetrics, BrepModel, BrepSourceFormat, CurveGeometry, CurveKind, Edge, Face, HalfEdge,
        Loop, PointGeometry, Region, RegionKind, Sense, Shell, SourceNodeRef, SurfaceGeometry, SurfaceKind,
        TopologyValidation, Vector3, Vertex,
    };

    fn source() -> SourceNodeRef {
        SourceNodeRef {
            node_index: 0,
            node_type: 0,
            type_name: String::new(),
            node_id: None,
            byte_range: 0..0,
        }
    }

    fn point(id: u32, position: Vector3) -> PointGeometry {
        PointGeometry { id, position, owner: None, source: source() }
    }

    fn vertex(id: u32) -> Vertex {
        Vertex {
            id,
            point: id,
            tolerance: None,
            owner: source(),
            source: source(),
        }
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn unit_cube_has_twelve_triangles() {
        let corners = [
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            Vector3 { x: 1.0, y: 0.0, z: 0.0 },
            Vector3 { x: 1.0, y: 1.0, z: 0.0 },
            Vector3 { x: 0.0, y: 1.0, z: 0.0 },
            Vector3 { x: 0.0, y: 0.0, z: 1.0 },
            Vector3 { x: 1.0, y: 0.0, z: 1.0 },
            Vector3 { x: 1.0, y: 1.0, z: 1.0 },
            Vector3 { x: 0.0, y: 1.0, z: 1.0 },
        ];
        let faces = [
            ([0u32, 3, 2, 1], Vector3 { x: 0.0, y: 0.0, z: -1.0 }),
            ([4, 5, 6, 7], Vector3 { x: 0.0, y: 0.0, z: 1.0 }),
            ([0, 1, 5, 4], Vector3 { x: 0.0, y: -1.0, z: 0.0 }),
            ([3, 7, 6, 2], Vector3 { x: 0.0, y: 1.0, z: 0.0 }),
            ([0, 4, 7, 3], Vector3 { x: -1.0, y: 0.0, z: 0.0 }),
            ([1, 2, 6, 5], Vector3 { x: 1.0, y: 0.0, z: 0.0 }),
        ];
        let mut curves = Vec::new();
        let mut edges = Vec::new();
        let mut half_edges = Vec::new();
        let mut loops = Vec::new();
        let mut face_nodes = Vec::new();
        let mut surfaces = Vec::new();
        let mut edge_of: Vec<(u32, u32, u32)> = Vec::new();
        for (face_index, (corners_of_face, normal)) in faces.iter().enumerate() {
            let mut loop_halves = Vec::new();
            for corner in 0..4 {
                let start = corners_of_face[corner];
                let end = corners_of_face[(corner + 1) % 4];
                let existing = edge_of.iter().find(|item| item.0 == end && item.1 == start).copied();
                let (edge_id, sense) = if let Some((_, _, id)) = existing {
                    (id, Sense::Negative)
                } else {
                    let id = u32::try_from(edges.len()).unwrap_or(0);
                    curves.push(CurveGeometry {
                        id: u32::try_from(curves.len()).unwrap_or(0),
                        sense: Sense::Positive,
                        owner: None,
                        kind: CurveKind::Line {
                            point: corners[start as usize],
                            direction: Vector3 {
                                x: corners[end as usize].x - corners[start as usize].x,
                                y: corners[end as usize].y - corners[start as usize].y,
                                z: corners[end as usize].z - corners[start as usize].z,
                            },
                        },
                        source: source(),
                    });
                    edges.push(Edge {
                        id,
                        owner: source(),
                        half_edges: Vec::new(),
                        start_vertex: Some(start),
                        end_vertex: Some(end),
                        curve: Some(u32::try_from(curves.len() - 1).unwrap_or(0)),
                        tolerance: None,
                        source: source(),
                    });
                    edge_of.push((start, end, id));
                    (id, Sense::Positive)
                };
                let half_id = u32::try_from(half_edges.len()).unwrap_or(0);
                half_edges.push(HalfEdge {
                    id: half_id,
                    loop_id: Some(u32::try_from(face_index).unwrap_or(0)),
                    forward: None,
                    backward: None,
                    vertex: Some(end),
                    other: None,
                    edge: Some(edge_id),
                    curve: None,
                    sense,
                    dummy: false,
                    source: source(),
                });
                edges[edge_id as usize].half_edges.push(half_id);
                loop_halves.push(half_id);
            }
            loops.push(Loop {
                id: u32::try_from(face_index).unwrap_or(0),
                face: u32::try_from(face_index).unwrap_or(0),
                half_edges: loop_halves,
                source: source(),
            });
            let x_axis = Vector3 {
                x: corners[corners_of_face[1] as usize].x - corners[corners_of_face[0] as usize].x,
                y: corners[corners_of_face[1] as usize].y - corners[corners_of_face[0] as usize].y,
                z: corners[corners_of_face[1] as usize].z - corners[corners_of_face[0] as usize].z,
            };
            surfaces.push(SurfaceGeometry {
                id: u32::try_from(face_index).unwrap_or(0),
                sense: Sense::Positive,
                owner: None,
                kind: SurfaceKind::Plane {
                    point: corners[corners_of_face[0] as usize],
                    normal: *normal,
                    x_axis,
                },
                source: source(),
            });
            face_nodes.push(Face {
                id: u32::try_from(face_index).unwrap_or(0),
                back_shell: 0,
                front_shell: 0,
                loops: vec![u32::try_from(face_index).unwrap_or(0)],
                surface: Some(u32::try_from(face_index).unwrap_or(0)),
                sense: Sense::Positive,
                source: source(),
            });
        }
        let model = BrepModel {
            source_format: BrepSourceFormat::Text,
            schema_key: "test".into(),
            complete: true,
            bodies: vec![Body {
                id: 0,
                kind: BodyKind::Solid,
                size_resolution: 0.0,
                linear_resolution: 0.0,
                regions: vec![0],
                edges: Vec::new(),
                vertices: Vec::new(),
                source: source(),
            }],
            regions: vec![Region {
                id: 0,
                kind: RegionKind::Solid,
                body: 0,
                shells: vec![0],
                source: source(),
            }],
            shells: vec![Shell {
                id: 0,
                region: 0,
                back_faces: Vec::new(),
                front_faces: (0..6).collect(),
                wire_edges: Vec::new(),
                isolated_vertex: None,
                source: source(),
            }],
            faces: face_nodes,
            loops,
            half_edges,
            edges,
            vertices: (0..8).map(vertex).collect(),
            points: corners
                .iter()
                .enumerate()
                .map(|(id, position)| point(u32::try_from(id).unwrap_or(0), *position))
                .collect(),
            curves,
            surfaces,
            topology: TopologyValidation {
                valid: true,
                closed_loop_count: 6,
                closed_edge_ring_count: 12,
                euler_characteristic: 2,
            },
            metrics: BrepMetrics {
                bounding_box: None,
                surface_area: None,
                volume: None,
            },
            diagnostics: Vec::new(),
        };
        let bytes = tessellate(&model).unwrap_or_default();
        assert!(bytes.len() > 16);
        let triangles = u32::from_le_bytes(bytes[28..32].try_into().unwrap_or([0; 4]));
        assert_eq!(triangles, 12);
    }
}
