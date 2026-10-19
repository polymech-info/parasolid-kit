"""Shared trimmed topology with source UV curves and bounded intersection fits.

Imported only after the optional OCCT runtime has been resolved. Representation
operations construct pcurves, seams and parameter agreement; they do not sew,
move, or discard source faces. Every source boundary remains mapped.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from itertools import pairwise
from math import copysign, isfinite, pi

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Curve2d
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeSolid
from OCP.BRepCheck import BRepCheck_Analyzer, BRepCheck_ListOfStatus
from OCP.BRepLib import BRepLib
from OCP.BSplCLib import BSplCLib
from OCP.Precision import Precision
from OCP.ShapeBuild import ShapeBuild_ReShape
from OCP.ShapeFix import ShapeFix_Edge, ShapeFix_Face, ShapeFix_Wire
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_FORWARD,
    TopAbs_REVERSED,
    TopAbs_VERTEX,
    TopAbs_WIRE,
)
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS, TopoDS_Face, TopoDS_Shell
from OCP.TopTools import TopTools_IndexedMapOfShape

from ...brep.geometry import (
    BlendBoundarySurface,
    BlendedEdgeSurface,
    ConeSurface,
    CylinderSurface,
    IntersectionCurve,
    LineCurve,
    NurbsCurve,
    OffsetSurface,
    PlaneSurface,
    SphereSurface,
    SurfaceParametricCurve,
    TorusSurface,
    TrimmedCurve,
)
from ...brep.topology import BodyKind, RegionKind, Sense, is_isolated_vertex_loop
from ...diagnostics import Diagnostic, DiagnosticKind, DiagnosticSeverity, SourceLocation
from .intersection import fit_intersection, intersect_analytic, project_curve
from .model import ShapeRelationKind, SourceEntityKind
from .topology import BuiltTopology, TopologyBuilder, _sense_sign


def _period_shift(delta: float, period: float | None, tolerance: float) -> float:
    """Move ``delta`` by whole periods when the remainder is only numerical noise."""

    if period is None or period <= 0.0:
        return 0.0
    turns = round(delta / period)
    if turns == 0:
        return 0.0
    if abs(delta - turns * period) > tolerance:
        return 0.0
    return -turns * period


def _oriented_uv(edge, face) -> tuple[tuple[float, float], tuple[float, float]]:
    """Pcurve ends in the edge orientation used by the wire.

    ``BRepAdaptor_Curve2d`` reports the stored parameter direction. A reversed
    fin walks that curve from its last parameter back to its first.
    """

    curve = BRepAdaptor_Curve2d(edge, face)
    start = curve.Value(curve.FirstParameter())
    end = curve.Value(curve.LastParameter())
    if int(edge.Orientation()) == int(TopAbs_REVERSED):
        return (end.X(), end.Y()), (start.X(), start.Y())
    return (start.X(), start.Y()), (end.X(), end.Y())


def _translate_pcurve(curve, du: float, dv: float):
    if du == 0.0 and dv == 0.0:
        return curve
    from OCP.gp import gp_Vec2d

    return curve.Translated(gp_Vec2d(du, dv))


def _status_names(status) -> list[str]:
    """Every status in an OCCT list. The first entry is often NoError."""

    if status.IsEmpty():
        return []
    copy = BRepCheck_ListOfStatus()
    copy.Assign(status)
    names = []
    while not copy.IsEmpty():
        names.append(str(copy.First()))
        copy.RemoveFirst()
    return names


def _first_failing_status(analyzer, shape) -> str:
    """First failing BRepCheck status, including a status recorded on a parent shape."""

    for candidate in (
        shape,
        *subshapes(shape, TopAbs_WIRE),
        *subshapes(shape, TopAbs_EDGE),
        *subshapes(shape, TopAbs_VERTEX),
    ):
        result = analyzer.Result(candidate)
        for name in _status_names(result.Status()):
            if "NoError" not in name:
                return name
        result.InitContextIterator()
        while result.MoreShapeInContext():
            for name in _status_names(result.StatusOnShape()):
                if "NoError" not in name:
                    return name
            result.NextShapeInContext()
    return "BRepCheck_NoError"


def boundary_census(shape: object) -> dict[str, int]:
    """Classify shell edges by use, without treating a same-face seam as a hole.

    A periodic seam is one edge used twice by the same face. A degenerated edge
    closes a singularity. A free boundary is a single non-degenerated use.
    """

    groups: list[dict[str, object]] = []
    for face_index, face_shape in enumerate(subshapes(shape, TopAbs_FACE)):
        for edge_shape in subshapes(face_shape, TopAbs_EDGE):
            edge = TopoDS.Edge_s(edge_shape)
            group = next((item for item in groups if item["edge"].IsSame(edge)), None)
            if group is None:
                group = {
                    "edge": edge,
                    "faces": set(),
                    "uses": 0,
                    "degenerated": bool(BRep_Tool.Degenerated_s(edge)),
                }
                groups.append(group)
            group["faces"].add(face_index)
            group["uses"] += 1
    counts = {"free": 0, "seam": 0, "degenerated": 0, "shared": 0, "edges": len(groups)}
    for group in groups:
        if group["degenerated"]:
            counts["degenerated"] += 1
        elif len(group["faces"]) >= 2:
            counts["shared"] += 1
        elif group["uses"] >= 2:
            counts["seam"] += 1
        else:
            counts["free"] += 1
    return counts


def subshapes(shape: object, kind: object) -> list:
    indexed = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, kind, indexed)
    return [indexed.FindKey(i) for i in range(1, indexed.Extent() + 1)]


class RefusedFaces(Exception):
    """One or more source faces failed; the document was not assembled."""

    def __init__(self, diagnostics: tuple[Diagnostic, ...]) -> None:
        self.face_diagnostics = diagnostics
        super().__init__(f"{len(diagnostics)} source faces were refused")


class ParametricTopologyBuilder(TopologyBuilder):
    """Two-pass face construction keeps both sides of each shared edge consistent."""

    def __init__(self, model, geometry):
        super().__init__(model, geometry)
        self.context = ShapeBuild_ReShape()
        self.reversed_edges = set()
        resolutions = [
            b.linear_resolution * geometry.scale
            for b in model.bodies
            if b.linear_resolution is not None and b.linear_resolution > 0
        ]
        self.precision = min(resolutions, default=Precision.Confusion_s() / 10)
        if not isfinite(self.precision) or self.precision <= 0:
            raise ValueError("body linear resolution must be finite and positive")
        self.curve_anchors = defaultdict(list)
        for curve in model.curves:
            d = curve.definition
            if isinstance(d, TrimmedCurve):
                self.curve_anchors[d.basis_curve].extend(
                    (geometry.point(d.start_point), geometry.point(d.end_point))
                )
        self.geometry_checks = Counter()
        # Curves built by approximation and the deviation each one may carry.
        self.approximated_curves: dict[int, float] = {}
        self.diagnostics = []
        self.rendered_bodies: list[tuple[int, object, bool]] = []
        self.isolated_edge_shapes: dict[int, object] = {}

    def build(self) -> BuiltTopology:
        self._build_vertices()
        # A bad edge must not discard every face. The document still does not
        # assemble: the edge is recorded and RefusedFaces is raised below.
        refused_edges: list[Diagnostic] = []
        for edge in self.model.edges:
            try:
                self._build_edge(edge)
            except Exception as error:
                self.edge_shapes.pop(edge.id, None)
                refused_edges.append(self._refused_edge_diagnostic(edge, error))
        # All face pcurves must exist before a periodic seam splits shared rings.
        # A refused face is recorded and skipped. Shared edges can make a later
        # face fail because an earlier one did; the messages stay per face.
        refused: dict[int, Exception] = {}
        for face in self.model.faces:
            try:
                self._build_face(face)
            except Exception as error:
                refused[face.id] = error
                self.face_shapes.pop(face.id, None)
        for face in self.model.faces:
            if face.id in refused:
                continue
            try:
                self._finish_face(face)
            except Exception as error:
                refused[face.id] = error
                self.face_shapes.pop(face.id, None)
        # A later periodic face can replace a shared edge in the reshape context.
        # The replacement keeps both curve representations, but ShapeFix's copy
        # leaves SameParameter and SameRange inconsistent. Set the flags only
        # where those representations already agree.
        self._reconcile_parameter_flags()
        for face in self.model.faces:
            if face.id in refused:
                continue
            try:
                shape = TopoDS.Face_s(self.context.Apply(self.face_shapes[face.id]))
                analyzer = BRepCheck_Analyzer(shape)
                if not analyzer.IsValid():
                    raise ValueError(
                        f"OCCT parametric face {face.id} is invalid: "
                        f"{_first_failing_status(analyzer, shape)}"
                    )
                self.face_shapes[face.id] = shape
            except Exception as error:
                refused[face.id] = error
                self.face_shapes.pop(face.id, None)
        if refused or refused_edges:
            findings = tuple(
                self._refused_face_diagnostic(face, refused[face.id])
                for face in self.model.faces
                if face.id in refused
            ) + tuple(refused_edges)
            self.diagnostics.extend(findings)
            raise RefusedFaces(findings)
        self._validate_source_boundaries()
        # The base wire builder records provisional relations; resolve them only
        # after every seam replacement has propagated to both adjacent faces.
        self.relations.clear()
        self._record_boundaries()
        bodies = self._build_bodies()
        return BuiltTopology(
            self._make_root(bodies),
            tuple(self.relations),
            tuple(dict.fromkeys(self.operations)),
            tuple(self.diagnostics),
        )

    def _edge_tolerance(self, edge):
        if edge.tolerance is not None and (not isfinite(edge.tolerance) or edge.tolerance < 0):
            raise ValueError(f"edge {edge.id} has an invalid source tolerance")
        return max(
            Precision.Confusion_s(),
            (edge.tolerance or self.precision / self.geometry.scale) * self.geometry.scale,
        )

    def _build_vertices(self):
        super()._build_vertices()
        builder = BRep_Builder()
        for vertex in self.model.vertices:
            tolerance = vertex.tolerance or self.precision / self.geometry.scale
            if not isfinite(tolerance) or tolerance < 0:
                raise ValueError(f"vertex {vertex.id} has an invalid tolerance")
            builder.UpdateVertex(
                self.vertex_shapes[vertex.id],
                max(Precision.Confusion_s(), tolerance * self.geometry.scale),
            )

    def _curve_geometry(self, curve_id):
        if curve_id in self.curve_geometries:
            return self.curve_geometries[curve_id]
        definition = self.curves[curve_id].definition
        if not isinstance(definition, IntersectionCurve):
            return super()._curve_geometry(curve_id)
        if (
            not definition.start_points
            or not definition.end_points
            or len(definition.chart_points) < 2
        ):
            raise ValueError(f"intersection {curve_id} requires source CHART and LIMIT points")
        points = [
            self.geometry.point(p)
            for p in (
                definition.start_points[0],
                *definition.chart_points,
                definition.end_points[0],
            )
        ]
        boundaries = [
            i
            for i in definition.surfaces
            if isinstance(self.surfaces[i].definition, BlendBoundarySurface)
        ]
        if boundaries:
            result = self._blend_contact_curve(definition, points, boundaries[0])
            self.operations.append("blend_contact_curve")
            self.approximated_curves[curve_id] = max(
                self.precision, self.geometry_checks.get("blend_contact_deviation", 0.0)
            )
            self.curve_geometries[curve_id] = result
            return result
        surfaces = [self._surface_geometry(i) for i in definition.surfaces]
        analytic = all(
            isinstance(
                self.surfaces[i].definition,
                (PlaneSurface, CylinderSurface, ConeSurface, SphereSurface, TorusSurface),
            )
            for i in definition.surfaces
        )
        if analytic and any(
            isinstance(self.surfaces[i].definition, PlaneSurface) for i in definition.surfaces
        ):
            result = intersect_analytic(
                surfaces, points, max(self.precision, Precision.Confusion_s())
            )
            self.operations.append("source_identified_surface_intersection")
        else:
            result = fit_intersection(
                surfaces, points, self.curve_anchors[curve_id], self.precision
            )
            self.operations.append("intersection_curve_numerical_approximation")
            self.approximated_curves[curve_id] = self.precision
        self.curve_geometries[curve_id] = result
        return result

    def _blend_contact_curve(self, definition, points, boundary_id):
        """The curve where a rolling-ball blend touches one support.

        A blend boundary surface is the construction surface whose
        intersection with a support is that contact curve; its support is
        ``surface[1 - boundary]`` of the blend (XT p. 65). The contact point
        for a spine point is the foot of the ball on the support, at the
        blend radius. The source CHART points fix the spine range; the curve
        is interpolated through contact points between them and must pass
        through every chart point within the precision.
        """
        from OCP.GeomAPI import GeomAPI_Interpolate, GeomAPI_ProjectPointOnSurf
        from OCP.TColgp import TColgp_HArray1OfPnt

        boundary = self.surfaces[boundary_id].definition
        blend = self.surfaces[boundary.blend_surface].definition
        if not isinstance(blend, BlendedEdgeSurface):
            raise ValueError("blend boundary does not reference a blend surface")
        support = self._surface_geometry(blend.supporting_surfaces[1 - boundary.boundary_index])
        spine = self._curve_geometry(blend.spine_curve)
        radius = abs(blend.ranges[0]) * self.geometry.scale
        parameters = [project_curve(spine, p)[0] for p in points]
        if spine.IsPeriodic() or spine.IsClosed():
            period = (
                spine.Period()
                if spine.IsPeriodic()
                else spine.LastParameter() - spine.FirstParameter()
            )
            for index in range(1, len(parameters)):
                step = parameters[index] - parameters[index - 1]
                parameters[index] = (
                    parameters[index - 1] + (step + period / 2) % period - period / 2
                )
        numerical = max(1e-14, self.precision * 1e-3)

        def contact(parameter):
            centre = spine.Value(parameter)
            projection = GeomAPI_ProjectPointOnSurf(centre, support, numerical)
            if not projection.NbPoints():
                raise ValueError("blend contact projection failed")
            point = projection.NearestPoint()
            if abs(point.Distance(centre) - radius) > self.precision:
                raise ValueError("blend contact point is not at the blend radius")
            return point

        samples = []
        for a, b in pairwise(parameters):
            if abs(b - a) <= numerical:
                continue
            for k in range(8):
                sample = contact(a + (b - a) * k / 8)
                if not samples or sample.Distance(samples[-1]) > numerical:
                    samples.append(sample)
        last = contact(parameters[-1])
        if not samples or last.Distance(samples[-1]) > numerical:
            samples.append(last)
        closed = points[0].Distance(points[-1]) <= numerical
        if closed and len(samples) > 2 and samples[0].Distance(samples[-1]) <= numerical:
            samples.pop()
        if len(samples) < 2:
            raise ValueError("blend contact curve has fewer than two distinct points")
        array = TColgp_HArray1OfPnt(1, len(samples))
        for index, point in enumerate(samples, 1):
            array.SetValue(index, point)
        fit = GeomAPI_Interpolate(array, closed, numerical)
        fit.Perform()
        if not fit.IsDone():
            raise ValueError("blend contact curve interpolation failed")
        curve = fit.Curve()
        # The spine is itself an approximation within the precision and the
        # chart points carry the source resolution, so the contact curve may
        # differ from a chart point by both.
        worst = max(project_curve(curve, p)[1] for p in points)
        if worst > 4 * self.precision:
            raise ValueError(
                f"blend contact curve misses a source chart point by {worst:g} "
                f"(limit {4 * self.precision:g})"
            )
        self.geometry_checks["blend_contact_deviation"] = max(
            self.geometry_checks.get("blend_contact_deviation", 0.0), worst
        )
        return curve

    def _parameter_curve(self, definition):
        curve = self.curves[definition.parameter_curve].definition
        if not isinstance(curve, NurbsCurve):
            raise ValueError("surface parameter curve must reference NURBS")
        support = self.surfaces[definition.surface].definition
        # An offset surface keeps the parameters of its basis (XT p. 71).
        seen = set()
        while isinstance(support, OffsetSurface):
            if support.basis_surface in seen:
                raise ValueError("offset surface basis chain is circular")
            seen.add(support.basis_surface)
            support = self.surfaces[support.basis_surface].definition
        return self.geometry.parameter_curve(curve, support)

    def _surface_trim(self, curve_id):
        definition = None if curve_id is None else self.curves[curve_id].definition
        if isinstance(definition, TrimmedCurve):
            basis = self.curves[definition.basis_curve].definition
            if isinstance(basis, SurfaceParametricCurve):
                return definition, basis
        return None

    def _build_edge(self, edge):
        tolerance = self._edge_tolerance(edge)
        if edge.curve is not None:
            # An edge on an approximated branch carries that approximation:
            # the branch is built first so that its deviation is known.
            basis = self.curves[edge.curve].definition
            geometry_id = basis.basis_curve if isinstance(basis, TrimmedCurve) else edge.curve
            if isinstance(self.curves[geometry_id].definition, IntersectionCurve):
                self._curve_geometry(geometry_id)
            if geometry_id in self.approximated_curves:
                tolerance = max(tolerance, 2 * self.approximated_curves[geometry_id])
        vs = None if edge.start_vertex is None else self.vertex_shapes[edge.start_vertex]
        ve = None if edge.end_vertex is None else self.vertex_shapes[edge.end_vertex]
        if (vs is None) != (ve is None):
            raise ValueError(f"edge {edge.id} has only one endpoint vertex")
        ps, pe = (None, None) if vs is None else (BRep_Tool.Pnt_s(vs), BRep_Tool.Pnt_s(ve))

        def endpoints(q1, q2):
            nonlocal vs, ve
            if vs is None:
                return
            if q1.Distance(ps) + q2.Distance(pe) > q1.Distance(pe) + q2.Distance(ps):
                vs, ve = ve, vs
                self.reversed_edges.add(edge.id)
            for point, vertex in ((q1, vs), (q2, ve)):
                if point.Distance(BRep_Tool.Pnt_s(vertex)) > max(
                    tolerance, BRep_Tool.Tolerance_s(vertex)
                ):
                    raise ValueError(
                        f"edge {edge.id} geometry misses its source endpoint tolerance"
                    )

        if edge.curve is None:
            if not edge.half_edges or vs is None:
                raise ValueError(f"edge {edge.id} requires finite FIN geometry")
            trimmed = self._surface_trim(self.half_edges[edge.half_edges[0]].curve)
            if trimmed is None:
                raise ValueError(f"edge {edge.id} requires trimmed surface-parametric FIN curves")
            definition, support = trimmed
            curve = self._parameter_curve(support)
            surface = self._surface_geometry(support.surface)
            a, b = definition.start_parameter, definition.end_parameter
            if a == b:
                raise ValueError(f"edge {edge.id} requires distinct UV trim parameters")
            q1, q2 = (surface.Value(curve.Value(t).X(), curve.Value(t).Y()) for t in (a, b))
            for point, source in ((q1, definition.start_point), (q2, definition.end_point)):
                if point.Distance(self.geometry.point(source)) > tolerance:
                    raise ValueError(f"edge {edge.id} UV trim disagrees with its source point")
            # A trim stored against the parameter direction keeps its end
            # points; the edge range is the same interval, so the vertex at
            # the lower parameter comes first.
            endpoints(q1, q2) if a < b else endpoints(q2, q1)
            builder = BRepBuilderAPI_MakeEdge(curve, surface, vs, ve, min(a, b), max(a, b))
            self.operations.append("surface_parametric_curve_3d_approximation")
        else:
            source = self.curves[edge.curve]
            definition = source.definition
            if isinstance(definition, LineCurve):
                if vs is None:
                    raise ValueError(f"line edge {edge.id} requires endpoints")
                self.edge_shapes[edge.id] = self.geometry.make_line_edge(
                    definition,
                    vs,
                    ve,
                    self.points[self.vertices[edge.start_vertex].point].position,
                    self.points[self.vertices[edge.end_vertex].point].position,
                )
                return
            trimmed_parameter = self._surface_trim(edge.curve)
            if isinstance(definition, SurfaceParametricCurve) or trimmed_parameter:
                # An edge whose own geometry is a surface parameter curve, or a
                # trim of one: the 3D edge is the image of the 2D curve on its
                # surface over the stored interval.
                if trimmed_parameter:
                    trim, definition = trimmed_parameter
                    a, b = sorted((trim.start_parameter, trim.end_parameter))
                    if a == b:
                        raise ValueError(f"edge {edge.id} requires distinct UV trim parameters")
                    curve = self._parameter_curve(definition)
                    surface = self._surface_geometry(definition.surface)
                    for parameter, source_point in (
                        (trim.start_parameter, trim.start_point),
                        (trim.end_parameter, trim.end_point),
                    ):
                        uv = curve.Value(parameter)
                        point = surface.Value(uv.X(), uv.Y())
                        if point.Distance(self.geometry.point(source_point)) > tolerance:
                            raise ValueError(
                                f"edge {edge.id} UV trim disagrees with its source point"
                            )
                else:
                    curve = self._parameter_curve(definition)
                    surface = self._surface_geometry(definition.surface)
                    a, b = curve.FirstParameter(), curve.LastParameter()
                q1, q2 = (surface.Value(curve.Value(t).X(), curve.Value(t).Y()) for t in (a, b))
                if vs is None:
                    if q1.Distance(q2) > tolerance:
                        raise ValueError(f"edge {edge.id} on a surface parameter curve is open")
                    builder = BRepBuilderAPI_MakeEdge(curve, surface, a, b)
                else:
                    endpoints(q1, q2)
                    builder = BRepBuilderAPI_MakeEdge(curve, surface, vs, ve, a, b)
                self.operations.append("surface_parametric_curve_3d_approximation")
            elif isinstance(definition, TrimmedCurve):
                basis = self.curves[definition.basis_curve]
                curve = self._curve_geometry(basis.id)
                a, b = definition.start_parameter, definition.end_parameter
                if isinstance(basis.definition, IntersectionCurve):
                    a, da = project_curve(curve, self.geometry.point(definition.start_point))
                    b, db = project_curve(curve, self.geometry.point(definition.end_point))
                    if max(da, db) > tolerance:
                        raise ValueError(
                            f"intersection edge {edge.id} misses its source trim points"
                        )
                elif isinstance(basis.definition, LineCurve):
                    a, b = a * self.geometry.scale, b * self.geometry.scale
                for parameter, point in ((a, definition.start_point), (b, definition.end_point)):
                    if curve.Value(parameter).Distance(self.geometry.point(point)) > tolerance:
                        raise ValueError(
                            f"trimmed edge {edge.id} parameter disagrees with its source point"
                        )
                if curve.IsPeriodic():
                    # On a closed basis the trim is the arc that follows the
                    # source parameter direction from start to end.
                    if definition.start_parameter > definition.end_parameter:
                        a, b = b, a
                    if b <= a:
                        b += curve.Period()
                else:
                    a, b = sorted((a, b))
                endpoints(curve.Value(a), curve.Value(b))
                builder = (
                    BRepBuilderAPI_MakeEdge(curve, a, b)
                    if vs is None
                    else BRepBuilderAPI_MakeEdge(curve, vs, ve, a, b)
                )
            else:
                curve = self._curve_geometry(source.id)
                if vs is None:
                    if not curve.IsClosed():
                        raise ValueError(f"edge {edge.id} is unbounded")
                    builder = BRepBuilderAPI_MakeEdge(curve)
                else:
                    a, da = project_curve(curve, ps)
                    b, db = project_curve(curve, pe)
                    if max(da, db) > max(
                        tolerance, BRep_Tool.Tolerance_s(vs), BRep_Tool.Tolerance_s(ve)
                    ):
                        raise ValueError(f"edge {edge.id} cannot project its source vertices")
                    # A vertex on an approximated branch must enclose its
                    # distance to that branch for the edge to be built on it.
                    for vertex, distance in ((vs, da), (ve, db)):
                        if distance > BRep_Tool.Tolerance_s(vertex):
                            BRep_Builder().UpdateVertex(vertex, distance + Precision.Confusion_s())
                    if curve.IsPeriodic():
                        if source.sense is Sense.POSITIVE and b <= a:
                            b += curve.Period()
                        elif source.sense is Sense.NEGATIVE and a <= b:
                            a += curve.Period()
                    a, b = sorted((a, b))
                    endpoints(curve.Value(a), curve.Value(b))
                    builder = BRepBuilderAPI_MakeEdge(curve, vs, ve, a, b)
        if not builder.IsDone():
            raise ValueError(f"OCCT edge {edge.id} construction failed: {builder.Error()}")
        shape = builder.Edge()
        BRep_Builder().UpdateEdge(shape, tolerance)
        if not BRepLib.BuildCurve3d_s(shape, tolerance, MaxSegment=1000):
            raise ValueError(f"OCCT edge {edge.id} 3D approximation failed")
        self.edge_shapes[edge.id] = shape

    def _edge_for_half_edge(self, half_edge):
        edge = self.edges[half_edge.edge]
        closed = edge.start_vertex is not None and edge.start_vertex == edge.end_vertex
        if half_edge.vertex is None or closed:
            if edge.curve is None:
                raise ValueError(f"half-edge {half_edge.id} has no orientation reference")
            # A closed curve names the same vertex at both ends, so the fin
            # sense is the only direction. Comparing the vertex first would
            # always walk the curve forward.
            forward = half_edge.sense is self.curves[edge.curve].sense
        elif half_edge.vertex == edge.end_vertex:
            forward = True
        elif half_edge.vertex == edge.start_vertex:
            forward = False
        else:
            raise ValueError(f"half-edge {half_edge.id} does not end at its edge vertex")
        if edge.id in self.reversed_edges:
            forward = not forward
        return TopoDS.Edge_s(
            self.edge_shapes[edge.id].Oriented(TopAbs_FORWARD if forward else TopAbs_REVERSED)
        )

    def _analytic_sphere_pcurve(self, edge, face) -> bool:
        """Map a 3D curve onto a sphere by inverse parameters, not by projection.

        ``ElSLib.Parameters`` is the sphere's own inverse. The fitted pcurve is
        kept only when ``SphereValue`` reproduces the 3D curve over the whole
        trim. A miss raises, so ``FixMissingSeam`` never repairs a bad pcurve.
        """

        from OCP.BRepAdaptor import BRepAdaptor_Surface
        from OCP.ElSLib import ElSLib
        from OCP.Geom2d import Geom2d_BSplineCurve
        from OCP.Geom2dAPI import Geom2dAPI_Interpolate
        from OCP.GeomAbs import GeomAbs_Sphere
        from OCP.gp import gp_Pnt2d
        from OCP.TColgp import TColgp_Array1OfPnt2d, TColgp_HArray1OfPnt2d
        from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal, TColStd_HArray1OfReal

        adaptor = BRepAdaptor_Surface(face)
        if adaptor.GetType() != GeomAbs_Sphere:
            return False
        sphere = adaptor.Sphere()
        frame = sphere.Position()
        radius = sphere.Radius()
        curve = BRepAdaptor_Curve(edge)
        first = curve.FirstParameter()
        last = curve.LastParameter()
        if not isfinite(first) or not isfinite(last) or last <= first:
            raise ValueError("sphere edge has a non-finite parameter interval")

        def sample(count: int):
            rows = []
            previous_u = None
            roundtrip = 0.0
            for index in range(count):
                parameter = first + (last - first) * index / (count - 1)
                original = curve.Value(parameter)
                u, v = ElSLib.Parameters_s(sphere, original)
                if abs(abs(v) - pi / 2) <= 1.0e-8:
                    u = 0.0 if previous_u is None else previous_u
                elif previous_u is not None:
                    while u - previous_u > pi:
                        u -= 2 * pi
                    while previous_u - u > pi:
                        u += 2 * pi
                previous_u = u
                reconstructed = ElSLib.SphereValue_s(u, v, frame, radius)
                roundtrip = max(roundtrip, original.Distance(reconstructed))
                rows.append((parameter, u, v))
            return rows, roundtrip

        def deviation(fitted_curve, checks: int) -> float:
            fitted_error = 0.0
            for index in range(checks):
                parameter = first + (last - first) * index / (checks - 1)
                uv = fitted_curve.Value(parameter)
                reconstructed = ElSLib.SphereValue_s(uv.X(), uv.Y(), frame, radius)
                fitted_error = max(fitted_error, curve.Value(parameter).Distance(reconstructed))
            return fitted_error

        def polyline(rows):
            pole_count = len(rows)
            poles = TColgp_Array1OfPnt2d(1, pole_count)
            knots = TColStd_Array1OfReal(1, pole_count)
            mults = TColStd_Array1OfInteger(1, pole_count)
            previous = None
            for index, (parameter, u, v) in enumerate(rows, 1):
                if previous is not None and parameter <= previous:
                    parameter = previous + 1.0e-12
                poles.SetValue(index, gp_Pnt2d(u, v))
                knots.SetValue(index, parameter)
                mults.SetValue(index, 2 if index in (1, pole_count) else 1)
                previous = parameter
            return Geom2d_BSplineCurve(poles, knots, mults, 1, False)

        def latitude(rows):
            return polyline((rows[0], rows[-1]))

        def cross_pole(rows, bridge: float):
            """A meridian reaches V = ±π/2 and leaves with U shifted by π.

            The two sphere points are the same pole. The pcurve stays on that
            pole for a short parameter interval, then follows the outgoing
            meridian. A straight segment across the U jump leaves the circle.
            """

            built = [rows[0]]
            for parameter, u, v in rows[1:]:
                previous_t, previous_u, previous_v = built[-1]
                near_pole = max(abs(previous_v), abs(v)) > pi / 2 - 0.2
                if abs(u - previous_u) > pi / 2 and near_pole:
                    pole_v = copysign(pi / 2, previous_v if abs(previous_v) >= abs(v) else v)
                    joined = min(previous_t + bridge, parameter - bridge)
                    if joined <= previous_t:
                        joined = (previous_t + parameter) * 0.5
                    if abs(previous_v - pole_v) > 1.0e-8:
                        built.append((max(previous_t, joined - bridge), previous_u, pole_v))
                    built.append((joined, u, pole_v))
                built.append((parameter, u, v))
            return built

        def interpolate(rows):
            pole_count = len(rows)
            poles = TColgp_HArray1OfPnt2d(1, pole_count)
            parameters = TColStd_HArray1OfReal(1, pole_count)
            for index, (parameter, u, v) in enumerate(rows, 1):
                poles.SetValue(index, gp_Pnt2d(u, v))
                parameters.SetValue(index, parameter)
            operation = Geom2dAPI_Interpolate(poles, parameters, False, Precision.Confusion_s())
            operation.Perform()
            if not operation.IsDone():
                raise ValueError("sphere pcurve interpolation failed; FixMissingSeam was not run")
            return operation.Curve()

        samples, roundtrip = sample(33)
        limit = max(BRep_Tool.Tolerance_s(edge), Precision.Confusion_s())
        if roundtrip > limit:
            raise ValueError(
                f"sphere inverse misses the 3D curve by {roundtrip:.6e} "
                f"(limit {limit:.6e}); FixAddPCurve was not used"
            )
        bridge = min((last - first) * 1.0e-8, 0.2 * limit / max(radius, 1.0e-9))
        bridge = max(bridge, 1.0e-12)
        v_span = max(row[2] for row in samples) - min(row[2] for row in samples)
        u_steps = [samples[index][1] - samples[index - 1][1] for index in range(1, len(samples))]
        monotone = all(step >= -1.0e-9 for step in u_steps) or all(step <= 1.0e-9 for step in u_steps)
        crosses_pole = any(abs(step) > pi / 2 for step in u_steps)
        pcurve = None
        fitted = limit + 1.0
        if v_span <= 1.0e-8 and monotone and abs(samples[-1][1] - samples[0][1]) > 1.0e-8:
            pcurve = latitude(samples)
            fitted = deviation(pcurve, 65)
        elif crosses_pole:
            count = 33
            while fitted > limit and count <= 1025:
                if count != 33:
                    samples, denser_roundtrip = sample(count)
                    roundtrip = max(roundtrip, denser_roundtrip)
                    if roundtrip > limit:
                        raise ValueError(
                            f"sphere inverse misses the 3D curve by {roundtrip:.6e} "
                            f"(limit {limit:.6e}); FixAddPCurve was not used"
                        )
                pcurve = polyline(cross_pole(samples, bridge))
                fitted = deviation(pcurve, max(65, count * 4))
                count = count * 2 - 1
        count = 33
        while fitted > limit and count <= 1025 and not crosses_pole:
            if count != 33:
                samples, denser_roundtrip = sample(count)
                roundtrip = max(roundtrip, denser_roundtrip)
                if roundtrip > limit:
                    raise ValueError(
                        f"sphere inverse misses the 3D curve by {roundtrip:.6e} "
                        f"(limit {limit:.6e}); FixAddPCurve was not used"
                    )
            pcurve = interpolate(samples)
            fitted = deviation(pcurve, max(65, count * 4))
            count = count * 2 - 1
        if pcurve is None or fitted > limit:
            raise ValueError(
                f"fitted sphere pcurve misses the 3D curve by {fitted:.6e} "
                f"(inverse {roundtrip:.6e}, limit {limit:.6e}); FixMissingSeam was not run"
            )
        builder = BRep_Builder()
        builder.UpdateEdge(edge, pcurve, face, BRep_Tool.Tolerance_s(edge))
        builder.Range(edge, face, first, last)
        builder.SameRange(edge, True)
        builder.SameParameter(edge, True)
        self.operations.append("analytic_sphere_parameters")
        return True

    def _add_pcurve(self, half_edge, face):
        shape = self.edge_shapes[half_edge.edge]
        trimmed = self._surface_trim(half_edge.curve)
        if trimmed is None and self._analytic_sphere_pcurve(shape, face):
            return
        if trimmed is None:
            ShapeFix_Edge().FixAddPCurve(shape, face, False, self.precision)
            self.operations.append("project_3d_curve_to_surface_parameters")
        else:
            definition, support = trimmed
            if self._surface_geometry(support.surface) is not BRep_Tool.Surface_s(face):
                raise ValueError(f"FIN {half_edge.id} support differs from its face surface")
            curve = self._parameter_curve(support)
            a, b = sorted((definition.start_parameter, definition.end_parameter))
            curve.Segment(a, b)
            adaptor = BRepAdaptor_Curve(shape)
            first = adaptor.Value(adaptor.FirstParameter())
            surface = BRep_Tool.Surface_s(face)
            p1, p2 = (surface.Value(curve.Value(t).X(), curve.Value(t).Y()) for t in (a, b))
            if first.Distance(p2) < first.Distance(p1):
                curve.Reverse()
            knots = curve.Knots()
            BSplCLib.Reparametrize_s(adaptor.FirstParameter(), adaptor.LastParameter(), knots)
            curve.SetKnots(knots)
            builder = BRep_Builder()
            builder.UpdateEdge(shape, curve, face, BRep_Tool.Tolerance_s(shape))
            builder.Range(shape, face, adaptor.FirstParameter(), adaptor.LastParameter())
        BRep_Builder().SameParameter(shape, False)
        BRepLib.SameParameter_s(shape, self.precision)

    def _build_face(self, face):
        if not face.loops:
            raise ValueError(f"parametric face {face.id} requires explicit source trim loops")
        shape = TopoDS_Face()
        builder = BRep_Builder()
        builder.MakeFace(
            shape,
            self._surface_geometry(face.surface),
            max(self.precision, Precision.Confusion_s()),
        )
        # Source loops run counterclockwise about the face normal, while the
        # seam and orientation fixes assume the surface normal. A face that
        # opposes its surface keeps its source region on a periodic surface
        # only when its wires are reversed here; _finish_face then reverses
        # the whole face, so the final boundary direction is unchanged.
        opposed = _sense_sign(face.sense) * _sense_sign(self.surfaces[face.surface].sense) < 0
        if opposed:
            self.operations.append("reverse_opposed_face_loops")
        isolated = [
            self.loops[loop_id]
            for loop_id in face.loops
            if is_isolated_vertex_loop(self.loops[loop_id], self.half_edges)
        ]
        ordinary = [loop_id for loop_id in face.loops if self.loops[loop_id] not in isolated]
        if not ordinary:
            raise ValueError(f"parametric face {face.id} has no edge-backed trim loop")

        def add_wire(wire: object, *, reverse: bool) -> None:
            shift = ShapeFix_Wire(wire, shape, self.precision)
            if shift.FixShifted():
                self.operations.append("unwrap_periodic_surface_parameters")
            # A loop through a sphere pole or a cone apex needs the degenerated
            # edge of that singularity; the seam fix alone leaves it unorientable.
            if shift.FixDegenerated():
                self.operations.append("insert_degenerated_singularity_edges")
            fixed = shift.Wire()
            builder.Add(shape, TopoDS.Wire_s(fixed.Reversed()) if reverse else fixed)

        for loop_id in ordinary:
            for half_edge_id in self.loops[loop_id].half_edges:
                self._add_pcurve(self.half_edges[half_edge_id], shape)
            # Seam pcurves are placed for the wire that is added. An opposed
            # face used to reverse that wire afterwards, which swaps the two
            # sides of a periodic seam.
            add_wire(self._make_parametric_wire(loop_id, shape, opposed=opposed), reverse=False)
        for loop in isolated:
            add_wire(self._cone_apex_wire(loop, face, shape), reverse=opposed)
            self.operations.append("cone_isolated_vertex_boundary")
        self.face_shapes[face.id] = shape

    def _make_parametric_wire(self, loop_id, face_shape, *, opposed: bool = False):
        """Wire in loop order, with a second pcurve on a periodic seam edge.

        Parasolid stores the cut of a cylinder, cone, or torus as one edge used
        by two fins of the same face. Those fins are the two sides of the
        parameter period. One pcurve cannot occupy both sides, and a wire
        builder that drops the second fin leaves the face unorientable.
        """

        from OCP.TopoDS import TopoDS_Wire

        loop = self.loops[loop_id]
        surface = BRep_Tool.Surface_s(face_shape)
        period_u = surface.UPeriod() if surface.IsUPeriodic() else None
        period_v = surface.VPeriod() if surface.IsVPeriodic() else None
        tolerance = max(self.precision, 1.0e-7)
        wire = TopoDS_Wire()
        builder = BRep_Builder()
        builder.MakeWire(wire)
        previous = None
        first_use: dict[int, bool] = {}
        half_edge_ids = tuple(reversed(loop.half_edges)) if opposed else loop.half_edges
        for half_edge_id in half_edge_ids:
            half_edge = self.half_edges[half_edge_id]
            oriented = self._edge_for_half_edge(half_edge)
            if opposed:
                oriented = TopoDS.Edge_s(oriented.Reversed())
            bare = TopoDS.Edge_s(oriented.Oriented(TopAbs_FORWARD))
            start, end = _oriented_uv(oriented, face_shape)
            if previous is not None:
                shift_u = _period_shift(start[0] - previous[0], period_u, tolerance)
                shift_v = _period_shift(start[1] - previous[1], period_v, tolerance)
                if shift_u or shift_v or half_edge.edge in first_use:
                    self._place_pcurve(
                        bare,
                        face_shape,
                        shift_u,
                        shift_v,
                        forward=int(oriented.Orientation()) == int(TopAbs_FORWARD),
                        seam=half_edge.edge in first_use,
                    )
                    start, end = _oriented_uv(oriented, face_shape)
                    if shift_u or shift_v:
                        self.operations.append("unwrap_periodic_surface_parameters")
            first_use.setdefault(half_edge.edge, True)
            builder.Add(wire, oriented)
            self.half_edge_shapes[half_edge.id] = oriented
            self._record(
                SourceEntityKind.HALF_EDGE,
                half_edge.id,
                half_edge.source,
                oriented,
            )
            previous = end
        wire.Closed(True)
        self.loop_shapes[loop.id] = wire
        self._record(SourceEntityKind.LOOP, loop.id, loop.source, wire)
        return wire

    def _place_pcurve(self, edge, face_shape, shift_u, shift_v, *, forward: bool, seam: bool):
        stored = BRep_Tool.CurveOnSurface_s(edge, face_shape, 0.0, 0.0)
        moved = _translate_pcurve(stored, shift_u, shift_v)
        tolerance = BRep_Tool.Tolerance_s(edge)
        if not seam:
            BRep_Builder().UpdateEdge(edge, moved, face_shape, tolerance)
            return
        # The stored curve is the edge's forward pcurve. The other side of the
        # period belongs to whichever fin is being placed now.
        if forward:
            forward_curve, reversed_curve = moved, stored
        else:
            forward_curve, reversed_curve = stored, moved
        BRep_Builder().UpdateEdge(
            edge,
            forward_curve,
            reversed_curve,
            face_shape,
            tolerance,
        )
        self.operations.append("periodic_seam_pcurve")

    def _cone_apex_wire(self, loop, face, shape):
        """Degenerated edge at the cone apex, spanning the trim's U period.

        The isolated fin has no curve. The cone parameterization supplies the
        boundary: V is the apex, and U covers one full turn when the other
        trim encircles the axis. A sector with no lateral edges is refused.
        """

        from OCP.Geom2d import Geom2d_Line
        from OCP.ShapeAnalysis import ShapeAnalysis_Surface
        from OCP.TopAbs import TopAbs_FORWARD, TopAbs_REVERSED
        from OCP.TopoDS import TopoDS_Edge, TopoDS_Wire
        from OCP.gp import gp_Dir2d, gp_Pnt2d

        definition = self.surfaces[face.surface].definition
        if not isinstance(definition, ConeSurface):
            kind = self.surfaces[face.surface].kind.value
            raise ValueError(
                f"isolated vertex loop {loop.id} on {kind} face {face.id} "
                "has no degenerate-boundary construction"
            )
        if abs(definition.sin_half_angle) <= 1.0e-15:
            raise ValueError(f"cone face {face.id} has a zero half-angle sine")
        fin = self.half_edges[loop.half_edges[0]]
        surface = self._surface_geometry(face.surface)
        apex_v = -(definition.radius * self.geometry.scale) / definition.sin_half_angle
        apex = surface.Value(0.0, apex_v)
        vertex = self.vertex_shapes[fin.vertex]
        source_point = self.geometry.point(self.points[self.vertices[fin.vertex].point].position)
        tolerance = max(self.precision, BRep_Tool.Tolerance_s(vertex))
        if source_point.Distance(apex) > tolerance * 10:
            raise ValueError(
                f"isolated vertex {fin.vertex} on cone face {face.id} is not the cone apex"
            )
        period = surface.UPeriod()
        probe = ShapeAnalysis_Surface(surface)
        samples: list[float] = []
        for edge in subshapes(shape, TopAbs_EDGE):
            adaptor = BRepAdaptor_Curve(TopoDS.Edge_s(edge))
            first, last = adaptor.FirstParameter(), adaptor.LastParameter()
            for index in range(5):
                point = adaptor.Value(first + (last - first) * index / 4)
                samples.append(probe.ValueOfUV(point, tolerance).X() % period)
        if len(samples) < 2:
            raise ValueError(f"cone face {face.id} has no edge samples for its apex boundary")
        ordered = sorted(samples)
        gaps = [right - left for left, right in pairwise(ordered)]
        gaps.append(ordered[0] + period - ordered[-1])
        largest = max(gaps)
        if largest > period * 0.25:
            raise ValueError(
                f"cone face {face.id} trim does not encircle the apex; "
                "an isolated vertex cannot close a sector that has no lateral edges"
            )
        start = gaps.index(largest)
        u0 = ordered[0] if start == len(ordered) - 1 else ordered[start + 1]
        edge = TopoDS_Edge()
        builder = BRep_Builder()
        builder.MakeEdge(edge)
        builder.UpdateEdge(
            edge,
            Geom2d_Line(gp_Pnt2d(u0, apex_v), gp_Dir2d(1.0, 0.0)),
            shape,
            tolerance,
        )
        builder.Range(edge, shape, 0.0, period)
        builder.Add(edge, TopoDS.Vertex_s(vertex.Oriented(TopAbs_FORWARD)))
        builder.Add(edge, TopoDS.Vertex_s(vertex.Oriented(TopAbs_REVERSED)))
        builder.Degenerated(edge, True)
        self.isolated_edge_shapes[fin.id] = edge
        self.half_edge_shapes[fin.id] = edge
        wire = TopoDS_Wire()
        builder.MakeWire(wire)
        builder.Add(wire, edge)
        self.loop_shapes[loop.id] = wire
        return wire

    def _refused_edge_diagnostic(self, edge, error: Exception) -> Diagnostic:
        node_id = edge.source.node_id
        return Diagnostic(
            code="occt.edge_refused",
            severity=DiagnosticSeverity.ERROR,
            kind=DiagnosticKind.INVALID,
            fatal=True,
            message=f"edge {edge.id} was not converted: {error}",
            location=SourceLocation(byte_offset=edge.source.byte_range.start),
            node_type=edge.source.node_type,
            node_id=None if node_id is None or node_id < 0 else node_id,
            schema_key=self.model.schema_key,
            details={
                "entity_kind": "edge",
                "entity_id": edge.id,
                "exception_type": type(error).__name__,
            },
        )

    def _refused_face_diagnostic(self, face, error: Exception) -> Diagnostic:
        surface = self.surfaces.get(face.surface)
        kind = None if surface is None else surface.kind.value
        node_id = face.source.node_id
        return Diagnostic(
            code="occt.face_refused",
            severity=DiagnosticSeverity.ERROR,
            kind=(
                DiagnosticKind.UNSUPPORTED
                if "no degenerate-boundary construction" in str(error)
                else DiagnosticKind.INVALID
            ),
            fatal=True,
            message=(
                f"{kind or 'unknown'} face {face.id} ({len(face.loops)} loops) "
                f"was not converted: {error}"
            ),
            location=SourceLocation(byte_offset=face.source.byte_range.start),
            node_type=face.source.node_type,
            node_id=None if node_id is None or node_id < 0 else node_id,
            schema_key=self.model.schema_key,
            details={
                "entity_kind": "face",
                "entity_id": face.id,
                "geometry_kind": kind,
                "loop_count": len(face.loops),
                "exception_type": type(error).__name__,
            },
        )

    def _finish_face(self, face):
        shape = TopoDS.Face_s(self.context.Apply(self.face_shapes[face.id]))
        operation = ShapeFix_Face(shape)
        operation.SetContext(self.context)
        operation.SetPrecision(self.precision)
        operation.SetMaxTolerance(max(self.precision, Precision.Confusion_s()))
        if operation.FixMissingSeam():
            self.operations.append("generate_periodic_surface_seam")
        if operation.FixOrientation():
            self.operations.append("orient_surface_boundary_wires")
        shape = operation.Face()
        if shape.IsNull():
            # The seam fix can hand back its single face inside a shell.
            faces = subshapes(operation.Result(), TopAbs_FACE)
            if len(faces) != 1:
                raise ValueError(f"OCCT face {face.id} was split into {len(faces)} faces")
            shape = TopoDS.Face_s(faces[0])
            self.operations.append("unwrap_single_face_fix_result")
        for candidate in subshapes(shape, TopAbs_EDGE):
            edge = TopoDS.Edge_s(candidate)
            if not BRep_Tool.SameRange_s(edge):
                BRep_Builder().SameParameter(edge, False)
                BRepLib.SameRange_s(edge, max(1e-14, self.precision * 1e-4))
            BRepLib.SameParameter_s(edge, self.precision)
        self.face_shapes[face.id] = TopoDS.Face_s(
            self._orient_shape(shape, face.sense, self.surfaces[face.surface].sense)
        )

    def _representations_agree(self, edge, samples: int = 32) -> bool:
        """Whether every pcurve matches the 3D curve on the same parameter interval."""

        curves = edge.TShape().Curves()
        pending = type(curves)()
        pending.Assign(curves)
        curve3d = None
        pcurves = []
        while not pending.IsEmpty():
            representation = pending.First()
            pending.RemoveFirst()
            if representation.IsCurve3D() and representation.Curve3D() is not None:
                curve3d = representation
            elif representation.IsCurveOnSurface() and representation.PCurve() is not None:
                pcurves.append((representation, representation.PCurve(), representation.Surface()))
                if representation.IsCurveOnClosedSurface() and representation.PCurve2() is not None:
                    pcurves.append((representation, representation.PCurve2(), representation.Surface()))
        if curve3d is None or not pcurves:
            return False
        first, last = curve3d.First(), curve3d.Last()
        located = curve3d.Curve3D()
        if not curve3d.Location().IsIdentity():
            located = located.Transformed(curve3d.Location().Transformation())
        limit = BRep_Tool.Tolerance_s(edge) + Precision.Confusion_s()
        for representation, pcurve, surface in pcurves:
            if abs(representation.First() - first) > 1.0e-7 or abs(representation.Last() - last) > 1.0e-7:
                return False
            if surface is None:
                return False
            if not representation.Location().IsIdentity():
                surface = surface.Transformed(representation.Location().Transformation())
            for index in range(samples + 1):
                parameter = first + (last - first) * index / samples
                uv = pcurve.Value(parameter)
                if located.Value(parameter).Distance(surface.Value(uv.X(), uv.Y())) > limit:
                    return False
        return True

    def _reconcile_parameter_flags(self) -> None:
        builder = BRep_Builder()
        changed = False
        for face in self.model.faces:
            if face.id not in self.face_shapes:
                continue
            shape = TopoDS.Face_s(self.context.Apply(self.face_shapes[face.id]))
            for candidate in subshapes(shape, TopAbs_EDGE):
                edge = TopoDS.Edge_s(candidate)
                if BRep_Tool.Degenerated_s(edge):
                    continue
                if BRep_Tool.SameParameter_s(edge) and BRep_Tool.SameRange_s(edge):
                    continue
                if not self._representations_agree(edge):
                    # A periodic copy can sit one turn away from the 3D range.
                    # SameRange brings that copy onto the 3D interval; keep the
                    # flag clear when the curves still do not meet.
                    if BRep_Tool.SameRange_s(edge):
                        continue
                    BRepLib.SameRange_s(edge, Precision.Confusion_s())
                    if not self._representations_agree(edge):
                        continue
                if not BRep_Tool.SameRange_s(edge):
                    builder.SameRange(edge, True)
                    changed = True
                if not BRep_Tool.SameParameter_s(edge):
                    builder.SameParameter(edge, True)
                    changed = True
        if changed:
            self.operations.append("reconcile_shared_edge_parameters")

    def _validate_source_boundaries(self):
        # Sample the original FIN geometry, independently of the reparameterized
        # output pcurve. Keep source tolerance and representation error separate.
        exceeded = {}
        for edge in self.model.edges:
            curve = BRep_Tool.Curve_s(self.edge_shapes[edge.id], 0.0, 0.0)
            source_tolerance = (
                edge.tolerance or self.precision / (2 * self.geometry.scale)
            ) * self.geometry.scale
            approximation = self._edge_tolerance(edge)
            validation = self.geometry.options.validation.linear_threshold(0.0)
            budget = source_tolerance + approximation + validation
            vertices = [
                (
                    self.geometry.point(self.points[self.vertices[i].point].position),
                    BRep_Tool.Tolerance_s(self.vertex_shapes[i]),
                )
                for i in (edge.start_vertex, edge.end_vertex)
                if i is not None
            ]
            for half_edge_id in edge.half_edges:
                trimmed = self._surface_trim(self.half_edges[half_edge_id].curve)
                if trimmed is None:
                    continue
                definition, support = trimmed
                pc = self._parameter_curve(support)
                surface = self._surface_geometry(support.surface)
                for sample in range(33):
                    t = (
                        definition.start_parameter
                        + (definition.end_parameter - definition.start_parameter) * sample / 32
                    )
                    uv = pc.Value(t)
                    point = surface.Value(uv.X(), uv.Y())
                    _, distance = project_curve(curve, point)
                    # Trim endpoints and their vertex neighborhoods carry their
                    # own source tolerances, distinct from the edge interior.
                    limit = max(
                        [
                            budget,
                            *(
                                tol + approximation
                                for p, tol in vertices
                                if p.Distance(point) <= tol
                            ),
                        ]
                    )
                    if distance > limit:
                        raise ValueError(
                            f"FIN {half_edge_id} differs from shared edge {edge.id} "
                            f"by {distance:g} (conversion limit {limit:g})"
                        )
                    if distance > source_tolerance:
                        previous = exceeded.get(edge.id, (0.0, source_tolerance, limit))
                        exceeded[edge.id] = max(previous, (distance, source_tolerance, limit))
                    self.geometry_checks["source_uv_samples"] += 1
        if exceeded:
            edge_id = max(exceeded, key=lambda i: exceeded[i][0])
            distance, tolerance, limit = exceeded[edge_id]
            source = self.edges[edge_id].source
            self.geometry_checks["edges_exceeding_source_tolerance"] = len(exceeded)
            self.diagnostics.append(
                Diagnostic(
                    code="occt.parametric_approximation",
                    severity=DiagnosticSeverity.WARNING,
                    kind=DiagnosticKind.INVALID,
                    fatal=False,
                    message=(
                        f"{len(exceeded)} edges have sampled FIN-to-output differences "
                        "above their source edge tolerance; conversion error budgets passed"
                    ),
                    location=SourceLocation(byte_offset=source.byte_range.start),
                    node_type=source.node_type,
                    schema_key=self.model.schema_key,
                    details={
                        "edge_count": len(exceeded),
                        "max_deviation_edge_id": edge_id,
                        "max_deviation": distance,
                        "source_edge_tolerance": tolerance,
                        "conversion_limit": limit,
                        "samples_per_fin": 33,
                        "target_unit": self.geometry.options.target_unit,
                    },
                )
            )
        self.operations.append("validate_source_fin_geometry")

    def _record_boundaries(self):
        def record_many(kind, source, shapes, note=None):
            relation = ShapeRelationKind.SPLIT if len(shapes) > 1 else ShapeRelationKind.DIRECT
            for shape in shapes:
                self._record(kind, source.id, source.source, shape, relation, note)

        source_vertices = set()
        for vertex in self.model.vertices:
            shape = self.context.Apply(self.vertex_shapes[vertex.id])
            record_many(SourceEntityKind.VERTEX, vertex, [shape])
            point = self.points[vertex.point]
            record_many(SourceEntityKind.POINT, point, [shape])
            source_vertices.add(hash(shape))
        source_edges = set()
        mapped_edges = {}
        curve_targets = defaultdict(dict)
        for edge in self.model.edges:
            shapes = subshapes(self.context.Apply(self.edge_shapes[edge.id]), TopAbs_EDGE)
            if not shapes:
                raise ValueError(f"source edge {edge.id} disappeared during seam construction")
            mapped_edges[edge.id] = shapes
            source_edges.update(hash(shape) for shape in shapes)
            record_many(SourceEntityKind.EDGE, edge, shapes)
            curve_ids = {edge.curve} if edge.curve is not None else set()
            for half_edge_id in edge.half_edges:
                half_edge = self.half_edges[half_edge_id]
                record_many(SourceEntityKind.HALF_EDGE, half_edge, shapes)
                if half_edge.curve is not None:
                    curve_ids.add(half_edge.curve)
            pending = list(curve_ids)
            while pending:
                curve_id = pending.pop()
                curve_targets[curve_id].update((hash(shape), shape) for shape in shapes)
                definition = self.curves[curve_id].definition
                basis = (
                    definition.basis_curve
                    if isinstance(definition, TrimmedCurve)
                    else definition.parameter_curve
                    if isinstance(definition, SurfaceParametricCurve)
                    else None
                )
                if basis is not None and basis not in curve_ids:
                    curve_ids.add(basis)
                    pending.append(basis)
        for curve_id, targets in curve_targets.items():
            record_many(SourceEntityKind.CURVE, self.curves[curve_id], list(targets.values()))
        for face in self.model.faces:
            shape = self.face_shapes[face.id]
            record_many(SourceEntityKind.FACE, face, [shape])
            self._record_surface(self.surfaces[face.surface], shape)
            wires = subshapes(shape, TopAbs_WIRE)
            wire_edges = [{hash(edge) for edge in subshapes(wire, TopAbs_EDGE)} for wire in wires]
            loop_targets = []
            for loop_id in face.loops:
                loop = self.loops[loop_id]
                if is_isolated_vertex_loop(loop, self.half_edges):
                    fin = self.half_edges[loop.half_edges[0]]
                    shapes = subshapes(
                        self.context.Apply(self.isolated_edge_shapes[fin.id]),
                        TopAbs_EDGE,
                    )
                    expected = {hash(edge) for edge in shapes}
                    candidates = [
                        i for i, values in enumerate(wire_edges) if expected and expected <= values
                    ]
                    if len(candidates) != 1:
                        # Seam construction can replace the apex edge. The fin
                        # still denotes the degenerated boundary on this face.
                        candidates = [
                            i
                            for i, wire in enumerate(wires)
                            if any(
                                BRep_Tool.Degenerated_s(TopoDS.Edge_s(edge))
                                for edge in subshapes(wire, TopAbs_EDGE)
                            )
                        ]
                        if len(candidates) == 1:
                            shapes = [
                                edge
                                for edge in subshapes(wires[candidates[0]], TopAbs_EDGE)
                                if BRep_Tool.Degenerated_s(TopoDS.Edge_s(edge))
                            ]
                    if len(candidates) != 1 or not shapes:
                        raise ValueError(
                            f"isolated vertex loop {loop_id} degenerated edge disappeared"
                        )
                    record_many(SourceEntityKind.HALF_EDGE, fin, shapes)
                    loop_targets.append((loop_id, candidates[0]))
                    continue
                else:
                    expected = {
                        hash(edge)
                        for hid in loop.half_edges
                        for edge in mapped_edges[self.half_edges[hid].edge]
                    }
                    # A shared edge split by a neighbouring seam is replaced by
                    # several edges. Only the pieces that remain on this face
                    # identify its wire; the others belong to the neighbour.
                    on_face = set().union(*wire_edges)
                    present = expected & on_face
                    if present:
                        expected = present
                candidates = [i for i, values in enumerate(wire_edges) if expected <= values]
                if len(candidates) != 1:
                    raise ValueError(
                        f"source loop {loop_id} cannot be mapped to its final OCCT wire"
                    )
                loop_targets.append((loop_id, candidates[0]))
            counts = Counter(index for _, index in loop_targets)
            for loop_id, index in loop_targets:
                loop = self.loops[loop_id]
                relation = (
                    ShapeRelationKind.MERGED if counts[index] > 1 else ShapeRelationKind.DIRECT
                )
                self._record(SourceEntityKind.LOOP, loop.id, loop.source, wires[index], relation)
            for kind, known in ((TopAbs_EDGE, source_edges), (TopAbs_VERTEX, source_vertices)):
                for generated in subshapes(shape, kind):
                    if hash(generated) not in known:
                        self._record(
                            SourceEntityKind.FACE,
                            face.id,
                            face.source,
                            generated,
                            ShapeRelationKind.GENERATED,
                            "periodic seam or split-boundary vertex",
                        )

    def _open_shell_diagnostic(self, shell, census: dict[str, int]) -> Diagnostic:
        return Diagnostic(
            code="occt.solid_not_closed",
            severity=DiagnosticSeverity.ERROR,
            kind=DiagnosticKind.INVALID,
            fatal=True,
            message=(
                f"shell {shell.id} stays an open shell: {census['free']} free boundaries, "
                f"{census['seam']} seam edges, {census['degenerated']} degenerated edges"
            ),
            location=SourceLocation(byte_offset=shell.source.byte_range.start),
            node_type=shell.source.node_type,
            node_id=(
                None
                if shell.source.node_id is None or shell.source.node_id < 0
                else shell.source.node_id
            ),
            schema_key=self.model.schema_key,
            details={
                "shell_id": shell.id,
                "free_boundaries": census["free"],
                "seam_edges": census["seam"],
                "degenerated_edges": census["degenerated"],
                "shared_edges": census["shared"],
            },
        )

    def _build_bodies(self):
        results = []
        self.rendered_bodies = []
        for body in self.model.bodies:
            regions = [self.regions[i] for i in body.regions]
            material = [r for r in regions if r.kind is RegionKind.SOLID]
            if body.kind is BodyKind.SHEET:
                material = regions
            shell_shapes = {}
            solids = []
            body_open = False
            for region in material:
                solid = BRepBuilderAPI_MakeSolid()
                region_shells = []
                region_open = False
                for shell_id in region.shells:
                    shell = self.shells[shell_id]
                    faces = frozenset((*shell.back_faces, *shell.front_faces))
                    if not faces:
                        raise ValueError(f"shell {shell_id} has no boundary faces")
                    shape = TopoDS_Shell()
                    builder = BRep_Builder()
                    builder.MakeShell(shape)
                    for face_id in sorted(faces):
                        builder.Add(shape, self.face_shapes[face_id])
                    if faces in shell_shapes:
                        raise ValueError("material regions repeat the same boundary shell")
                    shell_shapes[faces] = shape
                    region_shells.append((shell, shape))
                    if body.kind is BodyKind.SOLID and not BRep_Tool.IsClosed_s(shape):
                        region_open = True
                        census = boundary_census(shape)
                        self.diagnostics.append(
                            self._open_shell_diagnostic(shell, census)
                        )
                        self.operations.append("open_shell_retained")
                    else:
                        solid.Add(shape)
                if body.kind is BodyKind.SHEET or region_open:
                    body_open = body_open or region_open
                    for _shell, shape in region_shells:
                        solids.append((region.id, shape))
                    if region_open:
                        self._record(
                            SourceEntityKind.REGION,
                            region.id,
                            region.source,
                            region_shells[0][1],
                            ShapeRelationKind.SPLIT,
                            "open shell; solid conversion failed",
                        )
                elif body.kind is BodyKind.SOLID:
                    if not solid.IsDone():
                        raise ValueError(f"region {region.id} solid construction failed")
                    region_shape = solid.Solid()
                    solids.append((region.id, region_shape))
                    self._record(SourceEntityKind.REGION, region.id, region.source, region_shape)
            body_shape = self._make_root(solids)
            for region in regions:
                for shell_id in region.shells:
                    shell = self.shells[shell_id]
                    key = frozenset((*shell.back_faces, *shell.front_faces))
                    if key not in shell_shapes:
                        raise ValueError(f"shell {shell_id} has no matching material boundary")
                    shape = shell_shapes[key]
                    self._record(
                        SourceEntityKind.SHELL,
                        shell.id,
                        shell.source,
                        shape,
                        ShapeRelationKind.MERGED,
                    )
                    if region not in material or body.kind is BodyKind.SHEET:
                        self._record(
                            SourceEntityKind.REGION,
                            region.id,
                            region.source,
                            shape,
                            ShapeRelationKind.SPLIT
                            if len(region.shells) > 1
                            else ShapeRelationKind.DIRECT,
                            "source region boundary",
                        )
            self._record(SourceEntityKind.BODY, body.id, body.source, body_shape)
            self.rendered_bodies.append((body.id, body_shape, body_open))
            results.append((body.id, body_shape))
        self.operations.append("solid_with_source_region_shells")
        return results
