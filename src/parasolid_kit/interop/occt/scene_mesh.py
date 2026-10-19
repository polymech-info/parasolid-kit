"""Tessellate converted Parasolid bodies into the viewer's PSM payload.

Placements are not written. The Rust capi appends the verified instance trailer.
An open shell is meshed as a sheet; solid conversion stays a separate failure.
"""

from __future__ import annotations

import struct
import sys
from math import sqrt
from pathlib import Path

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.IMeshTools import IMeshTools_Parameters
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound

from ...brep.topology import BodyKind
from .geometry import GeometryFactory
from .options import UNIT_TO_METRES, LengthUnit, OcctConversionOptions, mesh_deflection
from .parametric import ParametricTopologyBuilder, RefusedFaces

MAGIC = 0x314D5350
ANGLE = 12.0 * 3.141592653589793 / 180.0
DEFAULT_SCHEMA = r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\schema_18"


def _mesh_shape(shape, working_unit: LengthUnit, to_source: float):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    if box.IsVoid():
        return [], [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    diagonal = sqrt((xmax - xmin) ** 2 + (ymax - ymin) ** 2 + (zmax - zmin) ** 2)
    params = IMeshTools_Parameters()
    params.Deflection = mesh_deflection(diagonal, working_unit)
    params.Angle = ANGLE
    params.AngleInterior = ANGLE
    params.InParallel = False
    params.Relative = False
    params.ControlSurfaceDeflection = True
    BRepMesh_IncrementalMesh(shape, params)
    triangles = []
    bounds = None
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = face.Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is not None and triangulation.NbTriangles() > 0:
            transform = location.Transformation()
            reversed_face = int(face.Orientation()) == int(TopAbs_REVERSED)
            for index in range(1, triangulation.NbTriangles() + 1):
                triangle = triangulation.Triangle(index)
                first, second, third = triangle.Get()
                if reversed_face:
                    second, third = third, second
                points = []
                for node in (first, second, third):
                    point = triangulation.Node(node).Transformed(transform)
                    points.append(
                        (point.X() * to_source, point.Y() * to_source, point.Z() * to_source)
                    )
                edge_a = [points[1][axis] - points[0][axis] for axis in range(3)]
                edge_b = [points[2][axis] - points[0][axis] for axis in range(3)]
                normal = [
                    edge_a[1] * edge_b[2] - edge_a[2] * edge_b[1],
                    edge_a[2] * edge_b[0] - edge_a[0] * edge_b[2],
                    edge_a[0] * edge_b[1] - edge_a[1] * edge_b[0],
                ]
                length = sqrt(sum(value * value for value in normal))
                normal = [0.0, 0.0, 1.0] if length <= 1e-15 else [value / length for value in normal]
                triangles.append((points, normal))
                for point in points:
                    if bounds is None:
                        bounds = [point[0], point[1], point[2], point[0], point[1], point[2]]
                    else:
                        for axis in range(3):
                            bounds[axis] = min(bounds[axis], point[axis])
                            bounds[axis + 3] = max(bounds[axis + 3], point[axis])
        explorer.Next()
    if bounds is None:
        bounds = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
    return triangles, bounds


def _kind_code(body, open_shell: bool) -> int:
    if body.kind is BodyKind.WIRE:
        return 2
    if body.kind is BodyKind.GENERAL:
        return 6
    if open_shell or body.kind is BodyKind.SHEET:
        return 3
    return 1


def _converted_faces(builder) -> dict:
    """Mesh faces that converted when assembly stopped on a refused face.

    The missing face is left out. Faces are not sewn, and solid validation is
    unchanged: ``build`` still refuses the document.
    """

    rendered = {}
    for body in builder.model.bodies:
        compound = TopoDS_Compound()
        maker = BRep_Builder()
        maker.MakeCompound(compound)
        missing = False
        for region_id in body.regions:
            region = builder.regions[region_id]
            for shell_id in region.shells:
                shell = builder.shells[shell_id]
                for face_id in (*shell.back_faces, *shell.front_faces):
                    shape = builder.face_shapes.get(face_id)
                    if shape is None:
                        missing = True
                        continue
                    maker.Add(compound, shape)
        rendered[body.id] = (compound, missing)
    return rendered


def scene_mesh_bytes(model, *, source_unit: LengthUnit = "m") -> bytes:
    """OCCT triangles for each body, in source units. The instance trailer is empty.

    Faces are built in millimetres, which is the conversion that validates the
    housing spheres. Triangle coordinates are then scaled back to ``source_unit``
    so the verified placement trailer applies without a second transform.
    """

    options = OcctConversionOptions(source_unit=source_unit, target_unit="mm")
    builder = ParametricTopologyBuilder(model, GeometryFactory(options))
    try:
        builder.build()
    except RefusedFaces as refused:
        for item in refused.face_diagnostics:
            print(item.message, file=sys.stderr)
        rendered = _converted_faces(builder)
    else:
        rendered = {
            body_id: (shape, open_shell) for body_id, shape, open_shell in builder.rendered_bodies
        }
    to_source = UNIT_TO_METRES["mm"] / UNIT_TO_METRES[source_unit]
    out = bytearray()

    def u32(value: int) -> None:
        out.extend(struct.pack("<I", value))

    def i32(value: int) -> None:
        out.extend(struct.pack("<i", value))

    def f32(value: float) -> None:
        out.extend(struct.pack("<f", value))

    u32(MAGIC)
    u32(2)
    u32(1 if model.complete else 0)
    u32(len(model.bodies))
    for ordinal, body in enumerate(model.bodies):
        shape, open_shell = rendered[body.id]
        triangles, bounds = _mesh_shape(shape, "mm", to_source)
        node_id = -1 if body.source.node_id is None else body.source.node_id
        u32(ordinal)
        u32(_kind_code(body, open_shell))
        i32(node_id)
        u32(len(triangles))
        for value in bounds:
            f32(value)
        for points, normal in triangles:
            for point in points:
                for value in point:
                    f32(value)
            for _corner in range(3):
                for value in normal:
                    f32(value)
    u32(0)
    return bytes(out)


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print("usage: scene_mesh.py <transmit> <out.psm> [schema_dir]", file=sys.stderr)
        return 2
    from parasolid_kit import read_brep

    schema = argv[3] if len(argv) > 3 else DEFAULT_SCHEMA
    parsed = read_brep(argv[1], schema_dir=schema)
    payload = scene_mesh_bytes(parsed.brep)
    Path(argv[2]).write_bytes(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
