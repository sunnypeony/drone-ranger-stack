#!/usr/bin/env python3
"""Print the prim tree, applied API schemas and physics attributes of a USD file."""
import argparse
import sys

from pxr import Usd, UsdGeom, UsdPhysics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("usd", help="path to .usd/.usda/.usdc")
    a = p.parse_args()
    stage = Usd.Stage.Open(a.usd)
    if stage is None:
        sys.exit(f"cannot open {a.usd}")
    print(f"file: {a.usd}")
    print(f"defaultPrim: {stage.GetDefaultPrim().GetPath()}  upAxis: {UsdGeom.GetStageUpAxis(stage)}  "
          f"metersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")
    ok = True
    for prim in stage.Traverse():
        depth = str(prim.GetPath()).count("/") - 1
        apis = prim.GetAppliedSchemas()
        print(f"{'  ' * depth}{prim.GetPath()} [{prim.GetTypeName()}] apis={list(apis)}")
        if prim.IsA(UsdGeom.Mesh):
            m = UsdGeom.Mesh(prim)
            pts = m.GetPointsAttr().Get()
            fvc = m.GetFaceVertexCountsAttr().Get()
            nrm = m.GetNormalsAttr().Get()
            col = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
            print(f"{'  ' * depth}  points={len(pts)} faces={len(fvc)} normals={len(nrm) if nrm else 0} "
                  f"displayColor={'%d (%s)' % (len(col.Get()), col.GetInterpolation()) if col and col.HasValue() else 'none'}")
            print(f"{'  ' * depth}  extent={m.GetExtentAttr().Get()}")
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                print(f"{'  ' * depth}  physics:approximation="
                      f"{UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()}")
            print(f"{'  ' * depth}  collisionEnabled={UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()} "
                  f"rigidBody={prim.HasAPI(UsdPhysics.RigidBodyAPI)}")
            ok &= prim.HasAPI(UsdPhysics.CollisionAPI) and prim.HasAPI(UsdPhysics.MeshCollisionAPI)
    print("collision schemas OK" if ok else "MISSING collision schemas")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
