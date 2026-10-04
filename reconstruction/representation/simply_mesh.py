# # macOS 上 Blender 命令行路径
# /Applications/Blender.app/Contents/MacOS/Blender \
#     --background --python ~/backyard/simplify_mesh.py
# example usage:
# `blender --background --python simplify_mesh.py`

# Count faces in OBJ file (lines starting with 'f')
# grep -c "^f " ~/backyard_reconstruction/mesh/scene_clean.obj
# Target: < 500000

import bpy

INPUT  = "/Users/你的用户名/backyard/mesh/scene_mesh.obj"
OUTPUT = "/Users/你的用户名/backyard/mesh/scene_clean.obj"
RATIO  = 0.05   # 保留 5% 的面

# empty the scene
bpy.ops.wm.read_factory_settings(use_empty=True)

# import the mesh
bpy.ops.wm.obj_import(filepath=INPUT)
obj = bpy.context.selected_objects[0]

print(f"Original: {len(obj.data.polygons):,} faces")

# add a decimate modifier and apply it
mod = obj.modifiers.new(name="Decimate", type="DECIMATE")
mod.ratio = RATIO
# apply the modifier
bpy.ops.object.modifier_apply(modifier="Decimate")

print(f"Simplified: {len(obj.data.polygons):,} faces")

# save the simplified mesh
bpy.ops.wm.obj_export(filepath=OUTPUT, export_selected_objects=True)
print(f"Saved: {OUTPUT}")
