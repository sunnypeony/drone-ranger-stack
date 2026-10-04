import open3d as o3d
import numpy as np

PROJECT = "/Users/你的用户名/backyard"

print("Loading point cloud...")
pcd = o3d.io.read_point_cloud(f"{PROJECT}/dense/fused.ply")
print(f"  Points: {len(pcd.points):,}")

# 1. Statistical denoising (outlier removal)
print("Denoising...")
pcd, _ = pcd.remove_statistical_outlier(
    nb_neighbors=20, 
    std_ratio=2.0)
print(f" After denoising: {len(pcd.points):,} points")

# 2. Estimate normals and orient them
print("Estimating normals...")
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
# Orient normals towards camera location (assumed to be at [0, 0, 5])
pcd.orient_normals_towards_camera_location(
    camera_location=np.array([0, 0, 5]))


# 3. Poisson surface reconstruction
print("Poisson reconstruction...")
mesh, densities = o3d.geometry.TriangleMesh\
    .create_from_point_cloud_poisson(pcd, depth=10)

# 4. Remove low-density vertices (keep top 95% density, (remove peripheral areas with poor reconstruction quality)
print("Removing low-density vertices...")
density_threshold = np.quantile(np.asarray(densities), 0.05)
vertices_to_remove = np.asarray(densities) < density_threshold
mesh.remove_vertices_by_mask(vertices_to_remove)
print(f" After removing low-density vertices: {len(mesh.triangles):,} triangles")


# 5. Save the mesh
output = f"{PROJECT}/mesh/scene_mesh.obj"
o3d.io.write_triangle_mesh(output, mesh)
print(f"Saved: {output}  ({len(mesh.triangles):,} triangles)")