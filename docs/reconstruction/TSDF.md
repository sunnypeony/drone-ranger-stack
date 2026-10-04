
# TSDF

[TSDF](https://www.open3d.org/docs/latest/tutorial/t_reconstruction_system/integration.html)


3DGS mode -> render many depth maps from training camera poses-> TSDF fusion -> mesh / occupancy -> Isaac Sim collision

Truncated Signed Distance Field (Computer Vision/Robotics)
Definition: A spatial grid representation where each voxel stores the signed distance to the nearest object surface.
Truncation: The distance values are cut off (truncated) within a small, fixed band close to the surface to save memory.
Application: Fuses noisy 3D depth data from sensors (like LiDAR or RGB-D cameras) to build smooth, real-time 3D room or object models. [1, 2, 3]
A Truncated Signed Distance Field (TSDF) is a 3D voxel array representing objects within a volume of space in which each voxel is labeled with the distance to the nearest surface. The TSDF algorithm can be efficiently parallelized on a general-purpose graphics processor, which allows data from RGB-D cameras to be integrated into the volume in real time.
Numerous observations of an object from different perspectives average out noise and errors due to specular highlights and interreflections, producing a smooth continuous surface. This is a key advantage over equivalent point-cloud-centric strategies, which require additional processing to distinguish between engineered features and erroneous artifacts in the scan data. The volume can be converted to a triangular mesh using the Marching Cubes algorithm and then handed off to application-specific processes.
