
## Convention for data version control
dataset_id: backyard_20261003_001

platform: handheld

sensors:
  camera: OAK-D Lite
  lidar: YDLIDAR T-mini Plus

topics:
  - /oak/rgb/image_raw
  - /oak/depth/image_raw
  - /scan
  - /tf
  - /tf_static
  - /oak/imu/data

environment:
  scene: backyard
  lighting: daylight

notes:
  - walking loop around table
  - vegetation present