import open3d as o3d
import numpy as np

print("Starting Intel RealSense L515 3D Viewer...")

devices = o3d.t.io.RealSenseSensor.enumerate_devices()

if len(devices) == 0:
    raise RuntimeError("No RealSense device detected.")

print("RealSense device detected.")

sensor = o3d.t.io.RealSenseSensor()

sensor.init_sensor()
sensor.start_capture(False)

print("Sensor streaming started.")

metadata = sensor.get_metadata()

intrinsic_matrix = metadata.intrinsics.intrinsic_matrix
depth_scale = metadata.depth_scale

print("Camera intrinsics:")
print(intrinsic_matrix)

print("Depth scale:")
print(depth_scale)

vis = o3d.visualization.Visualizer()

vis.create_window(
    window_name="Intel RealSense L515 - Live 3D Point Cloud",
    width=1280,
    height=720
)

point_cloud = o3d.geometry.PointCloud()

first_frame = True

try:

    while True:

        rgbd = sensor.capture_frame(
            wait=True,
            align_depth_to_color=True
        )

        if rgbd is None:
            continue

        pcd_t = o3d.t.geometry.PointCloud.create_from_rgbd_image(
            rgbd,
            intrinsic_matrix,
            depth_scale=depth_scale,
            depth_max=5.0,
            stride=2
        )

        pcd = pcd_t.to_legacy()

        pcd = pcd.voxel_down_sample(
            voxel_size=0.02
        )

        pcd.transform([
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]
        ])

        point_cloud.points = pcd.points
        point_cloud.colors = pcd.colors

        if first_frame:

            vis.add_geometry(point_cloud)

            options = vis.get_render_option()
            options.point_size = 2.0
            options.background_color = np.asarray(
                [0.05, 0.05, 0.05]
            )

            first_frame = False

        else:

            vis.update_geometry(point_cloud)

        if not vis.poll_events():
            break

        vis.update_renderer()

except KeyboardInterrupt:

    print("Stopping viewer...")

finally:

    sensor.stop_capture()
    vis.destroy_window()

    print("Sensor stopped.")