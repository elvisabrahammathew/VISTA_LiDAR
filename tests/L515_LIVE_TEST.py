import open3d as o3d

print("Starting Intel RealSense L515...")

sensor = o3d.t.io.RealSenseSensor()

# Initialize using a supported default configuration
sensor.init_sensor()

# Start streaming
sensor.start_capture(start_record=False)

print("L515 stream started successfully.")

try:
    while True:
        rgbd = sensor.capture_frame(
            wait=True,
            align_depth_to_color=True
        )

        print(
            "RGB:",
            rgbd.color.rows,
            "x",
            rgbd.color.columns,
            "| Depth:",
            rgbd.depth.rows,
            "x",
            rgbd.depth.columns
        )

except KeyboardInterrupt:
    print("\nStopping sensor...")

finally:
    sensor.stop_capture()