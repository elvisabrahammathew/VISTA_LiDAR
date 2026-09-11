import open3d as o3d

print("Open3D version:", o3d.__version__)
print("Searching for RealSense devices...")

devices = o3d.t.io.RealSenseSensor.enumerate_devices()

print("Devices found:")
print(devices)