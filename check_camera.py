import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize context and get available devices
context = rs.context()
devices = context.query_devices()

# List available devices
print("Available RealSense cameras:")
for i, device in enumerate(devices):
    print(f"{i}: {device.get_info(rs.camera_info.name)}")

# User selects a camera
selected_index = int(input("Select a camera index: "))
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(devices[selected_index].get_info(rs.camera_info.serial_number))

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a frame
        frames = pipeline.wait_for_frames()
        rgb_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not rgb_frame or not depth_frame:
            continue

        # Convert images to numpy arrays
        rgb_image = np.asanyarray(rgb_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Show RGB image
        cv2.imshow('RGB Image', rgb_image)

        # Show Depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
        cv2.imshow('Depth Image', depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()