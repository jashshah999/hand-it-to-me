import pyrealsense2 as rs

# Create a context object to manage the RealSense device
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
x=320 
y=320
# Wait for a coherent pair of frames: depth and color
frames = pipeline.wait_for_frames()

# Retrieve the depth frame and get its depth value at the given pixel
depth_frame = frames.get_depth_frame()
depth_value = depth_frame.get_distance(x, y)

# Retrieve the intrinsic properties of the depth sensor
intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

# Convert the pixel to a 3D point in the camera coordinate system
point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth_value)

# Print the 3D point and depth value
print("Pixel: ({}, {}) -> Point: ({}, {}, {}) [Depth = {}]".format(x, y, point[0], point[1], point[2], depth_value))

# Stop the pipeline and release resources
pipeline.stop()
