import open3d as o3d
import numpy as np

# Load the PCD file
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

# Print basic information about the point cloud
print(pcd)
print("Number of points:", len(pcd.points))

# Voxel downsample the point cloud (adjust voxel size as needed)
voxel_size = 0.2  # Adjust voxel size based on your data
voxel_pcd = pcd.voxel_down_sample(voxel_size)

# Convert the point cloud to a voxel grid for visualization
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

# Check if Open3D can create a window (i.e., check if GUI is supported)
try:
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="PointCloud Comparison", width=800, height=600)
    gui_available = True
except RuntimeError as e:
    print("Error: Unable to create a graphical window. Running in headless mode.")
    gui_available = False

if gui_available:
    # Add the original point cloud and voxel grid to the visualizer
    vis.add_geometry(pcd)
    vis.add_geometry(voxel_grid)

    # Run the visualizer
    vis.run()
    # Close the visualizer window after usage
    vis.destroy_window()

else:
    print("Running in headless mode, saving images.")
    # Create an offscreen visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)  # Create the window but don't display it

    # Add the point cloud and voxel grid
    vis.add_geometry(pcd)
    vis.add_geometry(voxel_grid)

    # Capture images
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image("original_pointcloud.png")
    vis.capture_screen_image("voxelized_pointcloud.png")

    # Close the offscreen visualizer
    vis.destroy_window()

    print("Images saved as 'original_pointcloud.png' and 'voxelized_pointcloud.png'.")

