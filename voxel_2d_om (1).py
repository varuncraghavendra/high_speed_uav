import open3d as o3d
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Use a non-interactive backend
import matplotlib.pyplot as plt

# Load PCD file
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

# Voxelization: Downsample the point cloud using a voxel size
voxel_size = 0.1  # 10 cm voxel grid
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

# Extract voxel centers
voxel_centers = np.array([voxel.grid_index for voxel in voxel_grid.get_voxels()])
x_coords = voxel_centers[:, 0] * voxel_size  # Convert to real-world scale
y_coords = voxel_centers[:, 1] * voxel_size  # Y as height

# Define grid resolution and size
grid_size = voxel_size  # Each cell corresponds to one voxel
x_min, x_max = np.min(x_coords), np.max(x_coords)
y_min, y_max = np.min(y_coords), np.max(y_coords)

# Compute grid dimensions
grid_width = int((x_max - x_min) / grid_size) + 1
grid_height = int((y_max - y_min) / grid_size) + 1

# Initialize Occupancy Grid (0 = free, 1 = occupied)
occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

# Convert voxel positions to grid indices
x_indices = ((x_coords - x_min) / grid_size).astype(int)
y_indices = ((y_coords - y_min) / grid_size).astype(int)

# Mark occupied cells
for i in range(len(x_indices)):
    occupancy_grid[y_indices[i], x_indices[i]] = 1  # Y controls rows, X controls columns

# Save Occupancy Grid as Image
plt.imshow(occupancy_grid, cmap="gray", origin="lower")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("2D Occupancy Grid from Voxelized PCD (XY Projection)")

# Save to file
plt.savefig("occupancy_grid_xy_voxelized.png", dpi=300, bbox_inches="tight")
print("Voxelized occupancy grid saved as 'occupancy_grid_xy_voxelized.png'.")
