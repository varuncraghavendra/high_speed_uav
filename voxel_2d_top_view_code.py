import open3d as o3d
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Use a non-interactive backend
import matplotlib.pyplot as plt

# Load PCD file
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

if pcd.is_empty():
    raise ValueError("The point cloud is empty. Please check your PCD file.")

# Rotation Matrix for aligning the point cloud (Z-axis, X-axis, and Y-axis rotations)
def rotate_point_cloud(pcd, angle_z_deg, angle_x_deg, angle_y_deg):
    # Convert angles to radians
    angle_z_rad = np.radians(angle_z_deg)
    angle_x_rad = np.radians(angle_x_deg)
    angle_y_rad = np.radians(angle_y_deg)
    
    # Rotation matrix for rotating about the Z-axis (top-down view)
    rotation_matrix_z = np.array([
        [np.cos(angle_z_rad), -np.sin(angle_z_rad), 0],
        [np.sin(angle_z_rad), np.cos(angle_z_rad), 0],
        [0, 0, 1]
    ])
    
    # Rotation matrix for rotating about the X-axis (flipping view)
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x_rad), -np.sin(angle_x_rad)],
        [0, np.sin(angle_x_rad), np.cos(angle_x_rad)]
    ])
    
    # Rotation matrix for rotating about the Y-axis
    rotation_matrix_y = np.array([
        [np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
        [0, 1, 0],
        [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]
    ])
    
    # Combined rotation matrix (rotate first Z, then Y, then X)
    combined_rotation_matrix = np.dot(rotation_matrix_y, np.dot(rotation_matrix_x, rotation_matrix_z))
    
    # Apply the rotation to the point cloud's points directly
    points = np.asarray(pcd.points)  # Convert points to a NumPy array
    rotated_points = np.dot(points, combined_rotation_matrix.T)  # Apply combined rotation matrix
    
    # Update point cloud with rotated points
    pcd.points = o3d.utility.Vector3dVector(rotated_points)
    return pcd

# Rotate the point cloud 
pcd_rotated = rotate_point_cloud(pcd, 0, 90, 0)

# Voxelization: Downsample the rotated point cloud using a voxel size
voxel_size = 0.1  # 10 cm voxel grid
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_rotated, voxel_size)

# Extract voxel centers
voxels = voxel_grid.get_voxels()
voxel_centers = np.array([voxel.grid_index for voxel in voxels])

# Convert voxel indices to real-world coordinates (X, Y)
x_coords = voxel_centers[:, 0] * voxel_size  # X coordinates
y_coords = voxel_centers[:, 1] * voxel_size  # Y coordinates

# Define grid resolution and size
grid_size = voxel_size  # Each cell corresponds to one voxel
x_min, x_max = np.min(x_coords), np.max(x_coords)
y_min, y_max = np.min(y_coords), np.max(y_coords)

# Compute grid dimensions
grid_width = int((x_max - x_min) / grid_size) + 1
grid_height = int((y_max - y_min) / grid_size) + 1

# Initialize Occupancy Grid (0 = free, 1 = occupied)
occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

# Convert voxel positions to grid indices (2D projection)
x_indices = ((x_coords - x_min) / grid_size).astype(int)
y_indices = ((y_coords - y_min) / grid_size).astype(int)

# Mark occupied cells
for i in range(len(x_indices)):
    occupancy_grid[y_indices[i], x_indices[i]] = 1  # Y controls rows, X controls columns

# Save Occupancy Grid as Image (Top-view)
plt.imshow(occupancy_grid, cmap="gray", origin="lower", extent=[x_min, x_max, y_min, y_max])
plt.xlabel("X-axis (meters)")
plt.ylabel("Y-axis (meters)")
plt.title("Top-View 2D Occupancy Grid from Rotated PCD (Z + X + Y Rotation)")

# Save to file
plt.savefig("occupancy_grid_rotated_top_view_Z_X_Y.png", dpi=300, bbox_inches="tight")
print("Voxelized top-view occupancy grid saved as 'occupancy_grid_rotated_top_view_Z_X_Y.png'.")

