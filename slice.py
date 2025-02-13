import open3d as o3d
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Use a non-interactive backend
import matplotlib.pyplot as plt
import imageio
import os

# Load PCD file
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

if pcd.is_empty():
    raise ValueError("The point cloud is empty. Please check your PCD file.")

# Function to rotate the point cloud
def rotate_point_cloud(pcd, angle_z_deg, angle_x_deg, angle_y_deg):
    angle_z_rad = np.radians(angle_z_deg)
    angle_x_rad = np.radians(angle_x_deg)
    angle_y_rad = np.radians(angle_y_deg)

    rotation_matrix_z = np.array([
        [np.cos(angle_z_rad), -np.sin(angle_z_rad), 0],
        [np.sin(angle_z_rad), np.cos(angle_z_rad), 0],
        [0, 0, 1]
    ])

    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x_rad), -np.sin(angle_x_rad)],
        [0, np.sin(angle_x_rad), np.cos(angle_x_rad)]
    ])

    rotation_matrix_y = np.array([
        [np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
        [0, 1, 0],
        [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]
    ])

    combined_rotation_matrix = np.dot(rotation_matrix_y, np.dot(rotation_matrix_x, rotation_matrix_z))
    points = np.asarray(pcd.points)
    rotated_points = np.dot(points, combined_rotation_matrix.T)
    
    pcd.points = o3d.utility.Vector3dVector(rotated_points)
    return pcd

# Rotate the point cloud
pcd_rotated = rotate_point_cloud(pcd, 0, 90, 0)

# Voxelization: Downsample the rotated point cloud using a voxel size (10cm resolution)
voxel_size = 0.1  # 10 cm voxel grid
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_rotated, voxel_size)

# Extract voxel centers
voxels = voxel_grid.get_voxels()
voxel_centers = np.array([voxel.grid_index for voxel in voxels])

# Convert voxel indices to real-world coordinates (X, Y, Z) in centimeters
x_coords = voxel_centers[:, 0] * voxel_size  
y_coords = voxel_centers[:, 1] * voxel_size  
z_coords = voxel_centers[:, 2] * voxel_size  

# Define directory to store images
output_dir = "slices"
os.makedirs(output_dir, exist_ok=True)

# Create slices and save images
images = []
y_levels = np.unique(y_coords)  # Change from x_coords or z_coords to y_coords
for y in y_levels:
    fig, ax = plt.subplots(figsize=(6, 6))
    mask = (y_coords == y)  # Mask based on y_coords
    ax.scatter(x_coords[mask], z_coords[mask], c='black', s=1)  # Scatter with x, z coordinates
    ax.set_xlim(min(x_coords), max(x_coords))
    ax.set_ylim(min(z_coords), max(z_coords))
    ax.set_title(f"Slice at Y = {y:.2f}")
    ax.axis('off')
    
    filename = os.path.join(output_dir, f"slice_{int(y * 100)}.png")
    plt.savefig(filename)
    plt.close(fig)
    images.append(imageio.imread(filename))

# Create GIF
imageio.mimsave("occupancy_grid_slices.gif", images, duration=0.5)

print("GIF saved as occupancy_grid_slices.gif")
import open3d as o3d
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Use a non-interactive backend
import matplotlib.pyplot as plt
import imageio
import os

# Load PCD file
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

if pcd.is_empty():
    raise ValueError("The point cloud is empty. Please check your PCD file.")

# Function to rotate the point cloud
def rotate_point_cloud(pcd, angle_z_deg, angle_x_deg, angle_y_deg):
    angle_z_rad = np.radians(angle_z_deg)
    angle_x_rad = np.radians(angle_x_deg)
    angle_y_rad = np.radians(angle_y_deg)

    rotation_matrix_z = np.array([
        [np.cos(angle_z_rad), -np.sin(angle_z_rad), 0],
        [np.sin(angle_z_rad), np.cos(angle_z_rad), 0],
        [0, 0, 1]
    ])

    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x_rad), -np.sin(angle_x_rad)],
        [0, np.sin(angle_x_rad), np.cos(angle_x_rad)]
    ])

    rotation_matrix_y = np.array([
        [np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
        [0, 1, 0],
        [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]
    ])

    combined_rotation_matrix = np.dot(rotation_matrix_y, np.dot(rotation_matrix_x, rotation_matrix_z))
    points = np.asarray(pcd.points)
    rotated_points = np.dot(points, combined_rotation_matrix.T)
    
    pcd.points = o3d.utility.Vector3dVector(rotated_points)
    return pcd

# Rotate the point cloud
pcd_rotated = rotate_point_cloud(pcd, 0, 90, 0)

# Voxelization: Downsample the rotated point cloud using a voxel size (10cm resolution)
voxel_size = 0.1  # 10 cm voxel grid
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_rotated, voxel_size)

# Extract voxel centers
voxels = voxel_grid.get_voxels()
voxel_centers = np.array([voxel.grid_index for voxel in voxels])

# Convert voxel indices to real-world coordinates (X, Y, Z) in centimeters
x_coords = voxel_centers[:, 0] * voxel_size  
y_coords = voxel_centers[:, 1] * voxel_size  
z_coords = voxel_centers[:, 2] * voxel_size  

# Define directory to store images
output_dir = "slices"
os.makedirs(output_dir, exist_ok=True)

# Create slices and save images
images = []
y_levels = np.unique(y_coords)  # Change from x_coords or z_coords to y_coords
for y in y_levels:
    fig, ax = plt.subplots(figsize=(6, 6))
    mask = (y_coords == y)  # Mask based on y_coords
    ax.scatter(x_coords[mask], z_coords[mask], c='black', s=1)  # Scatter with x, z coordinates
    ax.set_xlim(min(x_coords), max(x_coords))
    ax.set_ylim(min(z_coords), max(z_coords))
    ax.set_title(f"Slice at Y = {y:.2f}")
    ax.axis('off')
    
    filename = os.path.join(output_dir, f"slice_{int(y * 100)}.png")
    plt.savefig(filename)
    plt.close(fig)
    images.append(imageio.imread(filename))

# Create GIF
imageio.mimsave("occupancy_grid_slices.gif", images, duration=0.5)

print("GIF saved as occupancy_grid_slices.gif")

