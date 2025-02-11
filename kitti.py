import open3d as o3d
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import gc  # Garbage collection

# Function to read KITTI trajectory file
def read_kitti_trajectory(file_path):
    poses = []
    with open(file_path, 'r') as file:
        for line in file:
            tokens = line.strip().split()
            pose = np.array([float(token) for token in tokens]).reshape(3, 4)
            poses.append(pose)
    return np.array(poses)

# Load Point Cloud Data
pcd = o3d.io.read_point_cloud("pointcloud.pcd")
pcd = pcd.uniform_down_sample(every_k_points=50)  # Downsample more aggressively (every 50th point)

# Load KITTI trajectory file
trajectory_file = "trajectory.kitti"
poses = read_kitti_trajectory(trajectory_file)

# Set up the plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 5)  # x-axis limits in meters
ax.set_ylim(0, 5)  # y-axis limits in meters
ax.set_xlabel("X-axis (m)")
ax.set_ylabel("Y-axis (m)")
ax.set_title("Voxelized Map through PCD Frames")

# Define Voxel Size (increased voxel size)
voxel_size_m = 0.5  # 50 cm voxel grid size to reduce memory usage

# Function to update plot for each frame
def update(i):
    print(f"Processing frame {i+1}/100")  # Debugging output
    
    # Extract the current pose
    pose = poses[i]
    
    # Assuming the pose is a 3x4 matrix: [rotation | translation]
    translation = pose[:3, 3]  # Translation vector (x, y, z)
    
    # Define a region to sample around the current pose (e.g., 2.5m in each direction)
    region_size = 2.5  # meters (sample 2.5m around the current pose)
    x_min, x_max = translation[0] - region_size, translation[0] + region_size
    y_min, y_max = translation[1] - region_size, translation[1] + region_size
    
    # Filter points within the current region (around the current pose)
    points_m = np.asarray(pcd.points)  # Get points in meters
    mask = (points_m[:, 0] >= x_min) & (points_m[:, 0] <= x_max) & \
           (points_m[:, 1] >= y_min) & (points_m[:, 1] <= y_max)
    zoomed_points = points_m[mask]
    
    # Apply voxelization on the filtered points
    pcd_zoomed = o3d.geometry.PointCloud()
    pcd_zoomed.points = o3d.utility.Vector3dVector(zoomed_points)
    
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_zoomed, voxel_size_m)
    
    # Extract voxel centers (get_voxels() gives a list of Voxel objects, and we need to extract the grid index)
    voxel_centers = np.array([voxel.grid_index for voxel in voxel_grid.get_voxels()])  # Returns list of voxel indices
    
    if voxel_centers.size == 0:
        return  # No data, exit early
    
    # Convert voxel indices to real-world coordinates
    x_coords = voxel_centers[:, 0] * voxel_size_m  # Convert to real-world scale
    y_coords = voxel_centers[:, 1] * voxel_size_m  # Y as height
    
    # Define grid resolution and size
    grid_size = voxel_size_m  # Each cell corresponds to one voxel
    x_min_v = np.min(x_coords)
    x_max_v = np.max(x_coords)
    y_min_v = np.min(y_coords)
    y_max_v = np.max(y_coords)
    
    # Initialize Occupancy Grid (0 = free, 1 = occupied)
    occupancy_grid = np.zeros((int((y_max_v - y_min_v) / grid_size) + 1, int((x_max_v - x_min_v) / grid_size) + 1), dtype=np.uint8)
    
    # Convert voxel positions to grid indices
    x_indices = ((x_coords - x_min_v) / grid_size).astype(int)
    y_indices = ((y_coords - y_min_v) / grid_size).astype(int)
    
    # Mark occupied cells
    for j in range(len(x_indices)):
        occupancy_grid[y_indices[j], x_indices[j]] = 1
    
    # Clear the current plot and display the updated occupancy grid
    ax.clear()
    ax.imshow(occupancy_grid, cmap="gray", origin="lower", extent=[x_min_v, x_max_v, y_min_v, y_max_v])  # In meters
    
    # Plot the trajectory up to the current frame
    trajectory_x = poses[:i+1, 0, 3]
    trajectory_y = poses[:i+1, 1, 3]
    ax.plot(trajectory_x, trajectory_y, color='blue', linewidth=1, marker='o')
    
    # Set labels and title
    ax.set_xlabel("X-axis (m)")
    ax.set_ylabel("Y-axis (m)")
    ax.set_title(f"Voxelized Map with Trajectory (Frame {i+1}/100)")
    
    # Release memory
    gc.collect()

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=100, interval=200, repeat=False)

# Save the animation as a GIF
ani.save("trajectory_voxelization_with_trajectory.gif", writer="pillow", dpi=300)
print("Animation saved as 'trajectory_voxelization_with_trajectory.gif'")
