"""
Copyright © 2025 Agilebot Robotics Ltd. All rights reserved.
Author: Desheng.Li, September 4, 2025
Instruction:

Video Recorder Module
Handles video recording and visualization for camera data in a separate process
"""
import os
import cv2
import open3d as o3d
import numpy as np
from multiprocessing import Process, Queue, Event
import time


def depth_to_pointcloud(depth, rgb, intrinsic):
    """
    Convert depth image to point cloud
    
    Args:
        depth: np.ndarray, shape=(H,W), depth image (in meters)
        rgb: np.ndarray, shape=(H,W,3), RGB image (0-255)
        intrinsic: np.ndarray, shape=(3,3), camera intrinsic matrix
    
    Returns:
        pcd: open3d.geometry.PointCloud
    """
    fx, fy = intrinsic[0, 0], intrinsic[1, 1]
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]
    H, W = depth.shape

    # Construct pixel coordinate grid
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    Z = depth.astype(np.float32)
    
    # Handle invalid depth values (zero, negative, or NaN)
    valid_mask = (Z > 0) & np.isfinite(Z)
    Z_filtered = Z.copy()
    Z_filtered[~valid_mask] = 0
    
    # Calculate 3D coordinates with proper handling of invalid values
    X = np.zeros_like(Z_filtered)
    Y = np.zeros_like(Z_filtered)
    
    # Only compute valid points to avoid division by zero or invalid operations
    valid_points = (fx != 0) and (fy != 0) and valid_mask
    X[valid_points] = (u[valid_points] - cx) * Z_filtered[valid_points] / fx
    Y[valid_points] = (v[valid_points] - cy) * Z_filtered[valid_points] / fy

    # Concatenate as points (N,3)
    points = np.stack((X, Y, Z_filtered), axis=-1).reshape(-1, 3)

    # Corresponding RGB colors
    colors = rgb.reshape(-1, 3).astype(np.float32) / 255.0

    # Remove invalid depths (e.g., 0 or NaN)
    final_mask = valid_mask.reshape(-1)
    points = points[final_mask]
    colors = colors[final_mask]

    # Construct open3d point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def create_pointcloud_visualization(depth_image, rgb_image, intrinsic_matrix):
    """
    Create pointcloud visualization by projecting 3D points to 2D image
    
    Args:
        depth_image: np.ndarray, depth image
        rgb_image: np.ndarray, RGB image
        intrinsic_matrix: np.ndarray, camera intrinsic matrix
    
    Returns:
        pointcloud_img: np.ndarray, pointcloud visualization image
    """
    try:
        point_cloud = depth_to_pointcloud(depth_image, rgb_image, intrinsic_matrix)
        points = np.asarray(point_cloud.points)
        colors = np.asarray(point_cloud.colors)
        
        # Project 3D points back to 2D image using camera intrinsics
        fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
        cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]
        
        # Filter valid points (z > 0)
        valid_mask = points[:, 2] > 0
        points_valid = points[valid_mask]
        colors_valid = colors[valid_mask]
        
        # Project to 2D
        u = (points_valid[:, 0] * fx / points_valid[:, 2] + cx).astype(np.int32)
        v = (points_valid[:, 1] * fy / points_valid[:, 2] + cy).astype(np.int32)
        
        # Create image from projected points
        height, width = rgb_image.shape[:2]
        pointcloud_img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Filter points within image bounds
        in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u = u[in_bounds]
        v = v[in_bounds]
        colors_valid = colors_valid[in_bounds]
        
        # Set pixel colors
        pointcloud_img[v, u] = (colors_valid * 255).astype(np.uint8)
        return pointcloud_img
    except Exception as e:
        print(f"Error creating pointcloud visualization: {e}")
        # Return empty image as fallback
        return np.zeros((rgb_image.shape[0], rgb_image.shape[1], 3), dtype=np.uint8)


def normalize_depth(depth_image):
    """
    Normalize depth image to 8-bit grayscale
    
    Args:
        depth_image: np.ndarray, depth image
    
    Returns:
        depth_normalized: np.ndarray, normalized depth image (0-255)
    """
    depth_valid = depth_image.copy()
    depth_valid[~np.isfinite(depth_valid)] = 0
    depth_min = depth_valid.min()
    depth_max = depth_valid.max()
    if depth_max > depth_min:
        depth_normalized = ((depth_valid - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
    else:
        depth_normalized = np.zeros_like(depth_valid, dtype=np.uint8)
    return depth_normalized


def save_videos_worker(video_frames, video_fps, save_dir):
    """
    Worker function to save videos
    
    Args:
        video_frames: list of RGB frames
        video_fps: video frame rate
        save_dir: directory to save videos
    """
    try:
        # Save RGB video
        rgb_video_path = os.path.join(save_dir, "captured_video_rgb.mp4")
        frames_bgr = [cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) for frame in video_frames]
        height, width = frames_bgr[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(rgb_video_path, fourcc, video_fps, (width, height))
        for frame in frames_bgr:
            video_writer.write(frame)
        video_writer.release()
        print(f"RGB video saved to {rgb_video_path}")
    except Exception as e:
        print(f"Error saving videos: {e}")


def recording_process_worker(data_queue, stop_event, video_fps, save_dir):
    """
    Recording process worker that runs in a separate process
    
    Args:
        data_queue: Queue to receive raw camera data from main process
        stop_event: Event to signal when to stop recording
        video_fps: Video frame rate
        save_dir: Directory to save videos
    """
    video_frames = []
    
    print("Recording process started")
    
    while not stop_event.is_set():
        try:
            # Get data from queue with timeout
            data = data_queue.get(timeout=0.1)
            
            if data is None:
                # Sentinel value to stop recording
                break
            
            # Unpack raw camera data
            rgb_image = data
            
            # Store RGB frames
            video_frames.append(rgb_image)
            
        except Exception as e:
            if stop_event.is_set():
                break
            continue
    
    # Save videos when recording stops
    if len(video_frames) > 0:
        os.makedirs(save_dir, exist_ok=True)
        save_videos_worker(video_frames, video_fps, save_dir)
        print(f"Recording process: saved {len(video_frames)} frames")
    
    print("Recording process stopped")


class VideoRecorder:
    """
    Video recorder class that runs recording logic in a separate process
    """
    
    def __init__(self, video_fps=30, save_dir=None):
        """
        Initialize video recorder
        
        Args:
            video_fps: Video frame rate (default: 30)
            save_dir: Directory to save videos (default: ./saved_images)
        """
        self.video_fps = video_fps
        
        if save_dir is None:
            current_file_dir = os.path.dirname(os.path.abspath(__file__))
            self.save_dir = os.path.join(current_file_dir, "saved_images")
        else:
            self.save_dir = save_dir
        
        # Create queue and event for inter-process communication
        self.data_queue = Queue(maxsize=200)
        self.stop_event = Event()
        
        # Recording process
        self.recording_process = None
        self.recording = False
    
    def start_recording(self):
        """Start video recording in a separate process"""
        if self.recording:
            print("Recording already started")
            return
        
        self.stop_event.clear()
        self.recording_process = Process(
            target=recording_process_worker,
            args=(self.data_queue, self.stop_event, self.video_fps, self.save_dir)
        )
        self.recording_process.start()
        self.recording = True
        print("Video recording started in separate process")
    
    def stop_recording(self):
        """Stop video recording"""
        if not self.recording:
            print("Recording not started")
            return
        
        # Set stop event first
        self.stop_event.set()
        
        # Send sentinel value to stop recording
        try:
            self.data_queue.put(None, timeout=2.0)
        except:
            pass
        
        # Wait for process to finish
        if self.recording_process is not None:
            self.recording_process.join(timeout=10)
            if self.recording_process.is_alive():
                self.recording_process.terminate()
                self.recording_process.join()
        
        self.recording = False
        print("Video recording stopped")
    
    def record_frame(self, camera):
        """
        Send raw camera data to recording process
        
        Args:
            camera: Camera object to get frame from
        """
        if not self.recording:
            return
        
        # Check queue capacity before processing
        if self.data_queue.full():
            return
        
        # Get raw camera data (only RGB)
        try:
            latest_frame = camera.get_current_frame()
        except:
            return
        
        # Check if camera data is available
        if latest_frame is None or latest_frame.get("rgb") is None:
            return
        
        # Get RGB data with fast copy
        rgb_image = latest_frame["rgb"][:, :, :3].copy()
        
        # Send raw data to recording process without blocking
        try:
            self.data_queue.put_nowait(rgb_image)
        except:
            pass
    
    def close(self):
        """Stop recording and cleanup"""
        if self.recording:
            self.stop_recording()
