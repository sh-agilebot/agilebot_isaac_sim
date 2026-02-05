"""
Copyright © 2016 Agilebot Robotics Ltd. All rights reserved.

Pick and Place Example for GBT C5A Robot with Wrist Camera and Gripper

This is the main entry point that launches a standalone Isaac Sim application
and executes a pick and place task using the GBT C5A robot arm.

Reference:
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html
"""

# Import SimulationApp first - this must be imported and initialized before other Isaac Sim modules
from isaacsim import SimulationApp

# Initialize simulation app in non-headless mode (with GUI)
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os

from isaacsim.core.api import World
from controllers.pick_place import PickPlaceController
from tasks.pick_place import PickPlaceTask
from omni.isaac.sensor import Camera
from video_recorder import VideoRecorder

# Create world with meters as unit and physics timestep of 1/120 seconds
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)

# Set robot USD file path
usd_path = os.path.join(os.path.dirname(__file__), "usd/gbt-c5a_camera_gripper/gbt-c5a_camera_gripper.usd")

# Configure target placement position
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.1 / 2.0

# Set cube initial position to None for default placement
cube_initial_position = None

# Create pick and place task
my_task = PickPlaceTask(
    name="gbt_c5a_pick_place_task",
    target_position=target_position,
    cube_initial_position=cube_initial_position,
    cube_size=np.array([0.1, 0.0515, 0.1]),
    usd_path=usd_path
)

# Add task to world and reset
my_world.add_task(my_task)
my_world.reset()

# Get task parameters
task_params = my_world.get_task("gbt_c5a_pick_place_task").get_params()
cube_name = task_params["cube_name"]['value']
robot_name = task_params["robot_name"]['value']
my_gbt_c5a = my_world.scene.get_object(robot_name)

# Initialize pick and place controller
my_controller: PickPlaceController = PickPlaceController(
    name="pick_place_controller",
    robot_articulation=my_gbt_c5a,
    gripper=my_gbt_c5a.gripper
)

# Get robot articulation controller
articulation_controller = my_gbt_c5a.get_articulation_controller()

# Configure camera
# Camera prim path must match the USD file structure
camera_path = "/World/gbt_c5a_camera_gripper/link6/flange/camera_mount/Orbbec/camera_rgb/camera_rgb"
camera_width, camera_height = 1280, 720
camera = Camera(prim_path=camera_path, resolution=(camera_width, camera_height))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()  # Enable depth annotation
camera.attach_annotator("rgb")  # Enable RGB annotation

# Configure video recorder
rendering_dt = my_world.get_rendering_dt()
rendering_fps = 1.0 / rendering_dt
target_fps = 30
frame_skip = max(1, int(rendering_fps / target_fps))

video_recorder = VideoRecorder(video_fps=target_fps)
video_recorder.start_recording()

# State variables
reset_needed = False
task_completed = False
step_count = 0
frame_counter = 0

# End effector offset: distance from flange center to gripper center
END_EFFECTOR_OFFSET = np.array([0, 0, 0.22])

# Main simulation loop
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_playing():
        # Reset world if needed
        if reset_needed:
            my_world.reset()
            reset_needed = False
            my_controller.reset()
            task_completed = False

        # Reset controller on first step
        if my_world.current_time_step_index == 0:
            my_controller.reset()

        # Get observations and compute actions
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=END_EFFECTOR_OFFSET,
        )

        # Check for task completion
        if my_controller.is_done() and not task_completed:
            print("Task completed: pick and place finished")
            task_completed = True
            video_recorder.stop_recording()

        # Apply actions to robot
        articulation_controller.apply_action(actions)

        # Record camera frames at target FPS
        frame_counter += 1
        if frame_counter % frame_skip == 0:
            video_recorder.record_frame(camera)

    if my_world.is_stopped():
        reset_needed = True

# Cleanup
video_recorder.close()
simulation_app.close()
