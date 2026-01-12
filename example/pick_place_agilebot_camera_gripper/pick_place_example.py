"""
Copyright © 2025 Agilebot Robotics Ltd. All rights reserved.
Author: Desheng.Li, September 4, 2025
Instruction:
Implement pick and place functionality following Isaac Sim official tutorial. This file is the program entry point that launches a standalone Isaac Sim application and loads a pick_place task.
reference:
 https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html

"""
# Import Isaac Sim's SimulationApp module - must be imported and APP started first
from isaacsim import SimulationApp
# Initialize simulation app, non-headless mode (display GUI)
simulation_app=SimulationApp({"headless": False})
import numpy as np
from controllers.pick_place import PickPlaceController
from isaacsim.core.api import World
from tasks.pick_place import PickPlaceTask
from isaacsim.core.prims import SingleXFormPrim

# Create a world object, set scene units to meters, physics time step to 1/200 seconds, rendering time step to 20/200 seconds
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)

# Set placement target position
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.1 / 2.0 
# Set initial cube position to None, indicating use default value
cube_initial_position = None
my_task = PickPlaceTask(name="gbt_c5a_pick_place_task", target_position=target_position,
                        cube_initial_position=cube_initial_position,
                        cube_size=np.array([0.1, 0.0515, 0.1]))
# Add task to world
my_world.add_task(my_task)
my_world.reset()
# Get task parameters
task_params=my_world.get_task("gbt_c5a_pick_place_task").get_params()
cube_name=task_params["cube_name"]['value']
robot_name=task_params["robot_name"]['value']
my_gbt_c5a=my_world.scene.get_object(robot_name)

# Initialize PickPlace controller
my_controller:PickPlaceController=PickPlaceController(
    name="pick_place_controller",
    robot_articulation=my_gbt_c5a,
    gripper=my_gbt_c5a.gripper)
task_params=my_world.get_task("gbt_c5a_pick_place_task").get_params()
# Get robot joint controller
articulation_controller=my_gbt_c5a.get_articulation_controller()


from omni.isaac.sensor import Camera 
# from isaacsim.sensors.camera import SingleViewDepthSensorAsset,SingleViewDepthSensor,Camera # Do not use this API as it will cause errors
camera_path = "/World/gbt_c5a_camera_gripper/link6/flange/camera_mount/Orbbec/camera_rgb/camera_rgb"            # Must match the camera path created in USD
camera_width, camera_height = 1280, 720  # Set camera resolution, consistent with Graspnet baseline algorithm
camera = Camera(prim_path=camera_path, resolution=(camera_width, camera_height))
camera.initialize()
camera.add_distance_to_image_plane_to_frame() # Depth map
camera.attach_annotator("rgb") # Color image

# Import video recorder module
from video_recorder import VideoRecorder

# Calculate rendering fps from world settings
rendering_dt = my_world.get_rendering_dt()
rendering_fps = 1.0 / rendering_dt

# Fixed recording fps
target_fps = 30

# Calculate how many rendering steps to skip to achieve target fps
frame_skip = max(1, int(rendering_fps / target_fps))

# Initialize video recorder with target fps
video_recorder = VideoRecorder(video_fps=target_fps)
video_recorder.start_recording()

reset_needed = False
task_completed = False
step_count = 0
frame_counter = 0

# End effector offset: distance from flange center to gripper center
END_EFFECTOR_OFFSET = np.array([0, 0, 0.22])

# Main loop, continuously run until the simulation app closes
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            my_controller.reset()
            task_completed = False
        if my_world.current_time_step_index == 0:
            my_controller.reset()

        observations = my_world.get_observations()
        # Forward controller to get actions, including joint_positions and joint_velocities
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=END_EFFECTOR_OFFSET,
        )
        
        if my_controller.is_done() and not task_completed:
            print("done picking and placing")
            task_completed = True
            video_recorder.stop_recording()
        # Apply the calculated actions to the robot's joint controller
        articulation_controller.apply_action(actions)
        
        # Send raw camera data to recording process based on frame skip
        frame_counter += 1
        if frame_counter % frame_skip == 0:
            video_recorder.record_frame(camera)

    if my_world.is_stopped():
        reset_needed = True

# Stop video recording and cleanup
video_recorder.close()

simulation_app.close()
