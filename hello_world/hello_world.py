# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction
import carb
import numpy as np
from omni.isaac.core.controllers import BaseController
# This extension includes several generic controllers that could be used with multiple robots
from omni.isaac.motion_generation import WheelBasePoseController
# Robot specific controller
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html

class CoolController(BaseController):
    # This class specifies control movements, encapsulating the control logic from the scene/physics stuff
    # controls should return an ArticulationAction, which can be input into the robot's apply_action function

    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):

        world = self.get_world()
        world.scene.add_default_ground_plane()
        # Use the find_nucleus_server instead of changing it every time
        # you configure a new server with /Isaac folder in it
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        world.scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
            )
        )
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        return

    async def setup_post_load(self):
        # Called after first reset (or right after initial load of extension). Can do physics stuff here
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("fancy_robot")

        self.call_count = 0
        # physics is performed separately, so must use a callback
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)

        # Package controller, provided by omniverse. Does motion planning automatically 
        self._package_controller = WheelBasePoseController(name="package_controller",
                                                            open_loop_wheel_controller=
                                                            DifferentialController(name="simple_control",
                                                                                   wheel_radius=0.03, wheel_base=0.1125),
                                                            is_holonomic=False)

        # My custom controller
        self._my_controller = CoolController()
        return

    def send_robot_actions(self, step_size):
        # callback for each physics step 

        position, orientation = self._jetbot.get_world_pose()

        # forward has different parameters for velocities, but they're kinda scuffed
        self._jetbot.apply_action(self._package_controller.forward(start_position=position,
                                                            start_orientation=orientation,
                                                            goal_position=np.array([1.3, .7])))

        # if self.call_count < (5/step_size):
        #     # Every articulation controller has apply_action method
        #     # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
        #     # as optional args. It accepts numpy arrays of floats OR lists of floats and None
        #     # None means that nothing is applied to this dof index in this step
        #     # ALTERNATIVELY, same method is called from self._jetbot.apply_action(...)
        #     self._jetbot.apply_wheel_actions(self._my_controller.forward([.5, np.pi / 3]))
        #     self.call_count += 1
        # else:
            
        #     self._jetbot.apply_wheel_actions(self._my_controller.forward([0,0]))
        #     self._world.remove_physics_callback("sending_actions")
        return

    def print_cube_info(self, step_size):
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # will be shown on terminal
        print("Cube position is : " + str(position))
        print("Cube's orientation is : " + str(orientation))
        print("Cube's linear velocity is : " + str(linear_velocity))

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
