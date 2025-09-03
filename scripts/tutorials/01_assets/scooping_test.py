import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import (
    Articulation,
    ArticulationCfg,
    AssetBaseCfg,
    RigidObject,
    RigidObjectCfg,
    RigidObjectCollection,
    RigidObjectCollectionCfg,
    DeformableObject,
    DeformableObjectCfg,
    DeformableObjectData,
)
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab_assets.robots.xarm6 import XARM6_CONFIG, XARM6_CONFIG_HIGH
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import open_stage, get_current_stage
import isaaclab.sim.spawners.meshes as mesh_spawner
from omni.physx.scripts import physicsUtils, particleUtils
import omni.usd
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.physx.bindings._physx as physx_settings_bindings
import omni.physxdemos as demo

class ScoopingSceneCfg(InteractiveSceneCfg):


    """Designs the scene."""
    







    
    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # particle_system = sim_utils.MeshCfg()

    particle = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/particle", spawn=sim_utils.UsdFileCfg(
        usd_path="/home/yiheng/Downloads/fluid_new.usd",),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.6, 0.1, 0.5), rot=(1, 0.0, 0.0, 0.0),),
    )

    # table
    table = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/table",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/yiheng/Downloads/table1.usd",
            scale=(0.5, 0.5, 0.5),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
            
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.85, 0.15, 0.5),
            rot=(0.70711, 0.0, 0.0, -0.70711),
        ),
    )
    bowl_collection: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects={
            "bowl_1": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/bowl_1",
                spawn=sim_utils.UsdFileCfg(
                    usd_path="/home/yiheng/Downloads/bowl.usd",
                    scale=(1.0, 1.0, 1.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.8, 0.15, 0.55),
                    rot=(1, 0.0, 0.0, 0.0),
                ),
            ),
            "bowl_2": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/bowl_2",
                spawn=sim_utils.UsdFileCfg(
                    usd_path="/home/yiheng/Downloads/bowl.usd",
                    scale=(1.0, 1.0, 1.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.8, -0.15, 0.55),
                    rot=(1, 0.0, 0.0, 0.0),
                ),
            ),
            "bowl_3": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/bowl_3",
                spawn=sim_utils.UsdFileCfg(
                    usd_path="/home/yiheng/Downloads/bowl.usd",
                    scale=(1.0, 1.0, 1.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.95, 0.0, 0.55),
                    rot=(1, 0.0, 0.0, 0.0),
                ),
            ),
        }
    )

    # spoon_utensil_mount = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/spoon_utensil_mount",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path="/home/rfa/Downloads/spoon_utensil_mount.usd",
    #         scale=(1.0, 1.0, 1.0),


    # robot

    # xArm6 = XARM6_CONFIG.replace(prim_path="{ENV_REGEX_NS}/xArm6")
def init_particle_step():
    stage = get_current_stage()
    
    # ✅ Get the /World prim if it exists
    world_prim = stage.GetPrimAtPath("/World")
    
    if not world_prim.IsValid():
        # Only define if it doesn't already exist
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World")).GetPrim()
        stage.SetDefaultPrim(world_prim)
    
    default_prim_path = world_prim.GetPath()
    return stage, default_prim_path

def get_particles_position()->tuple[Gf.Vec3f, Gf.Vec3f]:
    # Gets particles' positions and velocities 
    stage, default_prim_path = init_particle_step()
    print(stage)
    prim = stage.GetPrimAtPath("/World/envs/env_0/particle/ParticleSet")
    # particles = UsdGeom.Points(stage, "/World/env0/particle")
    print(prim)
    particles = UsdGeom.Points(prim)
    print(particles)
    particles_pos = particles.GetPointsAttr().Get()
    particles_vel = particles.GetVelocitiesAttr().Get()
    print(f"Particles: {particles_pos}")
    return particles_pos, particles_vel

def set_particles_position(particles_pos:Gf.Vec3f, particles_vel:Gf.Vec3f):
    # Sets the particles' position and velocities to the given arrays
    stage, default_prim_path = init_particle_step()
    prim = stage.GetPrimAtPath("/World/envs/env_0/particle/ParticleSet")
    print("Prim type:", prim.GetTypeName())   # should be "Points"
    print("IsA Points:", prim.IsA(UsdGeom.Points))
    particles = UsdGeom.Points(prim)
    particles.GetPointsAttr().Set(particles_pos)
    particles.GetVelocitiesAttr().Set(particles_vel)

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    print("{ENV_REGEX_NS}")
    # Use this method for better readability
    # bowl: RigidObject = scene["bowl"]
    bowl_collection: RigidObjectCollection = scene["bowl_collection"]
    # xarm6: Articulation = scene["xArm6"]
    table: RigidObject = scene["table"]
    assert bowl_collection.data.num_objects == 3, "Expected 3 bowls in the collection."
    particle_pos, particle_vel = get_particles_position()
    while simulation_app.is_running():
    
        # reset
        if count % 500 == 0:

            # reset counters
            count = 0
            set_particles_position(particles_pos=particle_pos, particles_vel=particle_vel)
            root_bowl_collection_state = bowl_collection.data.default_object_state.clone()
            root_bowl_collection_state[..., :3] += scene.env_origins.unsqueeze(1)

            root_table_state = table.data.default_root_state.clone()
            root_table_state[:, :3] += scene.env_origins
            # root_xarm6_state = xarm6.data.default_root_state.clone()
            # root_xarm6_state[:, :3] += scene.env_origins
            # root_bowl_state = bowl.data.default_root_state.clone()
            # root_bowl_state[:, :3] += scene.env_origins


            
            bowl_collection.write_object_link_pose_to_sim(root_bowl_collection_state[..., :7])
            bowl_collection.write_object_com_velocity_to_sim(root_bowl_collection_state[..., 7:])
            # bowl.write_root_pose_to_sim(root_bowl_state[:, :7])
            # bowl.write_root_velocity_to_sim(root_bowl_state[:, 7:])
            table.write_root_pose_to_sim(root_table_state[:, :7])
            table.write_root_velocity_to_sim(root_table_state[:, 7:])

            # set_particles_position(particles_pos = particle_pos, particles_vel = particle_vel)
            # xarm6.write_root_pose_to_sim(root_xarm6_state[:, :7])
            # xarm6.write_root_velocity_to_sim(root_xarm6_state[:, 7:])

            # joint_pos, joint_vel = (
                # xarm6.data.default_joint_pos.clone(),
                # xarm6.data.default_joint_vel.clone(),
            # )

            # xarm6.write_joint_state_to_sim(joint_pos, joint_vel)
            # # bowl.write_data_to_sim()
            table.write_data_to_sim()
            # print(xarm6.joint_names)
            bowl_collection.write_data_to_sim()
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting xArm state...")
        # current_pose = xarm6.data.default_joint_pos
        # print(f"[INFO]: Current joint positions: {current_pose}")
        # target_pose = torch.tensor([2, 0.0, 0.0, 0.0, -1.2, 0.0, 0.1, 0, 0, 0, 0, 0], device='cuda:0')  # Example target pose for the gripper
        # new_pose = (1- alpha) * current_pose + alpha * target_pose


        # wave_action = xarm6.data.default_joint_pos
        # wave_action[:, 4] = 0.25 * np.sin(2 * np.pi * 0.5 * sim_time) -1.2
        # xarm6.set_joint_position_target(wave_action)
        # # scene["xArm6"].set_joint_position_target(wave_action)
        # print(f"[INFO]: Setting joint positions to: {new_pose}")
        # xarm6.set_joint_position_target(new_pose)
        # xarm6.write_data_to_sim()
        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    # sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim_cfg = sim_utils.SimulationCfg(device="cpu"  , use_fabric=False)
    physx_interface = omni.physx.get_physx_interface()
    # Override CPU setting to use GPU
    physx_interface.overwrite_gpu_setting(1)

    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = ScoopingSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
