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

    # particle = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/particle", spawn=sim_utils.UsdFileCfg(
    #     usd_path="/home/rfa/Downloads/particle.usd",),
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=(0.5, 0.0, 1.0), rot=(1, 0.0, 0.0, 0.0),),
    # )

    # table
    table = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/table",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/rfa/Downloads/table1.usd",
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
                    usd_path="/home/rfa/Downloads/bowl.usd",
                    scale=(1.0, 1.0, 1.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.8, 0.15, 0.6),
                    rot=(1, 0.0, 0.0, 0.0),
                ),
            ),
            "bowl_2": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/bowl_2",
                spawn=sim_utils.UsdFileCfg(
                    usd_path="/home/rfa/Downloads/bowl.usd",
                    scale=(1.0, 1.0, 1.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.8, -0.15, 0.6),
                    rot=(1, 0.0, 0.0, 0.0),
                ),
            ),
            "bowl_3": RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/bowl_3",
                spawn=sim_utils.UsdFileCfg(
                    usd_path="/home/rfa/Downloads/bowl.usd",
                    scale=(1.0, 1.0, 1.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.95, 0.0, 0.6),
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

    xArm6 = XARM6_CONFIG.replace(prim_path="{ENV_REGEX_NS}/xArm6")
    # xArm6= XARM6_CONFIG_HIGH.replace(prim_path="{ENV_REGEX_NS}/xArm6")


# def add_particle_system(stage, prim_path="{ENV_REGEX_NS}/particleSystem", sample_volume=True, contact_offset=1.0):
#     default_prim_path = stage.GetDefaultPrim().GetPath()
#     particle_system_path = default_prim_path.AppendChild("particleSystem")
#     particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
#     particle_system.CreateSimulationOwnerRel().SetTargets([stage.GetPrimAtPath("/World/physicsScene").GetPath()])
#     # The simulation determines the other offsets from the particle contact offset
#     particle_system.CreateParticleContactOffsetAttr().Set(1.0)
#     # Limit particle velocity for better collision detection
#     particle_system.CreateMaxVelocityAttr().Set(250.0)

#     # create particle material and assign it to the system:
#     particle_material_path = default_prim_path.AppendChild("particleMaterial")
#     particleUtils.add_pbd_particle_material(stage, particle_material_path)
#     physicsUtils.add_physics_material_to_prim(
#         stage, stage.GetPrimAtPath(particle_system_path), particle_material_path
#     )
#     return particle_system
# def add_particles_to_scene(stage, parent_prim_path="/World/particles", sample_volume=True, contact_offset=1.0):
#     # Create a PhysX particle system
#     particle_system_path = Sdf.Path(parent_prim_path + "/particleSystem")
#     particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
#     particle_system.CreateSimulationOwnerRel().SetTargets(["/physicsScene"])
#     particle_system.CreateParticleContactOffsetAttr().Set(contact_offset)
#     particle_system.CreateMaxVelocityAttr().Set(250.0)

#     # Define a particle target set
#     particle_points_path = Sdf.Path(parent_prim_path + "/sampledParticles")
#     points = UsdGeom.Points.Define(stage, particle_points_path)
#     particle_set_api = PhysxSchema.PhysxParticleSetAPI.Apply(points.GetPrim())
#     PhysxSchema.PhysxParticleAPI(particle_set_api).CreateParticleSystemRel().SetTargets([particle_system_path])

#     # Create a source mesh (cube) to sample
#     # cube_mesh = sim_utils.MeshCfg()
#     cube_mesh_path = omni.usd.get_stage_next_free_path(stage, parent_prim_path + "/Cube", True)
#     omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", prim_path=cube_mesh_path,select_new_prim=False)
#     cube_mesh = UsdGeom.Mesh.Get(stage, Sdf.Path(cube_mesh_path))
#     translate_op = cube_mesh.GetOrderedXformOps()[0]  # usually translate is first
#     translate_op.Set(Gf.Vec3f(0.8, 0.0, 1.0))
#     scale_op = None
#     for op in cube_mesh.GetOrderedXformOps():
#         if op.GetOpName() == "xformOp:scale":
#             scale_op = op
#             break

#     if scale_op:
#         scale_op.Set(Gf.Vec3f(0.2, 0.2, 0.2))
#     else:
#         cube_mesh.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
#     # cube_mesh.AddTranslateOp().Set(Gf.Vec3f(0.8, 0.0, 1.0))  # drop it near the bowl
#     # cube_mesh.AddScaleOp().Set(Gf.Vec3f(0.2))

#     # Compute sampling distance
#     fluid_rest_offset = 0.99 * 0.6 * contact_offset
#     particle_sampler_distance = 2.0 * fluid_rest_offset

#     # Apply sampler API
#     sampling_api = PhysxSchema.PhysxParticleSamplingAPI.Apply(cube_mesh.GetPrim())
#     sampling_api.CreateParticlesRel().AddTarget(particle_points_path)
#     sampling_api.CreateSamplingDistanceAttr().Set(particle_sampler_distance)
#     sampling_api.CreateMaxSamplesAttr().Set(5e5)
#     sampling_api.CreateVolumeAttr().Set(sample_volume)

#     print(f"[INFO] Particle system added at {parent_prim_path}")
from pxr import UsdGeom, Sdf, Gf, PhysxSchema
import omni.usd
import omni.physx

def add_particles_to_scene(stage, parent_prim_path="/World/particles", sample_volume=True, contact_offset=0.02):
    """
    One-time particle initialization for Isaac Lab.
    Creates a particle system, a particle set, and samples particles from a temporary cube mesh.
    """

    # Ensure parent prim exists
    parent_prim = stage.GetPrimAtPath(parent_prim_path)
    if not parent_prim:
        stage.DefinePrim(parent_prim_path)

    # --- Create particle system ---
    particle_system_path = Sdf.Path(parent_prim_path + "/particleSystem")
    particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
    particle_system.CreateSimulationOwnerRel().SetTargets(["/physicsScene"])
    particle_system.CreateParticleContactOffsetAttr().Set(contact_offset)
    particle_system.CreateMaxVelocityAttr().Set(250.0)

    # --- Create particle set ---
    particle_points_path = Sdf.Path(parent_prim_path + "/sampledParticles")
    points = UsdGeom.Points.Define(stage, particle_points_path)
    particle_set_api = PhysxSchema.PhysxParticleSetAPI.Apply(points.GetPrim())
    PhysxSchema.PhysxParticleAPI(particle_set_api).CreateParticleSystemRel().SetTargets([particle_system_path])

    # --- Create temporary cube mesh for sampling ---
    cube_path = omni.usd.get_stage_next_free_path(stage, parent_prim_path + "/Cube", True)
    omni.kit.commands.execute(
        "CreateMeshPrimWithDefaultXform",
        prim_type="Cube",
        prim_path=cube_path,
        select_new_prim=False
    )
    cube_mesh = UsdGeom.Mesh.Get(stage, Sdf.Path(cube_path))
    xform = UsdGeom.XformCommonAPI(cube_mesh)
    xform.SetTranslate((0.8, 0.0, 1.0))   # Position above the target bowl
    xform.SetScale((0.2, 0.2, 0.2))       # Small cube for sampling

    # --- Sample particles once ---
    # reference the particle set in the sampling api
    fluid_rest_offset = 0.99 * 0.6 * contact_offset
    particle_sampler_distance = 2.0 * fluid_rest_offset
    sampling_api = PhysxSchema.PhysxParticleSamplingAPI.Apply(cube_mesh.GetPrim())
    sampling_api.CreateParticlesRel().AddTarget(particle_points_path)
    sampling_api.CreateSamplingDistanceAttr().Set(particle_sampler_distance)
    sampling_api.CreateMaxSamplesAttr().Set(5e5)
    sampling_api.CreateVolumeAttr().Set(sample_volume)

    # Remove cube after sampling
    stage.RemovePrim(cube_mesh.GetPath())
    print(f"[INFO] One-time particle set created at {particle_points_path.pathString}")

    return particle_system, points


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    alpha = 0.05

    # Use this method for better readability
    # bowl: RigidObject = scene["bowl"]
    bowl_collection: RigidObjectCollection = scene["bowl_collection"]
    xarm6: Articulation = scene["xArm6"]
    table: RigidObject = scene["table"]
    assert bowl_collection.data.num_objects == 3, "Expected 3 bowls in the collection."


    while simulation_app.is_running():
        # reset
        if count % 500000 == 0:
            # reset counters
            count = 0

            root_bowl_collection_state = bowl_collection.data.default_object_state.clone()
            root_bowl_collection_state[..., :3] += scene.env_origins.unsqueeze(1)

            root_table_state = table.data.default_root_state.clone()
            root_table_state[:, :3] += scene.env_origins
            root_xarm6_state = xarm6.data.default_root_state.clone()
            root_xarm6_state[:, :3] += scene.env_origins
            # root_bowl_state = bowl.data.default_root_state.clone()
            # root_bowl_state[:, :3] += scene.env_origins


            
            bowl_collection.write_object_link_pose_to_sim(root_bowl_collection_state[..., :7])
            bowl_collection.write_object_com_velocity_to_sim(root_bowl_collection_state[..., 7:])
            # bowl.write_root_pose_to_sim(root_bowl_state[:, :7])
            # bowl.write_root_velocity_to_sim(root_bowl_state[:, 7:])
            table.write_root_pose_to_sim(root_table_state[:, :7])
            table.write_root_velocity_to_sim(root_table_state[:, 7:])
            xarm6.write_root_pose_to_sim(root_xarm6_state[:, :7])
            xarm6.write_root_velocity_to_sim(root_xarm6_state[:, 7:])

            joint_pos, joint_vel = (
                xarm6.data.default_joint_pos.clone(),
                xarm6.data.default_joint_vel.clone(),
            )

            xarm6.write_joint_state_to_sim(joint_pos, joint_vel)
            # bowl.write_data_to_sim()
            table.write_data_to_sim()
            print(xarm6.joint_names)
            bowl_collection.write_data_to_sim()
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting xArm state...")
        # current_pose = xarm6.data.default_joint_pos
        # print(f"[INFO]: Current joint positions: {current_pose}")
        # target_pose = torch.tensor([2, 0.0, 0.0, 0.0, -1.2, 0.0, 0.1, 0, 0, 0, 0, 0], device='cuda:0')  # Example target pose for the gripper
        # new_pose = (1- alpha) * current_pose + alpha * target_pose


        wave_action = xarm6.data.default_joint_pos
        wave_action[:, 4] = 0.25 * np.sin(2 * np.pi * 0.5 * sim_time) -1.2
        xarm6.set_joint_position_target(wave_action)
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
    sim_cfg = sim_utils.SimulationCfg(device='cpu', use_fabric=False)
    sim = sim_utils.SimulationContext(sim_cfg)
    physics_interface = omni.physx.acquire_physx_interface()
    physics_interface.overwrite_gpu_setting(1)
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = ScoopingSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    stage = omni.usd.get_context().get_stage()
    add_particles_to_scene(stage, parent_prim_path="/World/particles", sample_volume=True, contact_offset=0.02)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
