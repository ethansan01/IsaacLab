


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

XARM6_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/rfa/Downloads/xarm6_with_gripper_with_spoon.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": -1,
            "joint6": 0.0,
            "drive_joint": 0.2,  # Assuming the gripper has similar joints
            
        },
        pos=(0,0,0.4),
    ),
    actuators={
        "xarm6_arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-4]"],
            effort_limit_sim=None,
            stiffness=None,
            damping=None,
        ),
        "xarm6_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["joint[5-6]"],
            effort_limit_sim=None,
            stiffness=None,
            damping=None,
        ),
        "xarm6_gripper": ImplicitActuatorCfg(
            joint_names_expr=["drive_joint"],
            effort_limit_sim=None,
            stiffness=None,
            damping=None,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
    actuator_value_resolution_debug_print = True,

    

)

XARM6_CONFIG_HIGH = XARM6_CONFIG.copy()
XARM6_CONFIG_HIGH.spawn.rigid_props.disable_gravity = True
XARM6_CONFIG_HIGH.spawn.rigid_props.kinematic_enabled = True