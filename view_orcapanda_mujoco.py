"""Open bundled Panda-based Orca MJCF in MuJoCo for inspection (arms only).

Loads ``orca_arm`` MJCF paths as shipped—no extra world geometry, lighting, or
scene files. Downstream projects should compose their own MuJoCo scenes around
these models.
"""

import argparse
import time

import mujoco
import mujoco.viewer

import orca_arm

READY_ARM_QPOS = [0.0, -0.7, 0.0, -2.1, 0.0, 1.7, 0.785]

EMBODIMENTS = {
    "orcapanda": {
        "path": orca_arm.ORCAPANDA_MJCF_PATH,
        "camera_distance": 2.35,
    },
    "bimanual_orcapanda": {
        "path": orca_arm.BIMANUAL_ORCAPANDA_MJCF_PATH,
        "camera_distance": 3.1,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--embodiment",
        choices=tuple(EMBODIMENTS),
        default="orcapanda",
        help="Which Panda-based Orca MJCF to inspect.",
    )
    parser.add_argument(
        "--pose",
        choices=("qpos0", "home", "ready"),
        default="qpos0",
        help="Pose: model.qpos0, MJCF keyframe 'home', or the readable ready pose.",
    )
    parser.add_argument(
        "--print-joints",
        action="store_true",
        help="Print joint qpos values before opening the viewer.",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Only print/validate the requested pose; do not open the viewer.",
    )
    return parser.parse_args()


def set_actuator_controls_to_qpos(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        if joint_id < 0:
            continue
        qpos_addr = model.jnt_qposadr[joint_id]
        data.ctrl[actuator_id] = data.qpos[qpos_addr]


def set_ready_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    mujoco.mj_resetData(model, data)
    for joint_id in range(model.njnt):
        name = model.joint(joint_id).name
        for arm_joint_id, value in enumerate(READY_ARM_QPOS, start=1):
            if name.endswith(f"panda_joint{arm_joint_id}"):
                qpos_addr = model.jnt_qposadr[joint_id]
                data.qpos[qpos_addr] = value
                break


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, pose: str) -> None:
    if pose == "qpos0":
        mujoco.mj_resetData(model, data)
    elif pose == "home":
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id < 0:
            raise ValueError("This MJCF does not define a 'home' keyframe.")
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    elif pose == "ready":
        set_ready_pose(model, data)
    else:
        raise ValueError(f"Unsupported pose: {pose}")

    set_actuator_controls_to_qpos(model, data)
    mujoco.mj_forward(model, data)


def print_joints(model_path: str, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    print(f"Model: {model_path}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu}")
    for joint_id in range(model.njnt):
        joint = model.joint(joint_id)
        qpos_addr = model.jnt_qposadr[joint_id]
        lo, hi = model.jnt_range[joint_id]
        print(
            f"{joint_id:02d} {joint.name}: "
            f"qpos={data.qpos[qpos_addr]:.12g}, range=[{lo:.12g}, {hi:.12g}]"
        )


def main() -> None:
    args = parse_args()
    config = EMBODIMENTS[args.embodiment]
    model_path = config["path"]
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    set_pose(model, data, args.pose)

    if args.print_joints or args.no_viewer:
        print_joints(model_path, model, data)

    if args.no_viewer:
        return

    print(f"Opening MuJoCo viewer for {args.embodiment} at pose '{args.pose}'.")
    print("Simulation is not being stepped; close the viewer window to exit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.05, 0.0, 0.55]
        viewer.cam.distance = config["camera_distance"]
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -22
        while viewer.is_running():
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
