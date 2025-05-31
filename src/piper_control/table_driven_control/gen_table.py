import sys
import mujoco
import os
import numpy as np
import itertools
from multiprocessing import Pool, cpu_count, Process, Manager

# from queue import

sys.path.append(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
from piper_control.piper_control.ctrl_by_mujoco import CtrlByMujoco


# 步长，弧度
STRIDE = 1 / 180 * np.pi
anthropomorphic_site_name = "anthropomorphic_arm"


def quat_to_euler(quat: np.ndarray) -> np.ndarray:
    """
    将四元数转换为欧拉角
    :param quat: 四元数 [qw, qx, qy, qz]
    :return: 欧拉角 [roll, pitch, yaw]，单位为弧度
    """
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    roll = np.arctan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z)
    pitch = np.arcsin(-2.0 * (x * z - w * y))
    yaw = np.arctan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z)
    return np.array([roll, pitch, yaw])


def get_anthropomorphic_pose(ctrl: CtrlByMujoco) -> tuple[np.ndarray, np.ndarray]:
    return (
        ctrl.get_ee_pos(site_name=anthropomorphic_site_name),
        quat_to_euler(ctrl.get_ee_quat(site_name=anthropomorphic_site_name)),
    )


def get_ee_pose(ctrl: CtrlByMujoco) -> tuple[np.ndarray, np.ndarray]:
    return (
        ctrl.get_ee_pos(),
        quat_to_euler(ctrl.get_ee_quat()),
    )


def sim_a_range(joint_1, lower, upper, anthropomorphic, result_queue):
    # print(f"Processing Joint 1: {joint_1}, Anthropomorphic: {anthropomorphic}")
    ctrl = CtrlByMujoco(
        sim_steps=10,
        model_path=os.path.abspath(
            os.path.join(
                "./src/piper_description",
                "mujoco_model",
                (
                    "piper_no_gripper_anthropomorphic_description.xml"
                    if anthropomorphic
                    else "piper_no_gripper_wrist_description.xml"
                ),
            ),
        ),
    )
    ranges = [np.arange(l, u, STRIDE) for l, u in zip(lower, upper)]
    combinations = itertools.product(*ranges)
    for idx, c in enumerate(combinations):
        ctrl.reset()
        joints = {0: joint_1, 1: c[0], 2: c[1]}
        ctrl.set_joint(joints)
        ctrl.send_a_step()
        # print(joints)
        if anthropomorphic:
            ee_pos, ee_euler = get_anthropomorphic_pose(ctrl)
        else:
            ee_pos, ee_euler = get_ee_pose(ctrl)
        # print(ee_pos, ee_euler)
        result_queue.put(
            {
                "joints": ctrl.get_joint()[:3],
                "ee_pos": ee_pos,
                "ee_eular": ee_euler,
            }
        )
        print(f"\rProcessing: {idx + 1}/{np.prod([len(r) for r in ranges])}", end="")
    print()  # New line after processing
    return f"Joint 1: {joint_1}, Anthropomorphic: {anthropomorphic}, Completed!"


def collect_results(result):
    print(result)


def writer(result_queue, filename):
    with open(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), filename), "w"
    ) as f:
        f.write("Joint Angles, End Effector Position, End Effector Euler Angles\n")
        while True:
            result = result_queue.get()
            if result is None:
                break
            f.write(f"{result['joints']}, {result['ee_pos']}, {result['ee_eular']}\n")


def main():
    with Manager() as manager:
        p = Pool(cpu_count())
        result_queue = manager.Queue()
        lower = CtrlByMujoco.joint_lower_limits
        upper = CtrlByMujoco.joint_upper_limits
        for joint_1 in np.arange(lower[0], upper[0], STRIDE):
            p.apply_async(
                sim_a_range,
                args=(joint_1, lower[1:3], upper[1:3], True, result_queue),
                callback=collect_results,
                error_callback=lambda e: print(f"Error: {e}"),
            )

        p.close()
        pro = Process(target=writer, args=(result_queue, "anthropomorphic_results.txt"))
        pro.start()
        p.join()
        result_queue.put(None)  # Signal the writer to stop
        pro.join()

    with Manager() as manager:
        p = Pool(cpu_count())
        result_queue = manager.Queue()
        for joint_4 in np.arange(lower[3], upper[3], STRIDE):
            p.apply_async(
                sim_a_range,
                args=(joint_4, lower[4:6], upper[4:6], False, result_queue),
                callback=collect_results,
                error_callback=lambda e: print(f"Error: {e}"),
            )

        p.close()
        pro = Process(target=writer, args=(result_queue, "wrist_results.txt"))
        pro.start()
        p.join()
        result_queue.put(None)  # Signal the writer to stop
        pro.join()


if __name__ == "__main__":
    main()
