import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# 导入补全
if not os.path.exists('scene.xml'):
    print("请先运行 xml_gen.py！")
    exit()

model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)


class DemoState:
    mode = 0  # 0: 追踪, 1: 质检, 2: 庆典, 3: 维护
    start_time = 0


state = DemoState()


def key_callback(keycode):
    char = chr(keycode)
    if char in ['0', '1', '2', '3']:
        state.mode = int(char)
        state.start_time = time.time()
        print(f"切换模式: {state.mode}")


with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    print("\n--- 工业演示就绪 ---")
    print("[1] 激光质检 [2] 全线欢庆 [3] 紧急维护 [0] 恢复高速")

    # 预设姿态
    z1_home = np.array([0, 0.7, -0.4, 0, 0, 0])
    go2_stand = np.array([0, 0.8, -1.5] * 4)

    while viewer.is_running():
        step_start = time.time()
        now = time.time()

        # 1. 获取传送带位置
        dog_x = data.qpos[model.joint('belt_joint').qposadr[0]]
        if dog_x > 2.0 and state.mode != 2: data.qpos[model.joint('belt_joint').qposadr[0]] = -2.0

        # 2. 模式逻辑分支
        # ---------------------------------------------------------
        if state.mode == 1:  # 组合 A: 质检
            data.ctrl[36] = 0.4  # 慢速
            for i in range(4):
                base_x = model.body(f"{['L1', 'L2', 'R1', 'R2'][i]}_base").pos[0]
                dx = dog_x - base_x
                ctrl = z1_home.copy()
                if abs(dx) < 0.35:
                    ctrl = np.array([0, 1.5, -2.3, 1.2, 0, np.sin(now * 60) * 0.5])  # 深度下压震动
                else:
                    ctrl[0] = np.clip(dx * 2.5, -1.0, 1.0)
                data.ctrl[i * 6:(i + 1) * 6] = ctrl

        # ---------------------------------------------------------
        elif state.mode == 2:  # 组合 B: 庆典
            data.ctrl[36] = 0  # 停止
            data.ctrl[24:36] = np.array([0, 1.4, -0.8] * 4)  # 狗坐下
            for i in range(4):
                wave = np.sin(now * 6 - i * 0.7)
                ctrl = z1_home.copy()
                ctrl[1] -= wave * 0.6
                ctrl[5] = now * 25
                data.ctrl[i * 6:(i + 1) * 6] = ctrl

        # ---------------------------------------------------------
        elif state.mode == 3:  # 组合 C: 维护
            data.ctrl[36] = -1.2  # 反转
            for i in range(4):
                side = 1.4 if i < 2 else -1.4
                data.ctrl[i * 6:(i + 1) * 6] = np.array([side, 0.2, -0.2, 0, 0, 0])

        # ---------------------------------------------------------
        else:  # 默认模式 0
            data.ctrl[36] = 1.2  # 快
            for i in range(4):
                base_x = model.body(f"{['L1', 'L2', 'R1', 'R2'][i]}_base").pos[0]
                dx = dog_x - base_x
                ctrl = z1_home.copy()
                ctrl[0] = np.clip(dx * 3.0, -1.2, 1.2)
                if abs(dx) < 0.4: ctrl[1] += 0.5
                data.ctrl[i * 6:(i + 1) * 6] = ctrl

            # 高速步态
            walk = 0.5 * np.sin(now * 25)
            for j in range(4):
                data.ctrl[24 + j * 3 + 1] = go2_stand[j * 3 + 1] + (walk if j % 2 == 0 else -walk)
                data.ctrl[24 + j * 3 + 2] = go2_stand[j * 3 + 2] + (-walk if j % 2 == 0 else walk)

        mujoco.mj_step(model, data)
        viewer.sync()

        # 物理步进频率控制
        elapsed = time.time() - step_start
        if elapsed < 0.002: time.sleep(0.002 - elapsed)