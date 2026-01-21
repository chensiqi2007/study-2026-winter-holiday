import mujoco
import mujoco.viewer
import numpy as np
import time

# 加载模型
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)


# 获取必要 ID
def get_id(name, obj_type=mujoco.mjtObj.mjOBJ_GEOM):
    return mujoco.mj_name2id(model, obj_type, name)


red_id = get_id('led_red')
blue_id = get_id('led_blue')
belt_qpos_idx = model.joint('belt_joint').qposadr[0]
belt_act_id = model.actuator('belt_m').id

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("仿真已启动：无人机正在进行循环行程...")

    while viewer.is_running():
        step_start = time.time()
        now = time.time()

        # 1. 无人机行程循环逻辑
        # 当位置超过传送带右侧(2.1m)时，瞬间重置回左侧(-2.1m)
        if data.qpos[belt_qpos_idx] > 2.1:
            data.qpos[belt_qpos_idx] = -2.1
            data.qvel[model.joint('belt_joint').dofadr[0]] = 0  # 重置速度

        # 2. 传送带动力：设定恒定移动速度
        data.ctrl[belt_act_id] = 1.0

        # 3. 信号灯红蓝爆闪 (修正 model.geom_rgba 访问)
        if red_id != -1:
            is_flash = np.sin(now * 30) > 0
            model.geom_rgba[red_id] = [1, 0, 0, 1] if is_flash else [0.2, 0, 0, 1]
            model.geom_rgba[blue_id] = [0, 0.4, 1, 1] if not is_flash else [0, 0.1, 0.2, 1]

        # 4. 机械臂基础追逐 (底座关节跟随无人机 X 坐标)
        base_x = model.body("L1_base").pos[0]
        dx = data.qpos[belt_qpos_idx] - base_x
        data.ctrl[0] = np.clip(dx * 2.0, -1.2, 1.2)

        mujoco.mj_step(model, data)
        viewer.sync()

        # 维持仿真步长 (500Hz)
        time.sleep(max(0, 0.002 - (time.time() - step_start)))