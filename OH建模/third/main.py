import mujoco
import mujoco.viewer
import time
import threading
import numpy as np
import os

# 全局变量
blackboard_content = "WELCOME"
light_brightness = 1.2  # 默认亮度


def get_fixed_xml(path):
    if not os.path.exists(path):
        return None
    with open(path, 'r', encoding='utf-8') as f:
        xml = f.read()

    # 补全椅腿逻辑：根据 XML 中座垫 geom pos="0 0 0.52"
    # 我们生成 4 根金属细腿，中心高度设为 0.26，正好支撑到地面
    chair_leg_geom = """
                <geom type="cylinder" size="0.01 0.26" pos="0.18 0.18 0.26" material="mat_metal"/>
                <geom type="cylinder" size="0.01 0.26" pos="-0.18 0.18 0.26" material="mat_metal"/>
                <geom type="cylinder" size="0.01 0.26" pos="0.18 -0.18 0.26" material="mat_metal"/>
                <geom type="cylinder" size="0.01 0.26" pos="-0.18 -0.18 0.26" material="mat_metal"/>
            </body>"""
    # 针对 XML 中定义的多组座椅 body 进行替换
    fixed_xml = xml.replace('</body>', chair_leg_geom, 30)
    return fixed_xml


def terminal_input_thread():
    global blackboard_content, light_brightness
    brightness_levels = [0.1, 0.6, 1.2, 2.5, 5.0]

    print("\n" + "=" * 40)
    print(" [全功能交互版已启动] ")
    print(" 1. 亮度调节：输入数字 1-5")
    print(" 2. 黑板同步：输入任意文字")
    print(" 3. 退出：输入 exit")
    print("=" * 40 + "\n")

    while True:
        user_in = input("指令 >> ").strip()
        if user_in.lower() == 'exit':
            os._exit(0)
        elif user_in in ['1', '2', '3', '4', '5']:
            light_brightness = brightness_levels[int(user_in) - 1]
            print(f" >>> 亮度已调至第 {user_in} 档 ({light_brightness})")
        else:
            blackboard_content = user_in.upper()
            print(f" >>> 黑板已同步: {user_in}")


def main():
    global blackboard_content, light_brightness

    xml_path = "classroom_final_v5_clean.xml"
    xml_content = get_fixed_xml(xml_path)
    if not xml_content: return

    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)

    threading.Thread(target=terminal_input_thread, daemon=True).start()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 12.0
        viewer.cam.lookat = [0, 3, 1.5]

        while viewer.is_running():
            step_start = time.time()

            # 实时更新灯光亮度
            # XML 中有两个光源，索引为 0 和 1
            for i in range(model.nlight):
                model.light_diffuse[i] = [light_brightness] * 3

            mujoco.mj_step(model, data)

            with viewer.lock():
                if viewer.user_scn.maxgeom < 1:
                    viewer.user_scn.maxgeom = 1

                g = viewer.user_scn.geoms[0]
                g.type = mujoco.mjtGeom.mjGEOM_SPHERE
                g.size[:] = [0.001, 0.001, 0.001]

                # 设置浮空位置：黑板在 Y=7.89，设在 Y=7.65 增强浮空感
                g.pos[:] = [0, 7.65, 1.8]
                g.rgba[:] = [0, 0, 0, 0]

                # 修复 mat 形状：直接赋值 3x3 单位阵
                g.mat[:] = np.eye(3, dtype=np.float32)

                # 文字同步
                g.label = f"{blackboard_content}"
                viewer.user_scn.ngeom = 1

            viewer.sync()

            # 动态控制步进频率
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
