import os
import xml.etree.ElementTree as ET
import mujoco
import mujoco.viewer
import numpy as np
import time
import copy


def build_factory_xml():
    # --- 1. 严格保留原有场景逻辑 ---
    def get_xml_parts(filename):
        if not os.path.exists(filename): return None
        tree = ET.parse(filename)
        root = tree.getroot()
        return {'asset': root.find('asset'), 'default': root.find('default'),
                'actuator': root.find('actuator'), 'worldbody': root.find('worldbody')}

    def strip_free_joints(element):
        for child in list(element):
            if child.tag == 'freejoint' or (child.tag == 'joint' and child.get('type') == 'free'):
                element.remove(child)
            else:
                strip_free_joints(child)

    def rename_instance(element, prefix):
        if 'name' in element.attrib: element.attrib['name'] = f"{prefix}_{element.attrib['name']}"
        if 'joint' in element.attrib: element.attrib['joint'] = f"{prefix}_{element.attrib['joint']}"
        if 'site' in element.attrib: element.attrib['site'] = f"{prefix}_{element.attrib['site']}"
        for child in element: rename_instance(child, prefix)

    def rename_class_usage(element, old_name, new_name):
        if 'class' in element.attrib and element.attrib['class'] == old_name:
            element.attrib['class'] = new_name
        for child in element: rename_class_usage(child, old_name, new_name)

    z1_parts = get_xml_parts('z1.xml')
    cf2_parts = get_xml_parts('cf2.xml')
    
    # 解决 cf2 类名冲突
    if cf2_parts:
        for part in cf2_parts.values():
            if part is not None:
                # 处理 list 或者是 Element
                if isinstance(part, list):
                     for item in part: rename_class_usage(item, 'visual', 'cf2_visual')
                     for item in part: rename_class_usage(item, 'collision', 'cf2_collision')
                else:
                     rename_class_usage(part, 'visual', 'cf2_visual')
                     rename_class_usage(part, 'collision', 'cf2_collision')

    new_root = ET.Element('mujoco', {'model': 'z1_factory_final'})
    ET.SubElement(new_root, 'compiler', {'angle': 'radian', 'meshdir': 'assets', 'autolimits': 'true'})
    ET.SubElement(new_root, 'option', {'integrator': 'implicitfast', 'timestep': '0.002'})
    
    # 视觉增强
    visual = ET.SubElement(new_root, 'visual')
    ET.SubElement(visual, 'headlight', {'diffuse': '0.4 0.4 0.4', 'ambient': '0.1 0.1 0.1', 'specular': '0.4 0.4 0.4'})
    ET.SubElement(visual, 'rgba', {'haze': '0.15 0.25 0.35 1'}) # 科技感雾效
    ET.SubElement(visual, 'global', {'fovy': '45', 'elevation': '-20', 'azimuth': '150'})

    # 合并 Default
    new_default = ET.SubElement(new_root, 'default')
    
    # --- 处理 Z1 Default (注入材质) ---
    if z1_parts['default'] is not None:
        for child in z1_parts['default']:
            # 找到 z1 类定义
            if child.get('class') == 'z1':
                z1_node = copy.deepcopy(child)
                # 寻找 visual 子类并修改
                vis_node = None
                for sub in z1_node:
                    if sub.tag == 'default' and sub.get('class') == 'visual':
                        vis_node = sub
                        break
                
                if vis_node is not None:
                    # 找到 geom 节点并修改
                    geom_node = vis_node.find('geom')
                    if geom_node is not None:
                        geom_node.set('material', 'robot_metal')
                    else:
                        ET.SubElement(vis_node, 'geom', {'material': 'robot_metal'})
                else:
                    # 如果没有 visual 类，创建一个
                    vis_node = ET.SubElement(z1_node, 'default', {'class': 'visual'})
                    ET.SubElement(vis_node, 'geom', {'material': 'robot_metal'})
                
                new_default.append(z1_node)
            else:
                new_default.append(copy.deepcopy(child))

    if cf2_parts and cf2_parts['default'] is not None:
        for child in cf2_parts['default']:
            new_default.append(copy.deepcopy(child))

    # 合并 Asset
    new_asset = ET.SubElement(new_root, 'asset')
    
    # --- 添加高科技材质 ---
    ET.SubElement(new_asset, 'texture', {'name': 'grid', 'type': '2d', 'builtin': 'checker', 'width': '512', 'height': '512', 'rgb1': '.1 .2 .3', 'rgb2': '.2 .3 .4'})
    ET.SubElement(new_asset, 'material', {'name': 'grid', 'texture': 'grid', 'texrepeat': '1 1', 'texuniform': 'true', 'reflectance': '0.2'})
    
    # 机器人金属材质 (深空灰 + 强高光)
    ET.SubElement(new_asset, 'material', {'name': 'robot_metal', 'rgba': '0.3 0.3 0.35 1', 'specular': '1.0', 'shininess': '0.8', 'reflectance': '0.5'})
    
    # 传送带材质 (改为工业蓝灰)
    ET.SubElement(new_asset, 'material', {'name': 'conveyor_mat', 'rgba': '0.2 0.25 0.3 1', 'specular': '0.3', 'reflectance': '0.2'})

    for p in [z1_parts, cf2_parts]:
        if p and p['asset'] is not None: new_asset.extend(copy.deepcopy(list(p['asset'])))
    
    # 构建 Worldbody
    new_world = ET.SubElement(new_root, 'worldbody')
    
    # --- 赛博朋克灯光 (调暗) ---
    # 主光源 (冷白，降低亮度)
    ET.SubElement(new_world, 'light', {'pos': '0 0 5', 'dir': '0 0 -1', 'diffuse': '0.5 0.5 0.6', 'specular': '0.2 0.2 0.2', 'castshadow': 'true'})
    # 侧边氛围光 (青色 Neon) - 位于传送带两侧 (降低亮度)
    ET.SubElement(new_world, 'light', {'pos': '0 -2 2', 'dir': '0 1 -0.5', 'diffuse': '0.4 0.8 0.8', 'specular': '0.4 0.8 0.8', 'castshadow': 'false', 'attenuation': '0.5 0.1 0.01'})
    ET.SubElement(new_world, 'light', {'pos': '0 2 2', 'dir': '0 -1 -0.5', 'diffuse': '0.4 0.8 0.8', 'specular': '0.4 0.8 0.8', 'castshadow': 'false', 'attenuation': '0.5 0.1 0.01'})

    # 地板
    ET.SubElement(new_world, 'geom', {'name': 'floor', 'size': '5 5 0.01', 'type': 'plane', 'material': 'grid'})

    # 传送带场景（原样保留）
    conveyor = ET.SubElement(new_world, 'body', {'name': 'conveyor', 'pos': '0 0 0.4'})
    ET.SubElement(conveyor, 'geom', {'type': 'box', 'size': '2.5 0.3 0.05', 'material': 'conveyor_mat'})
    
    # --- 警示灯模型 (位于产线前端 x=-2.3) ---
    # 灯杆
    alarm_pole = ET.SubElement(conveyor, 'body', {'name': 'alarm_pole', 'pos': '-2.3 -0.35 0.25'})
    ET.SubElement(alarm_pole, 'geom', {'type': 'cylinder', 'size': '0.02 0.2', 'rgba': '0.2 0.2 0.2 1'}) # 黑色杆
    # 灯泡 (球体)
    # 注意：这里不设置 material，使用 rgba 以便在代码中动态修改
    ET.SubElement(alarm_pole, 'geom', {'name': 'alarm_bulb', 'type': 'sphere', 'size': '0.08', 'pos': '0 0 0.2', 'rgba': '0.5 0.5 0.5 1'})
    # 光源 (放入灯泡内部)
    ET.SubElement(alarm_pole, 'light', {'name': 'alarm_light', 'pos': '0 0 0.2', 'dir': '0 0 -1', 'mode': 'fixed', 'diffuse': '1 0 0', 'attenuation': '1 0 0'})
    
    carrier = ET.SubElement(conveyor, 'body', {'name': 'drone_carrier', 'pos': '0 0 0.06'})
    # 添加滑动关节，允许沿 X 轴移动
    ET.SubElement(carrier, 'joint', {'name': 'carrier_slide', 'type': 'slide', 'axis': '1 0 0', 'damping': '50'})
    ET.SubElement(carrier, 'geom', {'type': 'box', 'size': '0.15 0.15 0.01', 'rgba': '0.5 0.5 0.5 1'})

    if cf2_parts:
        cf2_body = copy.deepcopy(cf2_parts['worldbody'].find('body'))
        strip_free_joints(cf2_body);
        rename_instance(cf2_body, 'drone');
        carrier.append(cf2_body)

    # 注入机械臂
    new_actuator = ET.SubElement(new_root, 'actuator')
    
    # 优化布局：更靠近传送带中心，且通过 Euler 角让机械臂默认朝向中心 (0,0)
    # 计算逻辑：atan2(y_dist, x_dist) 指向中心
    # L1 (-0.45, -0.35) -> 指向 (+, +) -> 0.66 rad
    # L2 (0.45, -0.35)  -> 指向 (-, +) -> 2.48 rad
    # R1 (-0.45, 0.35)  -> 指向 (+, -) -> -0.66 rad
    # R2 (0.45, 0.35)   -> 指向 (-, -) -> -2.48 rad
    arm_configs = [
        ('L1', '-0.45 -0.35 0', '0 0 0.66'), 
        ('L2', '0.45 -0.35 0', '0 0 2.48'),
        ('R1', '-0.45 0.35 0', '0 0 -0.66'), 
        ('R2', '0.45 0.35 0', '0 0 -2.48')
    ]

    for prefix, pos, euler in arm_configs:
        arm_base = ET.SubElement(new_world, 'body', {'name': f"{prefix}_base", 'pos': pos, 'euler': euler})
        z1_body = copy.deepcopy(z1_parts['worldbody'].find('body'))

        # 【关键：显式设置 childclass】确保此 body 节点能找到 z1 类
        z1_body.set('childclass', 'z1')

        rename_instance(z1_body, prefix);
        arm_base.append(z1_body)
        z1_act = copy.deepcopy(z1_parts['actuator'])
        rename_instance(z1_act, prefix);
        new_actuator.extend(list(z1_act))

    # 添加传送带驱动器
    ET.SubElement(new_actuator, 'position', {'name': 'conveyor_drive', 'joint': 'carrier_slide', 'kp': '2000', 'ctrlrange': '-2.5 2.5'})

    # 生成最终 XML
    tree = ET.ElementTree(new_root)
    tree.write('scene_final.xml', encoding='utf-8', xml_declaration=True)

    # --- 2. 仿真逻辑：工业生产场景 (优化版) ---
    model = mujoco.MjModel.from_xml_path('scene_final.xml')
    data = mujoco.MjData(model)

    # 获取电机 ID
    arm_motors = {}
    for prefix in ['L1', 'L2', 'R1', 'R2']:
        arm_motors[prefix] = [model.actuator(f"{prefix}_motor{i}").id for i in range(1, 7)]
    
    # 传送带驱动 ID
    conveyor_id = model.actuator('conveyor_drive').id
    
    # 警示灯 ID
    alarm_light_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_LIGHT, "alarm_light")
    # 警示灯灯泡 Geom ID
    alarm_bulb_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "alarm_bulb")

    # 定义关键姿态 (Key Poses) [J1, J2, J3, J4, J5, J6]
    # J1=0 默认指向中心(由 Euler 保证)
    POSES = {
        'HOME': np.array([0, 0.5, -1.2, 0, 0, 0]),       # 待机位：收起
        'READY': np.array([0, 0.8, -1.0, 0, 0, 0]),      # 准备位
        'WORK_HIGH': np.array([0, 1.2, -0.8, 0, 0, 0]),  # 工作位(高)
        'WORK_LOW': np.array([0, 1.5, -1.2, 0, 0, 0]),   # 工作位(低 - 接近无人机)
        'SCAN_L': np.array([0.4, 1.2, -1.0, 0, 0.5, 0]), # 扫描左侧
        'SCAN_R': np.array([-0.4, 1.2, -1.0, 0, -0.5, 0])# 扫描右侧
    }

    # 状态控制
    state = {'mode': 9, 'arrived': False} # 9=Transporting

    def key_callback(keycode):
        if keycode == ord('1'): state['mode'] = 1    # 装配模式
        elif keycode == ord('2'): state['mode'] = 2  # 检测模式
        elif keycode == ord('0'): state['mode'] = 0  # 待机模式
        elif keycode == ord('9'): state['mode'] = 9  # 运输模式 (Reset)

    def get_target_pose(mode, t, prefix):
        # 如果还在运输中，机械臂保持待机
        if not state['arrived'] and mode != 0:
            return POSES['HOME']

        # 相位偏移：让机械臂动作错落有致
        offsets = {'L1': 0, 'L2': 1.0, 'R1': 0.5, 'R2': 1.5}
        local_t = t + offsets[prefix]
        
        if mode == 1: # 装配模式 (Pick & Place)
            cycle = 5.0
            p = (local_t % cycle) / cycle
            # 轨迹：Ready -> Work_High -> Work_Low -> Work_High -> Ready
            if p < 0.2: return POSES['READY']
            elif p < 0.4: # 下降
                a = (p - 0.2) / 0.2
                return (1-a)*POSES['READY'] + a*POSES['WORK_LOW']
            elif p < 0.6: # 作业
                return POSES['WORK_LOW']
            elif p < 0.8: # 上升
                a = (p - 0.6) / 0.2
                return (1-a)*POSES['WORK_LOW'] + a*POSES['READY']
            else: return POSES['READY']
            
        elif mode == 2: # 检测模式 (Scanning)
            cycle = 4.0
            p = (local_t % cycle) / cycle
            # 轨迹：Scan_L <-> Scan_R 平滑摆动
            alpha = 0.5 + 0.5 * np.sin(p * 2 * np.pi)
            return (1-alpha)*POSES['SCAN_L'] + alpha*POSES['SCAN_R']
            
        else: # 待机
            return POSES['HOME']

    # 初始化控制信号
    current_ctrl = {p: POSES['HOME'].copy() for p in arm_motors}
    alpha = 0.05 # 平滑系数
    
    # 传送带逻辑参数
    conveyor_pos = -2.0 # 起始位置 (左侧)
    conveyor_speed = 0.5

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        start_time = time.time()
        print(">>> 场景加载成功！\n [9] 重置运输 | [1] 装配 | [2] 检测 | [0] 待机")
        
        while viewer.is_running():
            step_start = time.time()
            t = time.time() - start_time
            
            # --- 警示灯逻辑 ---
            # 1秒周期：0.5s 红, 0.5s 蓝
            # 同步修改光源颜色和灯泡几何体颜色（模拟自发光）
            if (t % 1.0) < 0.5:
                 color = [1, 0, 0] # Red
                 model.light_diffuse[alarm_light_id] = color
                 model.geom_rgba[alarm_bulb_id][:3] = color
                 model.geom_rgba[alarm_bulb_id][3] = 0.9 # 不透明度
            else:
                 color = [0, 0, 1] # Blue
                 model.light_diffuse[alarm_light_id] = color
                 model.geom_rgba[alarm_bulb_id][:3] = color
                 model.geom_rgba[alarm_bulb_id][3] = 0.9

            # --- 传送带逻辑 ---
            if state['mode'] == 9: # 运输模式
                state['arrived'] = False
                if conveyor_pos < 0:
                    conveyor_pos += conveyor_speed * 0.002 # 简单的积分
                    if conveyor_pos >= 0:
                        conveyor_pos = 0
                        state['arrived'] = True
                        state['mode'] = 1 # 到达后自动切换到装配模式
                        print(">>> 无人机已到位，开始装配！")
            else:
                # 如果手动切换到其他模式，确保位置在中心
                if not state['arrived']:
                     conveyor_pos = 0
                     state['arrived'] = True

            # 应用传送带控制
            data.ctrl[conveyor_id] = conveyor_pos

            # --- 机械臂逻辑 ---
            for prefix, ids in arm_motors.items():
                target = get_target_pose(state['mode'], t, prefix)
                
                # 低通滤波平滑控制
                current_ctrl[prefix] = (1 - alpha) * current_ctrl[prefix] + alpha * target
                
                # 应用控制信号
                for i in range(6):
                    data.ctrl[ids[i]] = current_ctrl[prefix][i]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, 0.002 - (time.time() - step_start)))


if __name__ == "__main__":
    build_factory_xml()