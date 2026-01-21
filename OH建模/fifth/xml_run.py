import os

xml_content = """
<mujoco model="z1_factory_v3">
    <compiler angle="radian" meshdir="assets" autolimits="true"/>
    <option integrator="implicitfast" timestep="0.002"/>

    <default>
        <default class="z1">
            <joint damping="3" frictionloss="1"/>
            <general biastype="affine" gainprm="1500" biasprm="0 -1500 -150" forcerange="-50 50"/>
            <geom type="mesh" rgba="0.2 0.2 0.2 1" mass="0.6"/>
        </default>
        <default class="go2">
            <joint damping="5" armature="0.1"/>
            <position kp="300" forcerange="-40 40"/>
            <geom rgba="0.9 0.5 0.1 1" mass="0.5"/>
        </default>
    </default>

    <asset>
        <mesh name="link00" file="z1_Link00.stl"/><mesh name="link01" file="z1_Link01.stl"/>
        <mesh name="link02" file="z1_Link02.stl"/><mesh name="link03" file="z1_Link03.stl"/>
        <mesh name="link04" file="z1_Link04.stl"/><mesh name="link05" file="z1_Link05.stl"/>
        <mesh name="link06" file="z1_Link06.stl"/>
        <texture type="2d" name="grid" builtin="checker" mark="edge" rgb1="0.1 0.2 0.3" rgb2="0.2 0.3 0.4" width="300" height="300"/>
        <material name="grid" texture="grid" texrepeat="5 5" reflectance="0.2"/>
    </asset>

    <worldbody>
        <light pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
        <geom name="floor" size="5 5 0.01" type="plane" material="grid"/>

        <body name="conveyor" pos="0 0 0.4">
            <geom type="box" size="2.5 0.3 0.05" rgba="0.2 0.2 0.2 1" mass="10"/>
            <body name="go2_payload" pos="-1.8 0 0.06">
                <joint name="belt_joint" type="slide" axis="1 0 0" damping="100"/>
                <body name="go2_trunk" pos="0 0 0.15">
                    <geom type="box" size="0.2 0.1 0.06" mass="5"/>
"""

# 生成 Go2 腿部
leg_names = ['FL', 'FR', 'HL', 'HR']
for i, leg in enumerate(leg_names):
    y_s, x_s = (1 if 'L' in leg else -1), (1 if 'F' in leg else -1)
    xml_content += f"""
                    <body name="{leg}_hip" pos="{x_s*0.15} {y_s*0.1} 0">
                        <joint name="{leg}_j1" axis="1 0 0" class="go2"/>
                        <geom type="sphere" size="0.03" mass="0.5"/>
                        <body name="{leg}_thigh" pos="0 0 -0.1">
                            <joint name="{leg}_j2" axis="0 1 0" class="go2"/>
                            <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.1" mass="0.5"/>
                            <body name="{leg}_calf" pos="0 0 -0.1">
                                <joint name="{leg}_j3" axis="0 1 0" class="go2"/>
                                <geom type="capsule" size="0.015" fromto="0 0 0 0 0 -0.1" mass="0.4"/>
                            </body>
                        </body>
                    </body>"""

xml_content += """                </body>
            </body>
        </body>"""

# 生成 4 台机械臂
arm_pos = [(-0.8, -0.6), (0.8, -0.6), (-0.8, 0.6), (0.8, 0.6)]
arm_names = ['L1', 'L2', 'R1', 'R2']
for name, (px, py) in zip(arm_names, arm_pos):
    xml_content += f"""
        <body name="{name}_base" pos="{px} {py} 0" euler="0 0 {0 if py<0 else 3.14}">
            <geom type="box" size="0.1 0.1 0.2" pos="0 0 0.2" mass="2" rgba="0.3 0.3 0.3 1"/>
            <body name="{name}_link00" pos="0 0 0.4" childclass="z1">
                <geom mesh="link00"/><body name="{name}_link01" pos="0 0 0.05"><joint name="{name}_j1" axis="0 0 1"/>
                <geom mesh="link01"/><body name="{name}_link02" pos="0 0 0.04"><joint name="{name}_j2" axis="0 1 0"/>
                <geom mesh="link02"/><body name="{name}_link03" pos="-0.3 0 0"><joint name="{name}_j3" axis="0 1 0"/>
                <geom mesh="link03"/><body name="{name}_link04" pos="0.2 0 0.05"><joint name="{name}_j4" axis="0 1 0"/>
                <geom mesh="link04"/><body name="{name}_link05" pos="0.07 0 0"><joint name="{name}_j5" axis="0 0 1"/>
                <geom mesh="link05"/><body name="{name}_link06" pos="0.05 0 0"><joint name="{name}_j6" axis="1 0 0"/>
                <geom mesh="link06"/></body></body></body></body></body></body>
            </body>
        </body>"""

xml_content += """    </worldbody>
    <actuator>
"""
for name in arm_names:
    for j in range(1, 7): xml_content += f'        <general name="{name}_m{j}" joint="{name}_j{j}" class="z1"/>\n'
for leg in leg_names:
    for j in range(1, 4): xml_content += f'        <position name="{leg}_m{j}" joint="{leg}_j{j}" class="go2"/>\n'
xml_content += """        <velocity name="belt_m" joint="belt_joint" kv="200"/>
    </actuator>
</mujoco>"""

with open("scene.xml", "w", encoding="utf-8") as f: f.write(xml_content)
print("scene.xml 已生成。")