## Section 5. Improve the URDF with Xacro

In this section, you will learn to use Xacro to make the URDF cleaner, more dynamic, modular, and scalable. Xacro is an XML macro language, which is a tool used in ROS that provides macros and code generation capabilities.

### Make the URDF Compatible with Xacro

When Xacro is not installed, it can be installed by the following:
```
sudo apt update
sudo apt install ros-humble-xacro
```

Create a URDF file with xacro file extension by copying the previously created URDF file.

```
cd ~/ros2_ws/src/my_robot_description/urdf/
cp my_robot.urdf my_robot.urdf.xacro
```

Modify the robot tag in `my_robot.urdf.xacro` file as shown below to make it Xacro compatible.

```
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

Make changes to the URDF path in `display.launch.xml` as shown below so the new URDF file can be located.

```
<let name="urdf_path" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro" />
```

Also make similar changes to the URDF path in `display.launch.py` as shown below.

```
urdf_path = os.path.join(get_package_share_path("my_robot_description"), "urdf", "my_robot.urdf.xacro")
```

Build the workspace with `--symlink-install` option, so you don't need to build the workspace every time you make changes to URDF and launch files.

```
cd ~/ros2_ws/
colcon build --symlink-install
```

### Create Variables with Xacro Properties

Modify `my_robot.urdf.xacro` by using Xacro properties as shown below. Define variables with Xacro properties and use the expression `${}` to replace the previously hard coded values with the variables. Note that `${pi}` is also recognized with Xacro. You can also add mathematical operations within the curly brackets.

```
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />

    <material name="blue">
        <color rgba="0 0 0.5 1" />
    </material>

    <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
    </material>

    <link name="base_footprint" />

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}" />
            </geometry>
            <origin xyz="0 0 ${base_height / 2.0}" rpy="0 0 0" />
            <material name="blue" />
        </visual>
    </link>

    <link name="right_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0" />
            <material name="grey" />
        </visual>
    </link>

    <link name="left_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0" />
            <material name="grey" />
        </visual>
    </link>

    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius / 2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="grey" />
        </visual>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="${-base_length / 4.0} ${-(base_width + wheel_length) / 2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="${-base_length / 4.0} ${(base_width + wheel_length) / 2.0}  0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="${base_length / 3.0} 0 ${-wheel_radius / 2.0}" rpy="0 0 0" />
    </joint>
</robot>
```

Start `/robot_state_publisher` node with the launch file and check visualization of the robot model and TFs.

```
cd ~/ros2_ws/
source install/setup.bash
ros2 launch my_robot_description display.launch.xml
```

### Create Functions with Xacro Macros

With macros you can create reusable blocks and add parameters. For example, `xacro:macro` tags can be written as follows:

```
<xacro:macro name="example_macro" params="a b c">
    <link name="dummy_link">
        <visual>
            <geometry>
                <box size="${a} ${b} ${c}" />
            </geometry>
        </visual>
    </link>
</xacro:macro>
```

Once defined, this macro can be used as shown below.

```
<xacro:example_macro a="2" b="3" c="4" />
```

Now replace `right_wheel_link` and `left_wheel_link` in `my_robot.urdf.xacro` with `xacro:macro` tag.

```
<xacro:macro name="wheel_link" params="prefix">
    <link name="${prefix}_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0" />
            <material name="grey" />
        </visual>
    </link>
</xacro:macro>

<xacro:wheel_link prefix="right" />
<xacro:wheel_link prefix="left" />
```

### Include a Xacro File in Another Xacro File

Create additional URDF files to split the URDF content in `my_robot.urdf.xacro` file and separately store similar parts.

```
cd ~/ros2_ws/src/my_robot_description/urdf/
touch common_properties.xacro mobile_base.xacro
```

Cut and paste relevant tags into the newly created URDF files. First edit `common_properties.xacro` as shown below. Notice the robot tag does not include the robot name as the robot name should only be specified in the main URDF file.

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <material name="blue">
        <color rgba="0 0 0.5 1" />
    </material>

    <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
    </material>

</robot>
```

Edit `mobile_base.xacro` to include all the link and joint tags for the mobile base.

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />

    <link name="base_footprint" />

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}" />
            </geometry>
            <origin xyz="0 0 ${base_height / 2.0}" rpy="0 0 0" />
            <material name="blue" />
        </visual>
    </link>

    <xacro:macro name="wheel_link" params="prefix">
        <link name="${prefix}_wheel_link">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}" />
                </geometry>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0" />
                <material name="grey" />
            </visual>
        </link>
    </xacro:macro>

    <xacro:wheel_link prefix="right" />
    <xacro:wheel_link prefix="left" />

    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius / 2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="grey" />
        </visual>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="${-base_length / 4.0} ${-(base_width + wheel_length) / 2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="${-base_length / 4.0} ${(base_width + wheel_length) / 2.0}  0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="${base_length / 3.0} 0 ${-wheel_radius / 2.0}" rpy="0 0 0" />
    </joint>

</robot>
```

Now add `xacro:include` tags to `my_robot.urdf.xacro`. After applying the changes `my_robot.urdf.xacro` file should look like this.

```
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="common_properties.xacro" />
    <xacro:include filename="mobile_base.xacro" />

</robot>
```

Build the workspace.

```
cd ~/ros2_ws/
colcon build --symlink-install
```
