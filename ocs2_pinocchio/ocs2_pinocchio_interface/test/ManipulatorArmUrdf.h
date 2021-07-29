
#pragma once

// clang-format off
/* Mobile manipulator arm raw URDF string */
static constexpr auto manipulatorArmUrdf = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="mm_arm">
  <material name="grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="mm_red">
    <color rgba="0.698039215686 0.133333333333 0.133333333333 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base"/>
  <!-- Fixed joint to add dummy inertia link -->
  <joint name="world_to_arm_base" type="fixed">
    <parent link="base"/>
    <child link="arm_base"/>
    <origin rpy="0 0 0" xyz="0 0 0.2"/>
  </joint>
  <!-- Base link -->
  <link name="arm_base">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.1"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.411764705882 0.411764705882 0.411764705882 1.0"/>
      </material>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="-0.35 0 0.3"/>
      <geometry>
        <box size="0.2 0.5 0.8"/>
      </geometry>
      <material name="gray">
        <color rgba="0.411764705882 0.411764705882 0.411764705882 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.1"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 0" xyz="-0.35 0 0.3"/>
      <geometry>
        <box size="0.2 0.5 0.8"/>
      </geometry>
    </collision>
  </link>
  <!-- Fixed joint to add dummy inertia link -->
  <joint name="arm_base_to_arm_base_inertia" type="fixed">
    <parent link="arm_base"/>
    <child link="arm_base_inertia"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <!-- Dummy inertia link, because KDL cannot have inertia on the base link -->
  <link name="arm_base_inertia">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0648"/>
      <mass value="1000"/>
      <inertia ixx="0.00322" ixy="0.0" ixz="0.0" iyy="0.00322" iyz="0.0" izz="0.0023264"/>
    </inertial>
  </link>

  <!-- Arm components -->
  <!-- shoulder link -->
  <link name="SHOULDER">
    <visual>
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.03"/>
      <geometry>
        <cylinder length="0.31" radius="0.09"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <collision name="collision">
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.03"/>
      <geometry>
        <cylinder length="0.31" radius="0.09"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.047924822 0.0 -0.002274739"/>
      <mass value="7.6494994"/>
      <inertia ixx="0.029999999" ixy="3.2869e-05" ixz="0.000721956" iyy="0.059745945" iyz="1.0358e-05" izz="0.056496362"/>
    </inertial>
  </link>
  <!-- Joint: SH_ROT -->
  <joint name="SH_ROT" type="revolute">
    <parent link="arm_base"/>
    <child link="SHOULDER"/>
    <origin rpy="0 0 0" xyz="0.0 0.0 0.189"/>
    <axis xyz="0 0 1"/>
    <limit effort="150.0" lower="-5.23598775598" upper="5.23598775598" velocity="1.308996939"/>
    <dynamics damping="0.1" friction="0.3"/>
  </joint>

  <!-- arm link -->
  <link name="ARM">
    <visual>
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.205"/>
      <geometry>
        <cylinder length="0.22" radius="0.09"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <visual>
      <origin rpy="0.0 1.57079632679 0.0" xyz="0.36 0.0 0.195"/>
      <geometry>
        <cylinder length="0.52" radius="0.085"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <visual>
      <origin rpy="0.0 0.0 0.0" xyz="0.7 0.0 0.205"/>
      <geometry>
        <cylinder length="0.19" radius="0.07"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <collision name="collision_motor_1">
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.205"/>
      <geometry>
        <cylinder length="0.22" radius="0.09"/>
      </geometry>
    </collision>
    <collision name="collision_arm">
      <origin rpy="0.0 1.57079632679 0.0" xyz="0.36 0.0 0.195"/>
      <geometry>
        <cylinder length="0.52" radius="0.085"/>
      </geometry>
    </collision>
    <collision name="collision_motor_2">
      <origin rpy="0.0 0.0 0.0" xyz="0.7 0.0 0.205"/>
      <geometry>
        <cylinder length="0.19" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.29409722 0.0 0.19270522"/>
      <mass value="13.640494"/>
      <inertia ixx="0.060898243" ixy="-0.00013513623" ixz="-0.013074267" iyy="1.1757653" iyz="-2.6102602e-05" izz="1.1722998"/>
    </inertial>
  </link>
  <!-- Joint: SH_FLE -->
  <joint name="SH_FLE" type="revolute">
    <parent link="SHOULDER"/>
    <child link="ARM"/>
    <origin rpy="-1.57079632679 0 1.57079632679" xyz="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <limit effort="150.0" lower="-3.14159265359" upper="0.0" velocity="1.308996939"/>
    <dynamics damping="0.1" friction="0.3"/>
  </joint>
  <!-- Elbow link -->
  <link name="ELBOW">
    <visual>
      <origin rpy="0.0 1.57079632679 0.0" xyz="0.115 0.0 0.0"/>
      <geometry>
        <cylinder length="0.46" radius="0.066"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <collision name="collision">
      <origin rpy="0.0 1.57079632679 0.0" xyz="0.115 0.0 0.0"/>
      <geometry>
        <cylinder length="0.46" radius="0.066"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0045213596 -0.0001046407 0.035705904"/>
      <mass value="4.6480036"/>
      <inertia ixx="0.021549076" ixy="-2.5845846e-05" ixz="-0.00062888563" iyy="0.02331519" iyz="-3.1906253e-05" izz="0.011591539"/>
    </inertial>
  </link>
  <!-- Joint: EL_FLE -->
  <joint name="EL_FLE" type="revolute">
    <parent link="ARM"/>
    <child link="ELBOW"/>
    <origin rpy="0 0 0" xyz="0.7 0.0 0.04"/>
    <axis xyz="0 0 1"/>
    <limit effort="100.0" lower="-2.47836753783" upper="2.47836753783" velocity="2.53072741539"/>
    <dynamics damping="0.1" friction="0.2"/>
  </joint>
  <!-- Forearm link -->
  <link name="FOREARM">
    <visual>
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.115 0.37"/>
      <geometry>
        <cylinder length="0.25" radius="0.048"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <visual>
      <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.13 0.55"/>
      <geometry>
        <cylinder length="0.145" radius="0.05"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <collision name="collision_farm">
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.115 0.37"/>
      <geometry>
        <cylinder length="0.25" radius="0.048"/>
      </geometry>
    </collision>
    <collision name="collision_motor">
      <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.13 0.55"/>
      <geometry>
        <cylinder length="0.145" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00051024005 0.047442921 0.27441632"/>
      <mass value="5.7271652"/>
      <inertia ixx="0.24731453" ixy="-0.00015557711" ixz="-0.0003643398" iyy="0.2306736" iyz="-0.05673063" izz="0.029585235"/>
    </inertial>
  </link>
  <!-- Joint: EL_ROT -->
  <joint name="EL_ROT" type="revolute">
    <parent link="ELBOW"/>
    <child link="FOREARM"/>
    <origin rpy="1.57079632679 0 1.57079632679" xyz="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <limit effort="40.0" lower="-5.23598775598" upper="5.23598775598" velocity="2.53072741539"/>
    <dynamics damping="0.1" friction="0.2"/>
  </joint>
  <!-- wrist1 link -->
  <link name="WRIST_1">
    <visual>
      <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.016 0.0"/>
      <geometry>
        <cylinder length="0.148" radius="0.05"/>
      </geometry>
      <material name="mm_red"/>
    </visual>
    <collision name="collision">
      <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.016 0.0"/>
      <geometry>
        <cylinder length="0.148" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <mass value="1.9710281"/>
      <inertia ixx="0.0066945672" ixy="4.5275e-06" ixz="-7.5009e-06" iyy="0.0055642118" iyz="0.0003509941" izz="0.0038716147"/>
    </inertial>
  </link>
  <!-- Joint: WR_FLE -->
  <joint name="WR_FLE" type="revolute">
    <parent link="FOREARM"/>
    <child link="WRIST_1"/>
    <origin rpy="-1.57079632679 0 0" xyz="0.0 0.0 0.55"/>
    <axis xyz="0 0 1"/>
    <limit effort="40.0" lower="-2.39110107523" upper="2.39110107523" velocity="4.79965544298"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>
  <!-- Wrist2 link -->
  <link name="WRIST_2">
    <visual>
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.02"/>
      <geometry>
        <cylinder length="0.038" radius="0.05"/>
      </geometry>Z
      <material name="silver">
        <color rgba="0.737254901961 0.776470588235 0.8 1.0"/>
      </material>
    </visual>
    <collision name="collision">
      <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.02"/>
      <geometry>
        <cylinder length="0.038" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00038203947 -0.0019245031 -0.03"/>
      <mass value="1.7057686"/>
      <inertia ixx="0.0046630423" ixy="-1.6647842e-07" ixz="-2.5100296e-05" iyy="0.0046335717" iyz="4.8450275e-05" izz="0.002284043"/>
    </inertial>
  </link>
  <!-- Joint: WR_ROT -->
  <joint name="WR_ROT" type="revolute">
    <parent link="WRIST_1"/>
    <child link="WRIST_2"/>
    <origin rpy="1.57079632679 0 0" xyz="0.0 -0.1 0.0"/>
    <axis xyz="0 0 1"/>
    <limit effort="40.0" lower="-3.14159265359" upper="3.14159265359" velocity="4.79965544298"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>
</robot>
)";  // clang-format on
