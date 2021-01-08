
#pragma once

// clang-format off
/* Cart pole raw URDF string */
static constexpr auto cartPoleUrdf = R"(
<?xml version="1.0"?>
<robot name="cartpole">

  <link name="world"/>

  <link name="cart">
    <collision>
      <geometry>
          <box size="0.5 0.5 0.2"/>
      </geometry>
      <origin xyz="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="world_to_cart" type="prismatic">
    <axis xyz="1 0 0"/>
    <origin xyz="0.0 0.0 0.0"/>
    <parent link="world"/>
    <child link="cart"/>
    <limit effort="100" lower="-100" upper="100" velocity="100"/>
  </joint>

  <link name="pole">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0.5"/>
      <mass value="0.2"/>
      <inertia ixx="0.0000001" ixy="0.0" ixz="0.0" iyy="0.01667" iyz="0.0" izz="0.0000001"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.05 0.05 1.0"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0.5"/>
    </collision>
  </link>

  <joint name="cart_to_pole" type="continuous">
    <axis xyz="0 1 0"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0"/>
    <parent link="cart"/>
    <child link="pole"/>
  </joint>
</robot>
)";  // clang-format on
