//
// Created by rgrandia on 27.04.22.
//

#include <gtest/gtest.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/FrameDeclaration.h>
#include <ocs2_anymal_models/QuadrupedPinocchioMapping.h>
#include <ocs2_anymal_models/package_path.h>

TEST(TestFrameDeclaration, loadTestFile) {
  const auto decl = anymal::frameDeclarationFromFile(anymal::getPath() + "/urdf/frame_declaration_anymal_c.info");
  EXPECT_EQ(decl.root, "base");
  EXPECT_EQ(decl.legs[0].root, "LF_HAA");
  EXPECT_EQ(decl.legs[1].tip, "RF_FOOT");
  EXPECT_EQ(decl.legs[2].joints[1], "LH_HFE");
  EXPECT_EQ(decl.collisions[3].link, "RH_KFE");
  EXPECT_DOUBLE_EQ(decl.collisions[3].radius, 0.08);
  EXPECT_DOUBLE_EQ(decl.collisions[3].offset.x(), -0.055);
}

TEST(TestFrameMapping, camelPinocchioMapping) {
  const auto interface = ocs2::getPinocchioInterfaceFromUrdfString(getUrdfString(anymal::AnymalModel::Camel));
  const auto decl = anymal::frameDeclarationFromFile(anymal::getPath() + "/urdf/frame_declaration_anymal_c.info");
  anymal::QuadrupedPinocchioMapping mapping(decl, interface);

  EXPECT_EQ(mapping.getPinocchioFootIndex(0), 0);
  EXPECT_EQ(mapping.getPinocchioFootIndex(1), 2);
  EXPECT_EQ(mapping.getPinocchioFootIndex(2), 1);
  EXPECT_EQ(mapping.getPinocchioFootIndex(3), 3);
}
