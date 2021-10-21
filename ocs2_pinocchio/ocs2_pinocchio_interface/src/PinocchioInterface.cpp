/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
#include <iomanip>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/implementation/PinocchioInterface.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
PinocchioInterfaceCppAd PinocchioInterface::toCppAd() const {
  auto cppAdModel = getModel().cast<ad_scalar_t>();

  // TODO (rgrandia) : remove after bug fix. The cast function forgets to copy this member.
  cppAdModel.supports = getModel().supports;

  return PinocchioInterfaceCppAd(cppAdModel);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// explicit instantiation
template class PinocchioInterfaceTpl<scalar_t>;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p) {
  const auto& model = p.getModel();
  os << "model.nv = " << model.nv << '\n';
  os << "model.nq = " << model.nq << '\n';
  os << "model.njoints = " << model.njoints << '\n';
  os << "model.nbodies = " << model.nbodies << '\n';
  os << "model.nframes = " << model.nframes << '\n';

  os << "\nJoints:\n";
  for (int k = 0; k < model.njoints; ++k) {
    os << std::setw(20) << model.names[k] << ":  ";
    os << " ID = " << k;
    os << '\n';
  }

  os << "\nFrames:\n";
  for (int k = 0; k < model.nframes; ++k) {
    os << std::setw(20) << model.frames[k].name << ":  ";
    os << " ID = " << k;
    os << ", parent = " << model.frames[k].parent;
    os << ", type = ";

    std::string frameType;
    if ((model.frames[k].type & pinocchio::FrameType::OP_FRAME) != 0) {
      frameType += "OP_FRAME ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::JOINT) != 0) {
      frameType += "JOINT ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::FIXED_JOINT) != 0) {
      frameType += "FIXED_JOINT ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::BODY) != 0) {
      frameType += "BODY ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::SENSOR) != 0) {
      frameType += "SENSOR ";
    }
    os << "\"" << frameType << "\"\n";
  }
  return os;
}

}  // namespace ocs2
