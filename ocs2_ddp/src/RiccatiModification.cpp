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

#include <iostream>

#include "ocs2_ddp/riccati_equations/RiccatiModification.h"

namespace ocs2 {
namespace riccati_modification {

/**
 * Displays all variables
 */
void display(const Data& data) {
  std::cerr << std::endl;
  std::cerr << "time: " << data.time_ << "\n";
  std::cerr << "deltaQm:\n" << data.deltaQm_ << "\n";
  std::cerr << "deltaRm:\n" << data.deltaGm_ << "\n";
  std::cerr << "deltaPm:\n" << data.deltaGv_.transpose() << "\n";

  std::cerr << "constraintRangeProjector:\n" << data.constraintRangeProjector_ << "\n";
  std::cerr << "constraintNullProjector: \n" << data.constraintNullProjector_ << "\n";
  std::cerr << std::endl;
}

}  // namespace riccati_modification
}  // namespace ocs2
