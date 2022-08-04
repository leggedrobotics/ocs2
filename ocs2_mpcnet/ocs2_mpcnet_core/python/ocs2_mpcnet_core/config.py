###############################################################################
# Copyright (c) 2022, Farbod Farshidian. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#  * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

"""Configuration class.

Provides a class that handles the configuration parameters.
"""

import yaml
import torch


class Config:
    """Config.

    Loads configuration parameters from a YAML file and provides access to them as attributes of this class.

    Attributes:
        DTYPE: The PyTorch data type.
        DEVICE: The PyTorch device to select.
    """

    def __init__(self, config_file_path: str) -> None:
        """Initializes the Config class.

        Initializes the Config class by setting fixed attributes and by loading attributes from a YAML file.

        Args:
            config_file_path: A string with the path to the configuration file.
        """
        #
        # class config
        #
        # data type for tensor elements
        self.DTYPE = torch.float
        # device on which tensors will be allocated
        self.DEVICE = torch.device("cuda")
        #
        # yaml config
        #
        with open(config_file_path, "r") as stream:
            config = yaml.safe_load(stream)
            for key, value in config.items():
                setattr(self, key, value)
