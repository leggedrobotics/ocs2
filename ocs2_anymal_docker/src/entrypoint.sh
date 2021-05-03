#!/bin/bash

#=============================================================================
# Copyright (C) 2021, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# Authors: Vassilios Tsounis, tsounisv@ethz.ch
#=============================================================================

#==
# Pretty-print logo
#==

# Print pretty welcome
echo -e "\e[1;95m"
cat<<MSG
  ____   _____  _____ ___      _))
 / __ \ / ____|/ ____|__ \\   >  *\     _~
| |  | | |    | (___    ) |   \`;'\\\__-' \_
| |  | | |     \___ \  / /       | )  _ \\ \\
| |__| | |____ ____) |/ /_      / / ``   w w
 \____/ \_____|_____/|____|    w w

 OCS2: Optimal Control for Switched Systems
MSG

# Turn off colors
echo -e "\e[m"

#==
# Log into the container as the host user
#==

# Magic bash command.
set -m

# Configure environment for host user
export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

# Enable sudo access
echo "root ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Proceed as host user
if [ -z "$1" ];
then
  echo "[OCS2-Docker::Entrypoint] Launching shell as user '$HOST_USERNAME'"
  sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc
else
  echo "[OCS2-Docker::Entrypoint] Running commands as user '$HOST_USERNAME': $@"
  su $HOST_USERNAME -c "$@" --rcfile /etc/bash.bashrc
fi

# EOF
