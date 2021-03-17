# OCS2 SQP
This package contains a multiple-shooting, sequential-quadratic-programming solver for problems defined with the OCS2 toolbox.

## Dependencies
HPIPM is used as solver for the QP subproblems. Both HPIPM and Blasfeo are automatically installed and wrapped into catkin convention 
in the blasfeo_catkin and hpipm_catkin packages.

They are configured to install these dependencies into `~/.local/`. To make the shared libraries available at execution time, add the following to your ~/.bashrc

```
export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH
```