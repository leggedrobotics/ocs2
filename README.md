# OCS2 Toolbox

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/ocs2_dev/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/ocs2_dev/job/master/)

## Summary
This is a C++ library for an efficient continuous and discrete time optimal control implementation.
It includes methods for solving optimal control for continuous time problem with exogenous
signal for switching in between predefine modes. The toolbox is capable of solving constrained problems.

Our proposed method is based on a bi-level optimal control approach which synthesizes an optimal
feedback control policy for continuous inputs in the bottom-level and optimizes the switching times
in between two consecutive system modes in the top-level.

For more information refer to the project's [Documentation Page](https://leggedrobotics.github.io/ocs2/) 