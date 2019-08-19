# MPC mode sequence

float32[] eventTimes           # event times. Its size is equal to the size of phaseSequence minus one
int8[]    subsystems           # subsystem sequence. For a quadrupedal robot, it is in the set {0, 1,..., 15}