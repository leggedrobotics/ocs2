# MPC mode sequence

float64[] eventTimes           # event times: its size is equal to the size of phaseSequence minus one
int8[]    modeSequence         # mode sequence: e.g., for a quadrupedal robot, it is in the set {0, 1,..., 15}