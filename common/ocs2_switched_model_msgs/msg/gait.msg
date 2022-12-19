# Gait message

float32[] eventPhases       # event phases: length equal to modesequence's - 1. Values within [0,1]
int8[]    modeSequence      # mode sequence: e.g., for a quadrupedal robot, it is in the set {0, 1,..., 15}
float32 duration
