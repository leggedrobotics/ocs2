void AnymalKinoCentroidalDynamicsAD_jump_map_info(const char** baseName,
                                                  unsigned long* m,
                                                  unsigned long* n,
                                                  unsigned int* indCount,
                                                  unsigned int* depCount) {
   *baseName = "double  d";
   *m = 24;
   *n = 25;
   *depCount = 1; // number of dependent array variables
   *indCount = 1; // number of independent array variables
}

