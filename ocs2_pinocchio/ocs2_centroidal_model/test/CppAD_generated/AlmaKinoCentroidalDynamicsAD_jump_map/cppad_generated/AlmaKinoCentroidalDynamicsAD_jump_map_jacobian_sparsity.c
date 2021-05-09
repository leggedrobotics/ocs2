void AlmaKinoCentroidalDynamicsAD_jump_map_jacobian_sparsity(unsigned long const** row,
                                                             unsigned long const** col,
                                                             unsigned long* nnz) {
   static unsigned long const rows[28] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
   static unsigned long const cols[28] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28};
   *row = rows;
   *col = cols;
   *nnz = 28;
}
