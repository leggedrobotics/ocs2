void AnymalKinoCentroidalDynamicsAD_jump_map_jacobian_sparsity(unsigned long const** row,
                                                               unsigned long const** col,
                                                               unsigned long* nnz) {
   static unsigned long const rows[24] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
   static unsigned long const cols[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24};
   *row = rows;
   *col = cols;
   *nnz = 24;
}
