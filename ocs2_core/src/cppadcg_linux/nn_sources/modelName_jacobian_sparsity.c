void modelName_jacobian_sparsity(unsigned long const** row, unsigned long const** col, unsigned long* nnz) {
   static unsigned long const rows[4] = {0,0,1,1};
   static unsigned long const cols[4] = {0,1,0,1};
   *row = rows;
   *col = cols;
   *nnz = 4;
}
