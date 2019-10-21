void model_hessian_sparsity(unsigned long const** row, unsigned long const** col, unsigned long* nnz) {
   static unsigned long const rows[1] = {1};
   static unsigned long const cols[1] = {1};
   *row = rows;
   *col = cols;
   *nnz = 1;
}
