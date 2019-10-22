void model_hessian_sparsity2(unsigned long i,unsigned long const** row, unsigned long const** col, unsigned long* nnz) {
   static unsigned long const rows0[1] = {1};
   static unsigned long const cols0[1] = {1};
   switch(i) {
   case 0:
      *row = rows0;
      *col = cols0;
      *nnz = 1;
      break;
   default:
      *row = 0;
      *col = 0;
      *nnz = 0;
   break;
   };
}
