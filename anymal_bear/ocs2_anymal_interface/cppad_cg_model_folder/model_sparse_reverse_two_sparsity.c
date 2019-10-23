void model_sparse_reverse_two_sparsity(unsigned long pos, unsigned long const** elements, unsigned long* nnz) {
   static unsigned long const elements1[1] = {1};
   switch(pos) {
   case 1:
      *elements = elements1;
      *nnz = 1;
      break;
   default:
      *elements = 0;
      *nnz = 0;
   break;
   };
}
