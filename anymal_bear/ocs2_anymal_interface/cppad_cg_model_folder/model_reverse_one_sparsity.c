void model_reverse_one_sparsity(unsigned long pos, unsigned long const** elements, unsigned long* nnz) {
   static unsigned long const elements0[2] = {0,1};
   static unsigned long const elements1[2] = {0,1};
   switch(pos) {
   case 0:
      *elements = elements0;
      *nnz = 2;
      break;
   case 1:
      *elements = elements1;
      *nnz = 2;
      break;
   default:
      *elements = 0;
      *nnz = 0;
   break;
   };
}
