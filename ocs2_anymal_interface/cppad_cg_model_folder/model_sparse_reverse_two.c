typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel, int atomicIndex, int q, int p, const Array tx[], Array* ty);
    int (*reverse)(void* libModel, int atomicIndex, int p, const Array tx[], Array* px, const Array py[]);
};

void model_sparse_reverse_two_indep1(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun);

int model_sparse_reverse_two(unsigned long pos, double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   switch(pos) {
      case 1:
         model_sparse_reverse_two_indep1(in, out, atomicFun);
         return 0; // done
      default:
         return 1; // error
   };
}
