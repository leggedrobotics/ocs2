#include <math.h>
#include <stdio.h>

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

void model_sparse_reverse_one_dep0(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];
   const double* py = in[1];

   //dependent variables
   double* dw = out[0];

   // auxiliary variables

   dw[0] = 0.5 * py[0];
   dw[1] = (0.5 * x[1] + 0.5 * x[1]) * py[0];
}

