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

void model_sparse_forward_one_indep1(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];
   const double* dx = in[1];

   //dependent variables
   double* dy = out[0];

   // auxiliary variables

   dy[0] = (x[1] + x[1]) / 2. * dx[0];
   dy[1] = -2. * dx[0];
}

