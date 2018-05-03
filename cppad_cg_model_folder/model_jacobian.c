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

void model_jacobian(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   jac[1] = 0.5 * x[1] + 0.5 * x[1];
   // dependent variables without operations
   jac[0] = 0.5;
   jac[2] = 0;
   jac[3] = -2.;
   jac[4] = -3.;
   jac[5] = -4.;
}

