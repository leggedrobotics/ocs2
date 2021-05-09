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
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void AlmaKinoCentroidalDynamicsAD_jump_map_sparse_jacobian(double const *const * in,
                                                           double*const * out,
                                                           struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   // dependent variables without operations
   jac[0] = 1;
   jac[1] = 1;
   jac[2] = 1;
   jac[3] = 1;
   jac[4] = 1;
   jac[5] = 1;
   jac[6] = 1;
   jac[7] = 1;
   jac[8] = 1;
   jac[9] = 1;
   jac[10] = 1;
   jac[11] = 1;
   jac[12] = 1;
   jac[13] = 1;
   jac[14] = 1;
   jac[15] = 1;
   jac[16] = 1;
   jac[17] = 1;
   jac[18] = 1;
   jac[19] = 1;
   jac[20] = 1;
   jac[21] = 1;
   jac[22] = 1;
   jac[23] = 1;
   jac[24] = 1;
   jac[25] = 1;
   jac[26] = 1;
   jac[27] = 1;
}

