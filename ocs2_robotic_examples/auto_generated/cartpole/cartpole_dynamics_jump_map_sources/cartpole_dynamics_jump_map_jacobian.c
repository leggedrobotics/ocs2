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

void cartpole_dynamics_jump_map_jacobian(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   // dependent variables without operations
   jac[0] = 0;
   jac[1] = 1;
   jac[2] = 0;
   jac[3] = 0;
   jac[4] = 0;
   jac[5] = 0;
   jac[6] = 0;
   jac[7] = 0;
   jac[8] = 1;
   jac[9] = 0;
   jac[10] = 0;
   jac[11] = 0;
   jac[12] = 0;
   jac[13] = 0;
   jac[14] = 0;
   jac[15] = 1;
   jac[16] = 0;
   jac[17] = 0;
   jac[18] = 0;
   jac[19] = 0;
   jac[20] = 0;
   jac[21] = 0;
   jac[22] = 1;
   jac[23] = 0;
}

