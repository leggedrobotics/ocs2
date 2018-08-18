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

void cartpole_dynamics_flow_map_jacobian(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[12];

   v[0] = -0.5 * cos(x[1]);
   v[1] = 0.667083333333333 - v[0] * v[0];
   v[2] = 1 / v[1];
   jac[23] = 0.333541666666667 * v[2];
   v[3] = 0 - v[0];
   jac[17] = v[3] * v[2];
   v[4] = 0 - jac[17];
   v[5] = 0.5 * x[3] * x[3];
   v[6] = cos(x[1]);
   v[7] = sin(x[1]);
   v[8] = x[5] - v[5] * v[7];
   v[9] = 4.9 * v[7];
   v[10] = 0 - (0 - (v[8] * v[3] + v[9] * 2.) * 1 / v[1] * v[2]);
   v[11] = sin(x[1]);
   jac[13] = (2. * v[2] * 4.9 + v[4] * v[5]) * v[6] - (0 - v[8] * v[2] + v[10] * v[0] + v[10] * v[0]) * -0.5 * v[11];
   v[4] = v[4] * v[7] * 0.5;
   jac[15] = v[4] * x[3] + v[4] * x[3];
   v[4] = 0 - jac[23];
   v[8] = 0 - (0 - (v[8] * 0.333541666666667 + v[9] * v[3]) * 1 / v[1] * v[2]);
   jac[19] = (jac[17] * 4.9 + v[4] * v[5]) * v[6] - (0 - v[9] * v[2] + v[8] * v[0] + v[8] * v[0]) * -0.5 * v[11];
   v[4] = v[4] * v[7] * 0.5;
   jac[21] = v[4] * x[3] + v[4] * x[3];
   // dependent variables without operations
   jac[0] = 0;
   jac[1] = 0;
   jac[2] = 0;
   jac[3] = 1;
   jac[4] = 0;
   jac[5] = 0;
   jac[6] = 0;
   jac[7] = 0;
   jac[8] = 0;
   jac[9] = 0;
   jac[10] = 1;
   jac[11] = 0;
   jac[12] = 0;
   jac[14] = 0;
   jac[16] = 0;
   jac[18] = 0;
   jac[20] = 0;
   jac[22] = 0;
}

