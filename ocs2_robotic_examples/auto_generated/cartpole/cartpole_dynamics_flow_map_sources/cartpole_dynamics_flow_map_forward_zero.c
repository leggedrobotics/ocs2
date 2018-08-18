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

void cartpole_dynamics_flow_map_forward_zero(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[4];

   v[0] = -0.5 * cos(x[1]);
   v[1] = 1 / (0.667083333333333 - v[0] * v[0]);
   v[2] = sin(x[1]);
   v[3] = 4.9 * v[2];
   v[0] = (0 - v[0]) * v[1];
   v[2] = x[5] - 0.5 * x[3] * x[3] * v[2];
   y[2] = 2. * v[1] * v[3] + v[0] * v[2];
   y[3] = v[0] * v[3] + 0.333541666666667 * v[1] * v[2];
   // dependent variables without operations
   y[0] = x[3];
   y[1] = x[4];
}

