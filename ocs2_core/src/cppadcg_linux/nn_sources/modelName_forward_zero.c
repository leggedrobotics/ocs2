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

void modelName_forward_zero(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
  // independent variables
  const double* x = in[0];

  // dependent variables
  double* y = out[0];

  // auxiliary variables

  y[0] = (x[0] + x[1] * x[1]) / 2. + -5. * x[2];
  y[1] = -2. * x[0] + -4. * x[2] - 3. * x[1];
}
