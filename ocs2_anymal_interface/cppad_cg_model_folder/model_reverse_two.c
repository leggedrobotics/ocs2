#include <stdlib.h>
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

void model_sparse_reverse_two(unsigned long pos, double const *const * in, double*const * out, struct LangCAtomicFun atomicFun);
void model_sparse_reverse_two_sparsity(unsigned long pos, unsigned long const** elements, unsigned long* nnz);

int model_reverse_two(double const tx[], double const ty[], double px[], double const py[], struct LangCAtomicFun atomicFun) {
    unsigned long ej, ePos, i, j, nnz, nnzMax;
    unsigned long const* pos;
    unsigned long* txPos;
    unsigned long nnzTx;
    double const * in[3];
    double* out[1];
    double x[2];
    double w[2];
    double* compressed;
    int nonZeroW;

    nonZeroW = 0;
    for (i = 0; i < 2; i++) {
        if (py[i * 2] != 0.0) {
            return 1; // error
        }
        w[i] = py[i * 2 + 1];
        if(w[i] != 0.0) nonZeroW++;
    }

    for (j = 0; j < 2; j++) {
       px[j * 2] = 0;
    }

    if(nonZeroW == 0)
        return 0; //nothing to do

   txPos = 0;
   nnzTx = 0;
   nnzMax = 0;
   for (j = 0; j < 2; j++) {
      if (tx[j * 2 + 1] != 0.0) {
         nnzTx++;
         txPos = (unsigned long*) realloc(txPos, nnzTx * sizeof(unsigned long));
         txPos[nnzTx - 1] = j;
         model_sparse_reverse_two_sparsity(j, &pos, &nnz);
         if(nnz > nnzMax)
            nnzMax = nnz;
      }
   }

   if (nnzTx == 0) {
      free(txPos);
      return 0; //nothing to do
   }

    for (j = 0; j < 2; j++)
        x[j] = tx[j * 2];

   compressed = (double*) malloc(nnzMax * sizeof(double));

   for (ej = 0; ej < nnzTx; ej++) {
      j = txPos[ej];
      model_sparse_reverse_two_sparsity(j, &pos, &nnz);

      in[0] = x;
      in[1] = &tx[j * 2 + 1];
      in[2] = w;
      out[0] = compressed;
      model_sparse_reverse_two(j, in, out, atomicFun);

      for (ePos = 0; ePos < nnz; ePos++) {
         px[pos[ePos] * 2] += compressed[ePos];
      }

   }
   free(compressed);
   free(txPos);
   return 0;
};
