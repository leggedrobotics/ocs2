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

void model_sparse_forward_one(unsigned long pos, double const *const * in, double*const * out, struct LangCAtomicFun atomicFun);
void model_forward_one_sparsity(unsigned long pos, unsigned long const** elements, unsigned long* nnz);

int model_forward_one(double const tx[], double ty[], struct LangCAtomicFun atomicFun) {
   unsigned long ePos, ej, i, j, nnz, nnzMax;
   unsigned long const* pos;
   unsigned long* txPos;
   unsigned long nnzTx;
   double const * in[2];
   double* out[1];
   double x[2];
   double* compressed;

   txPos = 0;
   nnzTx = 0;
   nnzMax = 0;
   for (j = 0; j < 2; j++) {
      if (tx[j * 2 + 1] != 0.0) {
         nnzTx++;
         txPos = (unsigned long*) realloc(txPos, nnzTx * sizeof(unsigned long));
         txPos[nnzTx - 1] = j;
         model_forward_one_sparsity(j, &pos, &nnz);
         if(nnz > nnzMax)
            nnzMax = nnz;
      }
   }
   for (i = 0; i < 2; i++) {
      ty[i * 2 + 1] = 0;
   }

   if (nnzTx == 0) {
      free(txPos);
      return 0; //nothing to do
   }

   compressed = (double*) malloc(nnzMax * sizeof(double));

   for (j = 0; j < 2; j++)
      x[j] = tx[j * 2];

   for (ej = 0; ej < nnzTx; ej++) {
      j = txPos[ej];
      model_forward_one_sparsity(j, &pos, &nnz);

      in[0] = x;
      in[1] = &tx[j * 2 + 1];
      out[0] = compressed;
      model_sparse_forward_one(j, in, out, atomicFun);

      for (ePos = 0; ePos < nnz; ePos++) {
         ty[pos[ePos] * 2 + 1] += compressed[ePos];
      }

   }
   free(compressed);
   free(txPos);
   return 0;
}
