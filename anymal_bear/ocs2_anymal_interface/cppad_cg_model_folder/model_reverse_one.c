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

void model_sparse_reverse_one(unsigned long pos, double const *const * in, double*const * out, struct LangCAtomicFun atomicFun);
void model_reverse_one_sparsity(unsigned long pos, unsigned long const** elements, unsigned long* nnz);

int model_reverse_one(double const x[], double const ty[],double  px[], double const py[], struct LangCAtomicFun atomicFun) {
   unsigned long ei, ePos, i, j, nnz, nnzMax;
   unsigned long const* pos;
   unsigned long* pyPos;
   unsigned long nnzPy;
   double const * in[2];
   double* out[1];
   double* compressed;

   pyPos = 0;
   nnzPy = 0;
   nnzMax = 0;
   for (i = 0; i < 2; i++) {
      if (py[i] != 0.0) {
         nnzPy++;
         pyPos = (unsigned long*) realloc(pyPos, nnzPy * sizeof(unsigned long));
         pyPos[nnzPy - 1] = i;
         model_reverse_one_sparsity(i, &pos, &nnz);
         if(nnz > nnzMax)
            nnzMax = nnz;
      }
   }
   for (j = 0; j < 2; j++) {
      px[j] = 0;
   }

   if (nnzPy == 0) {
      free(pyPos);
      return 0; //nothing to do
   }

   compressed = (double*) malloc(nnzMax * sizeof(double));

   for (ei = 0; ei < nnzPy; ei++) {
      i = pyPos[ei];
      model_reverse_one_sparsity(i, &pos, &nnz);

      in[0] = x;
      in[1] = &py[i];
      out[0] = compressed;
      model_sparse_reverse_one(i, in, out, atomicFun);

      for (ePos = 0; ePos < nnz; ePos++) {
         px[pos[ePos]] += compressed[ePos];
      }

   }
   free(compressed);
   free(pyPos);
   return 0;
}
