/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>

#include <glpk.h>

TEST(testGLPK, glpk) {
  glp_prob* lp;
  int ia[1 + 1000], ja[1 + 1000];
  double ar[1 + 1000], z, x1, x2, x3;
  lp = glp_create_prob();
  glp_set_prob_name(lp, "FrankWolfe");
  glp_set_obj_dir(lp, GLP_MAX);

  glp_add_rows(lp, 3);
  glp_set_row_bnds(lp, 1, GLP_UP, 0.0, 100.0);
  glp_set_row_bnds(lp, 2, GLP_UP, 0.0, 600.0);
  glp_set_row_bnds(lp, 3, GLP_UP, 0.0, 300.0);

  glp_add_cols(lp, 3);
  glp_set_col_bnds(lp, 1, GLP_LO, 0.0, 0.0);
  glp_set_obj_coef(lp, 1, 10.0);
  glp_set_col_bnds(lp, 2, GLP_LO, 0.0, 0.0);
  glp_set_obj_coef(lp, 2, 6.0);
  glp_set_col_bnds(lp, 3, GLP_LO, 0.0, 0.0);
  glp_set_obj_coef(lp, 3, 4.0);

  ia[1] = 1, ja[1] = 1, ar[1] = 1.0;  /* a[1,1] =  1 */
  ia[2] = 1, ja[2] = 2, ar[2] = 1.0;  /* a[1,2] =  1 */
  ia[3] = 1, ja[3] = 3, ar[3] = 1.0;  /* a[1,3] =  1 */
  ia[4] = 2, ja[4] = 1, ar[4] = 10.0; /* a[2,1] = 10 */
  ia[5] = 3, ja[5] = 1, ar[5] = 2.0;  /* a[3,1] =  2 */
  ia[6] = 2, ja[6] = 2, ar[6] = 4.0;  /* a[2,2] =  4 */
  ia[7] = 3, ja[7] = 2, ar[7] = 2.0;  /* a[3,2] =  2 */
  ia[8] = 2, ja[8] = 3, ar[8] = 5.0;  /* a[2,3] =  5 */
  ia[9] = 3, ja[9] = 3, ar[9] = 6.0;  /* a[3,3] =  6 */
  glp_load_matrix(lp, 9, ia, ja, ar);

  glp_smcp lpOptions_;
  glp_init_smcp(&lpOptions_);
  lpOptions_.msg_lev = GLP_MSG_ERR;

  glp_simplex(lp, &lpOptions_);
  z = glp_get_obj_val(lp);
  x1 = glp_get_col_prim(lp, 1);
  x2 = glp_get_col_prim(lp, 2);
  x3 = glp_get_col_prim(lp, 3);

  printf("z = %g; x1 = %g; x2 = %g; x3 = %g\n", z, x1, x2, x3);

  glp_delete_prob(lp);
}
