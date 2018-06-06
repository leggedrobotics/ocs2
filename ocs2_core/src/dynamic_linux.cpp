/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

#include <iosfwd>
#include <vector>
#include <cppad/cg.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main(void) {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    std::vector<ADCG> x(2);
    Independent(x);

    // dependent variable vector
    std::vector<ADCG> y(1);

    // the model equation
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;

    ADFun<CGD> fun(x, y);

    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    // generates source code
    ModelCSourceGen<double> cgen(fun, "model");
    cgen.setCreateJacobian(true);
    cgen.setCreateForwardOne(true);
    cgen.setCreateReverseOne(true);
    cgen.setCreateReverseTwo(true);
    ModelLibraryCSourceGen<double> libcgen(cgen);

    // compile source code
    DynamicModelLibraryProcessor<double> p(libcgen);

    GccCompiler<double> compiler;
    std::unique_ptr<DynamicLib<double>> dynamicLib(p.createDynamicLibrary(compiler) );

    // save to files (not really required)
    SaveFilesModelLibraryProcessor<double> p2(libcgen);
    p2.saveSources();

    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/

    std::unique_ptr<GenericModel<double>> model (dynamicLib->model("model"));
    std::vector<double> xv(x.size());
    xv[0] = 2.5;
    xv[1] = 3.5;
    std::vector<double> jac = model->Jacobian(xv);

    // print out the result
    std::cout << jac[0] << " " << jac[1] << std::endl;
}
