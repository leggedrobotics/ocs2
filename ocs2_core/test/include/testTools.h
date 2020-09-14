#pragma once

#include <ostream>

#include <ocs2_core/Types.h>

namespace ocs2 {

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
inline bool isApprox(const VectorFunctionLinearApproximation& a, const VectorFunctionLinearApproximation& b, scalar_t precision = 1e-9) {
  return a.f.isApprox(b.f, precision) && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
inline std::ostream& operator<<(std::ostream& out, const VectorFunctionLinearApproximation& f) {
  out << "f: " << f.f.transpose() << '\n';
  out << "dfdx:\n" << f.dfdx << '\n';
  out << "dfdu:\n" << f.dfdu << '\n';
  return out;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
inline bool isApprox(const VectorFunctionQuadraticApproximation& a, const VectorFunctionQuadraticApproximation& b,
                     scalar_t precision = 1e-9) {
  if (!(a.f.isApprox(b.f, precision) && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision))) {
    return false;
  }
  for (size_t i = 0; i < a.f.rows(); i++) {
    if (!(a.dfdxx[i].isApprox(b.dfdxx[i], precision) && a.dfdux[i].isApprox(b.dfdux[i], precision) &&
          a.dfduu[i].isApprox(b.dfduu[i], precision))) {
      return false;
    }
  }
  return true;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
inline std::ostream& operator<<(std::ostream& out, const VectorFunctionQuadraticApproximation& f) {
  out << "f: " << f.f.transpose() << '\n';
  out << "dfdx:\n" << f.dfdx << '\n';
  out << "dfdu:\n" << f.dfdu << '\n';
  for (size_t i = 0; i < f.f.rows(); i++) {
    out << "dfdxx[" << i << "]:\n" << f.dfdxx[i] << '\n';
  }
  for (size_t i = 0; i < f.f.rows(); i++) {
    out << "dfdux[" << i << "]:\n" << f.dfdux[i] << '\n';
  }
  for (size_t i = 0; i < f.f.rows(); i++) {
    out << "dfduu[" << i << "]:\n" << f.dfduu[i] << '\n';
  }
  return out;
}

}  // namespace ocs2
