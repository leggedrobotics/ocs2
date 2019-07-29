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

#ifndef BOOST_NUMERIC_ODEINT_EXTERNAL_EIGEN_EIGEN_ALGEBRA_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_EXTERNAL_EIGEN_EIGEN_ALGEBRA_HPP_INCLUDED

#include <Eigen/Dense>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

// Necessary routines for Eigen matrices to work with vector_space_algebra
// from odeint
// (that is, it lets odeint treat the Eigen matrices correctly, knowing
// how to add, multiply, compute the norm, etc)

namespace Eigen {

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
namespace internal {
template <typename Scalar>
struct scalar_add_op {
  // FIXME default copy constructors seems bugged with std::complex<>
  EIGEN_DEVICE_FUNC inline scalar_add_op(const scalar_add_op& other) : m_other(other.m_other) {}
  EIGEN_DEVICE_FUNC inline scalar_add_op(const Scalar& other) : m_other(other) {}
  EIGEN_DEVICE_FUNC inline Scalar operator()(const Scalar& a) const { return a + m_other; }
  template <typename Packet>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const {
    return internal::padd(a, pset1<Packet>(m_other));
  }
  const Scalar m_other;
};
template <typename Scalar>
struct functor_traits<scalar_add_op<Scalar>> {
  enum { Cost = NumTraits<Scalar>::AddCost, PacketAccess = packet_traits<Scalar>::HasAdd };
};

}  // namespace internal
#endif

template <typename D>
inline const typename Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>
operator+(const typename Eigen::MatrixBase<D>& m, const typename Eigen::internal::traits<D>::Scalar& s) {
  return Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>(
      m.derived(), Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>(s));
}

template <typename D>
inline const typename Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>
operator+(const typename Eigen::internal::traits<D>::Scalar& s, const typename Eigen::MatrixBase<D>& m) {
  return Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>(
      m.derived(), Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>(s));
}

template <typename D1, typename D2>
inline const typename Eigen::CwiseBinaryOp<typename Eigen::internal::scalar_quotient_op<typename Eigen::internal::traits<D1>::Scalar>,
                                           const D1, const D2>
operator/(const Eigen::MatrixBase<D1>& x1, const Eigen::MatrixBase<D2>& x2) {
  return x1.cwiseQuotient(x2);
}

template <typename D>
inline const typename Eigen::CwiseUnaryOp<typename Eigen::internal::scalar_abs_op<typename Eigen::internal::traits<D>::Scalar>, const D>
abs(const Eigen::MatrixBase<D>& m) {
  return m.cwiseAbs();
}

}  // namespace Eigen

namespace boost {
namespace numeric {
namespace odeint {

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

// new boost
template <int S1, int S2, int O, int M1, int M2>
struct vector_space_norm_inf<Eigen::Matrix<double, S1, S2, O, M1, M2>> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using result_type = double;
  result_type operator()(const Eigen::Matrix<double, S1, S2, O, M1, M2>& m) const { return m.template lpNorm<Eigen::Infinity>(); }
};

#else

// old boost
template <int STATE_DIM>
struct vector_space_reduce<Eigen::Matrix<double, STATE_DIM, 1>> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template <class Op>
  double operator()(const Eigen::Matrix<double, STATE_DIM, 1>& x, Op op, double init) const {
    for (int i = 0; i < STATE_DIM; i++) {
      init = op(init, x(i));
    }
    return init;
  }
};

#endif
}  // namespace odeint
}  // namespace numeric
}  // namespace boost

#endif  // BOOST_NUMERIC_ODEINT_EXTERNAL_EIGEN_EIGEN_ALGEBRA_HPP_INCLUDED
