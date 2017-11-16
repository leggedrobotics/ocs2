/*
 * eigenIntegration.h
 *
 *  Created on: Jun 22, 2015
 *      Author: farbod
 */

/*
  [auto_generated]
  boost/numeric/odeint/external/eigen/eigen_algebra.hpp

  [begin_description]
  tba.
  [end_description]

  Copyright 2013 Christian Shelton
  Copyright 2013 Karsten Ahnert

  Distributed under the Boost Software License, Version 1.0.
  (See accompanying file LICENSE_1_0.txt or
  copy at http://www.boost.org/LICENSE_1_0.txt)
*/


#ifndef BOOST_NUMERIC_ODEINT_EXTERNAL_EIGEN_EIGEN_ALGEBRA_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_EXTERNAL_EIGEN_EIGEN_ALGEBRA_HPP_INCLUDED

#include <Eigen/Dense>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

// Necessary routines for Eigen matrices to work with vector_space_algebra
// from odeint
// (that is, it lets odeint treat the Eigen matrices correctly, knowing
// how to add, multiply, compute the norm, etc)

namespace Eigen {

#if EIGEN_VERSION_AT_LEAST(3,3,0)
namespace internal{
template<typename Scalar>
struct scalar_add_op {
  // FIXME default copy constructors seems bugged with std::complex<>
  EIGEN_DEVICE_FUNC inline scalar_add_op(const scalar_add_op& other) : m_other(other.m_other) { }
  EIGEN_DEVICE_FUNC inline scalar_add_op(const Scalar& other) : m_other(other) { }
  EIGEN_DEVICE_FUNC inline Scalar operator() (const Scalar& a) const { return a + m_other; }
  template <typename Packet>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return internal::padd(a, pset1<Packet>(m_other)); }
  const Scalar m_other;
};
template<typename Scalar>
struct functor_traits<scalar_add_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::AddCost, PacketAccess = packet_traits<Scalar>::HasAdd }; };

} // namespace internal
#endif


template<typename D>
inline const
typename Eigen::CwiseUnaryOp<
          typename Eigen::internal::scalar_add_op<
               typename Eigen::internal::traits<D>::Scalar>,
          const D >
operator+(const typename Eigen::MatrixBase<D> &m,
          const typename Eigen::internal::traits<D>::Scalar &s) {
     return Eigen::CwiseUnaryOp<
          typename Eigen::internal::scalar_add_op<
               typename Eigen::internal::traits<D>::Scalar>,
          const D >(m.derived(),Eigen::internal::scalar_add_op<
                    typename Eigen::internal::traits<D>::Scalar>(s));
}

template<typename D>
inline const
typename Eigen::CwiseUnaryOp<
          typename Eigen::internal::scalar_add_op<
               typename Eigen::internal::traits<D>::Scalar>,
          const D >
operator+(const typename Eigen::internal::traits<D>::Scalar &s,
const typename Eigen::MatrixBase<D> &m) {
     return Eigen::CwiseUnaryOp<
          typename Eigen::internal::scalar_add_op<
               typename Eigen::internal::traits<D>::Scalar>,
          const D >(m.derived(),Eigen::internal::scalar_add_op<
                    typename Eigen::internal::traits<D>::Scalar>(s));
}



template<typename D1,typename D2>
inline const
typename Eigen::CwiseBinaryOp<
    typename Eigen::internal::scalar_quotient_op<
        typename Eigen::internal::traits<D1>::Scalar>,
    const D1, const D2>
operator/(const Eigen::MatrixBase<D1> &x1, const Eigen::MatrixBase<D2> &x2) {
    return x1.cwiseQuotient(x2);
}


template< typename D >
inline const
typename Eigen::CwiseUnaryOp<
    typename Eigen::internal::scalar_abs_op<
        typename Eigen::internal::traits< D >::Scalar > ,
        const D >
abs( const Eigen::MatrixBase< D > &m ) {
    return m.cwiseAbs();
}



} // end Eigen namespace



namespace boost {
namespace numeric {
namespace odeint {

template<int STATE_DIM>
struct vector_space_reduce
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template< class Op >
  double operator()(const Eigen::Matrix<double, STATE_DIM, 1>& x , Op op , double init ) const
  {
	  for (int i=0; i<STATE_DIM; i++)
	  {
		  init = op( init , x(i) );
	  }
      return init;
  }
};


} } } // end boost::numeric::odeint namespace


#endif // BOOST_NUMERIC_ODEINT_EXTERNAL_EIGEN_EIGEN_ALGEBRA_HPP_INCLUDED
