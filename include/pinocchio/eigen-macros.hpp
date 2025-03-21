//
// Copyright (c) 2017-2024 CNRS INRIA
//

#ifndef __pinocchio_eigen_macros_hpp__
#define __pinocchio_eigen_macros_hpp__

#include "pinocchio/utils/eigen-fix.hpp"

/// \brief Macro giving access to the equivalent plain type of D
#define PINOCCHIO_EIGEN_PLAIN_TYPE(D)                                                              \
  Eigen::internal::plain_matrix_type<typename pinocchio::helper::argument_type<void(D)>::type>::type
#define PINOCCHIO_EIGEN_PLAIN_TYPE_NO_PARENS(D)                                                    \
  Eigen::internal::plain_matrix_type<typename pinocchio::helper::argument_type<void D>::type>::type

/// \brief Similar to macro PINOCCHIO_EIGEN_PLAIN_TYPE but with guaranty to provite a column major
/// type
#define PINOCCHIO_EIGEN_PLAIN_COLUMN_MAJOR_TYPE(D)                                                 \
  pinocchio::helper::handle_return_type_without_typename<                                          \
    D, Eigen::internal::plain_matrix_type_column_major>::type

/// \brief Similar to macro PINOCCHIO_EIGEN_PLAIN_TYPE but with guaranty to provite a row major type
#define PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(D)                                                    \
  pinocchio::helper::handle_return_type_without_typename<                                          \
    D, ::pinocchio::fix::Eigen::internal::plain_matrix_type_row_major>::type

/// \brief Macro giving access to the reference type of D
#define PINOCCHIO_EIGEN_REF_CONST_TYPE(D) Eigen::internal::ref_selector<D>::type
#if EIGEN_VERSION_AT_LEAST(3, 2, 90)
  #define PINOCCHIO_EIGEN_REF_TYPE(D) Eigen::internal::ref_selector<D>::non_const_type
#else
  #define PINOCCHIO_EIGEN_REF_TYPE(D)                                                              \
    Eigen::internal::conditional<                                                                  \
      bool(Eigen::internal::traits<D>::Flags & Eigen::NestByRefBit), D &, D>::type
#endif

/// \brief Macro giving access to the return type of the dot product operation
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
  #define PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(D1, D2)                                          \
    Eigen::ScalarBinaryOpTraits<                                                                   \
      typename Eigen::internal::traits<D1>::Scalar,                                                \
      typename Eigen::internal::traits<D2>::Scalar>::ReturnType
#else
  #define PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(D1, D2)                                          \
    Eigen::internal::scalar_product_traits<                                                        \
      typename Eigen::internal::traits<D1>::Scalar,                                                \
      typename Eigen::internal::traits<D2>::Scalar>::ReturnType
#endif

/// \brief Macro for an automatic const_cast
#define PINOCCHIO_EIGEN_CONST_CAST(TYPE, OBJ) const_cast<TYPE &>(OBJ.derived())

///  \brief Tell if Pinocchio should use the Eigen Tensor Module or not
#if EIGEN_VERSION_AT_LEAST(3, 2, 90)
  #define PINOCCHIO_WITH_EIGEN_TENSOR_MODULE
#endif

/// \brief Check memory allocation for Eigen.
/// \warning These macros do *not* work well with multithreading for Eigen <= 3.4
/// and *will* create a race condition - special care is required.

#ifdef PINOCCHIO_EIGEN_CHECK_MALLOC
  #define PINOCCHIO_EIGEN_MALLOC(allowed) ::Eigen::internal::set_is_malloc_allowed(allowed)
  #define PINOCCHIO_EIGEN_MALLOC_ALLOWED() PINOCCHIO_EIGEN_MALLOC(true)
  #define PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED() PINOCCHIO_EIGEN_MALLOC(false)
  #define PINOCCHIO_EIGEN_MALLOC_SAVE_STATUS() ::pinocchio::internal::save_eigen_malloc_status()
  #define PINOCCHIO_EIGEN_MALLOC_RESTORE_STATUS()                                                  \
    PINOCCHIO_EIGEN_MALLOC((::pinocchio::internal::get_saved_eigen_malloc_status()))
#else
  #define PINOCCHIO_EIGEN_MALLOC(allowed)
  #define PINOCCHIO_EIGEN_MALLOC_ALLOWED()
  #define PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED()
  #define PINOCCHIO_EIGEN_MALLOC_SAVE_STATUS()
  #define PINOCCHIO_EIGEN_MALLOC_RESTORE_STATUS()
#endif

#ifdef PINOCCHIO_EIGEN_CHECK_MALLOC
namespace pinocchio
{
  namespace internal
  {

    inline bool save_or_get_malloc_status(bool update, bool new_value = false)
    {
      thread_local static bool value;
      if (update)
        value = new_value;
      return value;
    }

    inline void save_eigen_malloc_status()
    {
      save_or_get_malloc_status(true, ::Eigen::internal::is_malloc_allowed());
    }

    inline bool get_saved_eigen_malloc_status()
    {
      return save_or_get_malloc_status(false);
    }
  } // namespace internal
} // namespace pinocchio
#endif // ifdef PINOCCHIO_EIGEN_CHECK_MALLOC

#endif // ifndef __pinocchio_eigen_macros_hpp__
