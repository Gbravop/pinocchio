//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_context_hpp__
#define __pinocchio_context_hpp__

#include <Eigen/Core>
#include "pinocchio/container/aligned-vector.hpp"


namespace pinocchio {

  template<typename _Scalar, int _Options> struct JointCollectionDefaultTpl;
  template<typename _Scalar, int _Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct ModelTpl;
  template<typename _Scalar, int _Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct DataTpl;
  template<typename _Scalar, int _Options = 0> class MotionTpl;
  template<typename _Scalar, int _Options = 0> class ForceTpl;

  template<typename _Scalar, int _Options> struct RigidConstraintModelTpl;
  template<typename _Scalar, int _Options> struct RigidConstraintDataTpl;

  typedef RigidConstraintModelTpl<double,0> RigidConstraintModel;
  typedef RigidConstraintDataTpl<double,0> RigidConstraintData;

  namespace context {
    typedef PINOCCHIO_SCALAR_TYPE Scalar;
    enum { Options = 0 };
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> Matrix6xs;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor | Options> RowMatrixXs;
    typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic,Options> Matrix3x;
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,10,Options> BodyRegressorType;

    typedef ModelTpl<Scalar, Options> Model;     
    typedef DataTpl<Scalar, Options> Data;

    typedef RigidConstraintModelTpl<Scalar,Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar,Options> RigidConstraintData;

    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) RigidConstraintModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) RigidConstraintDataVector;

    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;

  } //namespace context

} //namespace pinocchio

#endif // #ifndef __pinocchio_context_hpp__
