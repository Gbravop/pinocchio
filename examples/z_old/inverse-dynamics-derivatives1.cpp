#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"

#include "pinocchio/utils/timer.hpp"
// #include "pinocchio/utils/tensor_utils.hpp"

#include <iostream>
#include <math.h>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char ** argv)
{
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);

  // You should change here to set up your own URDF file or just pass it as an argument of this
  // example.
const std::string urdf_filename = "urdfs/iiwa.urdf";
//   const std::string urdf_filename =
//     (argc <= 1) ? PINOCCHIO_MODEL_DIR
//                     + std::string("/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf")
//                 : argv[1];

  // Load the URDF model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  // Build a data related to model
  Data data(model);
  // Sample a random joint configuration as well as random joint velocity and acceleration
  // Bounds
  double deg2rad = M_PI/180;
  
  // Eigen::VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  // Eigen::VectorXd qdmax = Eigen::VectorXd::Ones(model.nq);
  // Eigen::VectorXd taumax = Eigen::VectorXd::Ones(model.nq);
  
  Eigen::VectorXd qmax(model.nq);
  Eigen::VectorXd qdmax(model.nq);
  Eigen::VectorXd taumax(model.nq);

  qmax << 170.0,120.0,170.0,120.0,170.0,120.0,175.0;
  qdmax << 98.0,98.0,100.0,130.0,140.0,180.0,180.0;
  taumax << 98.0,98.0,100.0,130.0,140.0,180.0,180.0;
  
  // Eigen::VectorXd q = randomConfiguration(model, -deg2rad*170*qmax, deg2rad*170*qmax);
  // Eigen::VectorXd q2 = randomConfiguration(model, -deg2rad*170*qmax, deg2rad*170*qmax);
  // Eigen::VectorXd v = randomConfiguration(model, -deg2rad*180*qdmax, deg2rad*180*qdmax);
  // Eigen::VectorXd tau = randomConfiguration(model, -180*taumax, 180*taumax);

  Eigen::VectorXd q = randomConfiguration(model, -deg2rad*qmax, deg2rad*qmax);
  Eigen::VectorXd q2 = randomConfiguration(model, -deg2rad*qmax, deg2rad*qmax);
  Eigen::VectorXd v = randomConfiguration(model, -deg2rad*qdmax, deg2rad*qdmax);
  Eigen::VectorXd tau = randomConfiguration(model, -taumax, taumax);
  
  // Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  Convention rf = Convention::LOCAL;
  Eigen::VectorXd a = aba(model, data, q, v, tau, rf);

  // ==========================================================================
  // Allocate result container - ABA FO Derivatives
  Eigen::MatrixXd dFD_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
  Eigen::MatrixXd dFD_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
  Eigen::MatrixXd dFD_dtau = Eigen::MatrixXd::Zero(model.nv, model.nv);

  timer.tic();
  // Computes the forward dynamics (ABA) derivatives for all the joints of the robot
  computeABADerivatives(
    model, data, q, v, tau, dFD_dq, dFD_dv, dFD_dtau);

  // Get the running time
  double time_FO_FD = timer.toc();
  std::cout << "Run Time ABA FO derivatives = " << time_FO_FD << std::endl;

  // ==========================================================================
  // Allocate result container - RNEA FO Derivatives
  Eigen::MatrixXd dID_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
  Eigen::MatrixXd dID_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
  Eigen::MatrixXd dID_da = Eigen::MatrixXd::Zero(model.nv, model.nv);

  timer.tic();
  // Computes the inverse dynamics (RNEA) derivatives for all the joints of the robot
  computeRNEADerivatives(
    model, data, q, v, a, dID_dq, dID_dv, dID_da);

  // Get the running time
  double time_FO_ID = timer.toc();
  std::cout << "Run Time RNEA FO derivatives = " << time_FO_ID << std::endl;

  // ==========================================================================
  // Allocate result container - RNEA SO Derivatives
  Eigen::Tensor<double, 3> d2tau_dqdq(model.nv, model.nv, model.nv);
  Eigen::Tensor<double, 3> d2tau_dvdv(model.nv, model.nv, model.nv);
  Eigen::Tensor<double, 3> d2tau_dqdv(model.nv, model.nv, model.nv);
  Eigen::Tensor<double, 3> d2tau_dadq(model.nv, model.nv, model.nv);
  // Initialize to zero:
  d2tau_dqdq.setZero();
  d2tau_dvdv.setZero();
  d2tau_dqdv.setZero();
  d2tau_dadq.setZero();

  timer.tic();
  // Computes the inverse dynamics (RNEA) derivatives for all the joints of the robot
  ComputeRNEASecondOrderDerivatives(
    model, data, q, v, a, d2tau_dqdq, d2tau_dvdv, d2tau_dqdv, d2tau_dadq);

  // Get the running time
  double time_SO_ID = timer.toc();
  std::cout << "Run Time RNEA SO derivatives = " << time_SO_ID << std::endl;

  // ==========================================================================

  // Get access to the joint torque
  std::cout << "Joint angle: " << q.transpose() << std::endl;
  std::cout << "Joint angle: " << q2.transpose() << std::endl;
  std::cout << "Joint velocity: " << v.transpose() << std::endl;
  std::cout << "Joint acceleration: " << a.transpose() << std::endl;
  // std::cout << "Joint qdd a: " << qdd.transpose() << std::endl;
  std::cout << "Joint qdd b: " << data.ddq.transpose() << std::endl;
  std::cout << "Joint torque a: " << tau.transpose() << std::endl;
  std::cout << "Joint torque b: " << data.tau.transpose() << std::endl;
}
