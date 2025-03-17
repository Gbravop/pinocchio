#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"

#include "pinocchio/utils/timer.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>
#include <random>
#include <ctime>

#include <fstream>
#include <math.h>
#include <string>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Code Implementation based on: 
// S. Singh, R. P. Russell and P. M. Wensing, "On Second-Order Derivatives of Rigid-Body Dynamics: Theory and Implementation," in IEEE Transactions on Robotics, vol. 40, pp. 2233-2253, 2024, doi: 10.1109/TRO.2024.3370002.

// already in col major- CHEAP
// get matrix from a tensor along dimension-2 (or keeping second dim constant)
// dim of tens : nxnxn
// dim of mat : nxn
template <typename T>
void get_mat_from_tens2(const Eigen::Tensor<double, 3>& tens, Eigen::MatrixBase<T>& mat, int n, int k)
{
    for (int ii = 0; ii < n; ii++) {
        for (int jj = 0; jj < n; jj++) {
            mat(jj, ii) = tens(jj, k, ii);
        }
    }
}

// This is 30 x faster than row-major one
// changing the tens access and matrix access to col-major
// get matrix from a tensor along dimension-3 (or keeping third dim constant)
template <typename T>
void get_mat_from_tens3_v1(const Eigen::Tensor<double, 3>& tens, Eigen::MatrixBase<T>& mat, int n, int k)
{
    for (int ii = 0; ii < n; ii++) {
        for (int jj = 0; jj < n; jj++) {
            mat(jj, ii) = tens(jj, ii, k);
        }
    }
}

// Assigning code here for assigning a row or a column of the tensor- for Eigen::VectorXd
template <typename T>
void hess_assign(Eigen::Tensor<double, 3>& hess, const Eigen::MatrixBase<T>& vec, const int p, const int q, const int r,
    const int index, const int stride)
{
    if (index == 1) {
        for (int ii = 0; ii < stride; ii++) {
            hess(ii + p, q, r) = vec(ii);
        }

    } else if (index == 2) {
        for (int ii = 0; ii < stride; ii++) {
            hess(p, ii + q, r) = vec(ii);
        }

    } else if (index == 3) {
        for (int ii = 0; ii < stride; ii++) {
            hess(p, q, ii + r) = vec(ii);
        }
    } else {
        std::cout << "No index for hessian_assign provided!" << std::endl;
    }
}

// CHEAP- ~200 mus for 500 link- CAN BE USED
// Inserting a matrix in (along 1st and 3rd dim)
// Assigning code here- for Finite-Diff hessian- along 1st and 3rd dim, 2nd dim constant (input)
// Templated
template <typename T>
void hess_assign_fd2(Eigen::Tensor<double, 3>& hess, const Eigen::MatrixBase<T>& mat, int n, int k)
{
    for (int ii = 0; ii < n; ii++) {
        for (int jj = 0; jj < n; jj++) {
            hess(jj, k, ii) = mat(jj, ii);
        }
    }
}

// Code Implementation based on: 
// S. Singh, R. P. Russell and P. M. Wensing, "On Second-Order Derivatives of Rigid-Body Dynamics: Theory and Implementation," in IEEE Transactions on Robotics, vol. 40, pp. 2233-2253, 2024, doi: 10.1109/TRO.2024.3370002.
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


int main(int argc, char ** argv)
{
    using namespace pinocchio;

    PinocchioTicToc timer(PinocchioTicToc::US);

    // int nl_max = 3;
    int n_runs = 10000;
    
    // Number of links:
    std::list<int> N_list = {1,2,3,4,5,6,7,8,10,13,16,24,29,36,44,54,66,81,100};

    Eigen::MatrixXd T_SO = Eigen::MatrixXd::Zero(n_runs, N_list.size());

    std::cout << "nsz = " << N_list.size() << std::endl;

    std::string urdf_file;
    std::string n_links;
    
    int ctr = 0;
    for (int nl : N_list){

        n_links = std::to_string(nl);

        urdf_file = "urdfs/N" + n_links + "_link_pendulum.urdf";

        if (nl==1 || nl==100){
            std::cout << urdf_file << std::endl;
        }
        
        // URDF file 
        const std::string urdf_filename = urdf_file;
        
        // Load the URDF model
        Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);

        // Build a data related to model
        Data data(model);
        
        // Bounds
        double deg2rad = M_PI/180;
        Eigen::VectorXd qmax(model.nq);
        Eigen::VectorXd qdmax(model.nq);
        Eigen::VectorXd taumax(model.nq);

        for (int i=0; i<model.nq; i++) {
            if (i==0){
                qmax[i] = 170.0;
                qdmax[i] = 98.0;
                taumax[i] = 98.0;
            } else {
                qmax[i] = 120.0;
                qdmax[i] = 98.0;
                taumax[i] = 98.0;
            }
        }

        // std::cout << "HERE+++++++++++++++++" << std::endl;
        
        // ==========================================================================
        // Randomized Inputs;
        PINOCCHIO_ALIGNED_STD_VECTOR(Eigen::VectorXd) q(n_runs);
        PINOCCHIO_ALIGNED_STD_VECTOR(Eigen::VectorXd) v(n_runs);
        PINOCCHIO_ALIGNED_STD_VECTOR(Eigen::VectorXd) tau(n_runs);
        PINOCCHIO_ALIGNED_STD_VECTOR(Eigen::VectorXd) a(n_runs);
        
        for (size_t i = 0; i < n_runs; ++i) {
            q[i] = randomConfiguration(model, -deg2rad*qmax, deg2rad*qmax);
            v[i] = randomConfiguration(model, -deg2rad*qdmax, deg2rad*qdmax);
            tau[i] = randomConfiguration(model, -taumax, taumax);
            a[i] = aba(model, data, q[i], v[i], tau[i], Convention::LOCAL);
            } 
    
        // ==========================================================================
        // Allocate result container - ABA FO Derivatives
        Eigen::MatrixXd daba_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
        Eigen::MatrixXd daba_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
        Eigen::MatrixXd daba_dtau = Eigen::MatrixXd::Zero(model.nv, model.nv);

        // Allocate result container - RNEA SO Derivatives
        Eigen::Tensor<double, 3> dtau2_dqdq(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> dtau2_dvdv(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> dtau2_dqdv(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> dtau2_dadq(model.nv, model.nv, model.nv);
        // Initialize to zero:
        dtau2_dqdq.setZero();
        dtau2_dvdv.setZero();
        dtau2_dqdv.setZero();
        dtau2_dadq.setZero();

        // Allocate result container - ABA SO Derivatives
        Eigen::Tensor<double, 3> daba2_dqdq(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> daba2_dvdv(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> daba2_dqdv(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> daba2_dtaudq(model.nv, model.nv, model.nv);

        Eigen::Tensor<double, 3> prodq(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> prodv(model.nv, model.nv, model.nv);
        Eigen::Tensor<double, 3> prodqdd(model.nv, model.nv, model.nv);

        // Some temp variables here
        Eigen::VectorXd vec1(model.nv);
        Eigen::VectorXd vec2(model.nv);

        Eigen::MatrixXd mat1(model.nv, model.nv);
        Eigen::MatrixXd mat2(model.nv, model.nv);
        Eigen::MatrixXd mat3(model.nv, model.nv);
        Eigen::MatrixXd mat4(model.nv, model.nv);

        Eigen::MatrixXd Minv(Eigen::MatrixXd::Zero(model.nv, model.nv));
        Eigen::MatrixXd Minv_neg(Eigen::MatrixXd::Zero(model.nv, model.nv));
        Eigen::MatrixXd term_in(model.nv, 4 * model.nv * model.nv);  
        Eigen::MatrixXd term_out(model.nv, 4 * model.nv * model.nv);

        // double T_SO[n_runs];
        // for (int i=0; i< (n_runs); i++)
        timer.tic();
        SMOOTH(n_runs){
            //
            auto startTime = std::chrono::high_resolution_clock::now();
            //
            computeABADerivatives(model, data, q[_smooth], v[_smooth], tau[_smooth], daba_dq, daba_dv, daba_dtau);
            // 
            Minv = daba_dtau;
            Minv_neg = -Minv;
            // 
            ComputeRNEASecondOrderDerivatives(model, data, q[_smooth], v[_smooth], a[_smooth], dtau2_dqdq, dtau2_dvdv, dtau2_dqdv, dtau2_dadq);
            //
            // Inner term Compute using DTM (or DMM)
            if (model.nv <= 30) {
                for (int u = 0; u < model.nv; u++) {
                    get_mat_from_tens3_v1(dtau2_dadq, mat1, model.nv, u);
                    for (int w = 0; w < model.nv; w++) {
                        get_mat_from_tens3_v1(dtau2_dadq, mat2, model.nv, w);
                        vec1 = mat1 * daba_dq.col(w);
                        vec2 = mat2 * daba_dv.col(u);
                        hess_assign(prodq, vec1, 0, u, w, 1, model.nv); // slicing a vector in
                        hess_assign(prodv, vec2, 0, w, u, 1, model.nv); // slicing a vector in
                    }
                    mat3.noalias() = mat1 * Minv;
                    hess_assign_fd2(prodqdd, mat3, model.nv, u);
                }
            }
            // Inner term addition using single loop- overall cheaper than double loop inner-term add
            for (int u = 0; u < model.nv; u++) {
                get_mat_from_tens3_v1(dtau2_dqdq, mat1, model.nv, u); // changed
                get_mat_from_tens2(prodq, mat2, model.nv, u);
                get_mat_from_tens3_v1(prodq, mat3, model.nv, u);
                mat1 += mat2 + mat3;
                term_in.middleCols(4 * u * model.nv, model.nv) = mat1;
                // partial w.r.t v
                get_mat_from_tens3_v1(dtau2_dvdv, mat2, model.nv, u); // changed
                term_in.middleCols((4 * u + 1) * model.nv, model.nv) = mat2;
                // partial w.r.t q/v
                get_mat_from_tens3_v1(dtau2_dqdv, mat3, model.nv, u); // changed
                get_mat_from_tens3_v1(prodv, mat2, model.nv, u);
                mat3 += mat2;
                term_in.middleCols((4 * u + 2) * model.nv, model.nv) = mat3;
                // partial w.r.t tau/q
                get_mat_from_tens2(prodqdd, mat1, model.nv, u); // changed
                term_in.middleCols((4 * u + 3) * model.nv, model.nv) = mat1;
            }
            // outer term compute using DTM
            term_out = Minv_neg * term_in; // DMM here with -Minv

            // final assign using double loop- overall cheaper than single loop assign
            for (int u = 0; u < model.nv; u++) {
                for (int w = 0; w < model.nv; w++) {
                    hess_assign(daba2_dqdq, term_out.col(4 * u * model.nv + w), 0, w, u, 1, model.nv);
                    hess_assign(daba2_dvdv, term_out.col((4 * u + 1) * model.nv + w), 0, w, u, 1, model.nv);
                    hess_assign(daba2_dqdv, term_out.col((4 * u + 2) * model.nv + w), 0, w, u, 1, model.nv);
                    hess_assign(daba2_dtaudq, term_out.col((4 * u + 3) * model.nv + w), 0, w, u, 1, model.nv);
                }
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> dt_loop = (endTime - startTime);
            double time_SO_FD = dt_loop.count()*1e6;
            //
            T_SO(_smooth,ctr) = time_SO_FD;
        }
        double time_SO_FDT = timer.toc()/n_runs;
        ctr += 1;
        // ==========================================================================
        // 
        std::cout << "Time SO ABA_T: " + n_links << ": " << time_SO_FDT << std::endl;
    }

// Opening file to save data:
std::ofstream outfile;
std::string filewrite = "time_data_SO_PINOCCHIO_100.csv";
outfile.open(filewrite);

for (int i=0; i<T_SO.rows(); i++){
    for (int j=0; j<T_SO.cols(); j++){
        outfile << T_SO(i,j);
        if (j<T_SO.cols()-1){
            outfile << ",";
        }
    }    
    outfile << "\n";
}
outfile.close();

}
