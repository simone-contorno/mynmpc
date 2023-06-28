#ifndef PROXQP_H
#define PROXQP_H

#include <mynmpc/utils.h>
#include <mynmpc/model.h>
#include <mynmpc/structs.h>

#include <proxsuite/helpers/optional.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>

using namespace proxsuite::proxqp;

/* ProxQP solver class. */
class ProxQP : public MPCParams
{
public:
    /* Constructor. */
    ProxQP() {} 

    /* Initialization */

    void init(std::shared_ptr<Model> model);             
    
    /* Solving */

    std::tuple<MatrixXd, MatrixXd, VectorXd, proxsuite::proxqp::Info<double>> 
        solve(MatrixXd x, MatrixXd u, VectorXd u_prev, VectorXd w, MatrixXd goal_x, MatrixXd goal_u);

    /* Set */

    void setH();
    void setc(MatrixXd x, MatrixXd u, VectorXd w, MatrixXd goal_x, MatrixXd goal_u);
    void setE(MatrixXd x, MatrixXd u);
    void setb(MatrixXd x, MatrixXd u);
    void setC(MatrixXd x);
    void setd(MatrixXd x, MatrixXd u, VectorXd u_prev, VectorXd w);

    void setQ(MatrixXd Q);
    void setR(MatrixXd R);
    void setS(MatrixXd S);
    void setW(MatrixXd W);
    void setNp(size_t Np);
    void setNc(size_t Nc);
    void setdt(double dt);
    void setNEq(size_t n_eq);
    void setNIneq(size_t n_ineq);
    void setMaxInIter(size_t max_inn_iter);
    void setMaxOutIter(size_t max_out_iter);
    void setQPType(bool qp_type);
    void setGuess(bool guess);

    /* Obstacle avoidance */
    
    void setObs(MatrixXd obs);
    void setObsDim(double obs_l, double obs_w, double obs_pose_l, double obs_pose_w);

private:
    /* Robot model. */
    std::shared_ptr<Model> model;

    /* ProxQP initialization */
    isize qp_dim = 1;   // QP problem dimension (number of decision variables).
    isize qp_eq = 0;    // QP equality constraints number.
    isize qp_ineq = 0;  // QP inequality constraints number.

    sparse::QP<double, isize> qp_sparse = sparse::QP<double, isize>(qp_dim, qp_eq, qp_ineq);    // Sparse solver.
    dense::QP<double> qp_dense = dense::QP<double>(qp_dim, qp_eq, qp_ineq);                     // Dense solver.

    /* Problem configuration */
    MatrixXd H;     // Dense Hessian matrix.
    MatrixXd E;     // Dense equality constraints matrix.
    MatrixXd C;     // Dense inequality constraints matrix.

    Eigen::SparseMatrix<double> H_sparse; // Sparse Hessian matrix.
    Eigen::SparseMatrix<double> E_sparse; // Sparse equality constraints matrix.
    Eigen::SparseMatrix<double> C_sparse; // Sparse inequality constraints matrix.

    VectorXd c;     // Coefficient vector.
    VectorXd b;     // Equality constraints vector. 
    VectorXd upp;   // Inequality constraints upper bounds vector.
    VectorXd low;   // Inequality constraints lower bounds vector.
    
    /* Constraints */
    size_t n_eq;        // Number of equalities.
    size_t n_ineq;      // Number of inequalities.
    VectorXd eq_idx;    // Equalities indeces.
    VectorXd ineq_idx;  // Inequalities indeces.
    size_t eq_tot;      // Total number of equalities.
    size_t ineq_tot;    // Total number of inequalities.
    
    /* Problem dimensions */
    size_t n;   // State dimension.
    size_t m;   // Control dimension.
    size_t Np;  // Prediction horizon shooting nodes.
    size_t Nc;  // Control horizon shooting nodes.
    double dt;  // Step size.

    /* Decision variables */
    size_t n_dvars; // Total number of decision variables.
    size_t x_start; // State starting index.  
    size_t u_start; // Control starting index.
    size_t w_start; // Slack variable starting index.
    
    /* Settings */
    bool qp_type;           // Sparse (false) / dense (true) problem.
    bool guess;             // Use (true) / don't use (false) warm start for initial guesses.
    size_t max_out_iter;    // Maximum number of outer iterations.
    size_t max_inn_iter;    // Maximum number of inner iterations (proximal operator).

    /* Results */
    proxsuite::proxqp::sparse::Vec<double> result_x;        // Optimal decision variables.
    proxsuite::proxqp::sparse::Vec<double> result_lambda;   // Optimal equality Lagrange multipliers.
    proxsuite::proxqp::sparse::Vec<double> result_mu;       // Optimal inequality Lagrange multipliers.
    proxsuite::proxqp::Info<double> qp_info;                // QP information.

    /* Obstacle avoidance */
    MatrixXd obs;                           // Obstacle's goals.
    MatrixXd obs_box;                       // Obstacle's box.
    MatrixXd robot_box;                     // Robot's box.
    double dist_x, dist_y;                  // Distance between robot and obstacle on x.
    double robot_edist, obs_edist, dist;    // Distance between robot and obstacle on y.

    /* Obstacle dimensions */
    double obs_l;       // Obstacle box length.
    double obs_w;       // Obstacle box width.
    double obs_pose_l;  // Obstacle box distance between pose point and back side.
    double obs_pose_w;  // Obstacle box distance between pose point and right side.
};

#endif
