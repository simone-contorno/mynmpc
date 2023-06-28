#ifndef STRUCTS_H
#define STRUCTS_H

#include <mynmpc/utils.h>

/* Problem dimensions. */
struct ProbDim 
{
    size_t dvars;   // Number of decision variables.
    size_t Np;      // Prediction horizon shooting nodes.
    size_t Nc;      // Control horizon shooting horizon.
    size_t n;       // State variables number.
    size_t m;       // Control variables number.
    float dt;       // Step size.
    float T;        // Prediction horizon. 
    size_t n_eq;    // Equalities number.
    size_t n_ineq;  // Inequalities number.
};

/* Model Predictive Control parameters. */
struct MPCParams
{
    MatrixXd x;     // States (along the whole prediction horizon).
    MatrixXd u;     // Controls (along the whole control horizon).
    VectorXd w;     // Slack variable.
    MatrixXd S;     // Final state weight matrix.
    MatrixXd Q;     // Intermediate states weight matrix.
    MatrixXd R;     // Controls weight matrix.
    MatrixXd W;     // Slack variables weight matrix.
};

/* Model informations. */
struct ModelInfo 
{
    std::string name;   // Model name.
    size_t n;           // State vector dimension.
    size_t m;           // Control vector dimension.
    VectorXd params;    // Model parameters.
    VectorXd x;         // State vector.
    VectorXd u;         // Control vector.
    MatrixXd A;         // State matrix.
    MatrixXd B;         // Control matrix.
    VectorXd c;         // Local function approximation. 
};

/* Problem constraints. */
struct Constraints 
{
    std::map<int, std::vector<double>> ineq_x;  // State inequality constraints.
    std::map<int, std::vector<double>> ineq_u;  // Control inequality constraints.
    std::map<int, std::vector<double>> ineq_du; // Acceleration inequality constraints.
    std::map<int, std::vector<double>> ineq_w;  // Slack variable inequality constraints.

    size_t map_idx_x;   
    size_t map_idx_u;
    size_t map_idx_du;
    size_t map_idx_w;

    /* Obstacle avoidance */
    bool obs_flag;      // Obstacle avoidance active/inactive.
    double obs_dist;    // Minimum distance by the closest obstacle.
};

#endif
