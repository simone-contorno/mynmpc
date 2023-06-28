#include <mynmpc/proxqp.h>

/*!
 * Initialize ProxQP solver. 
 * @param model model pointer.
 */
void ProxQP::init(std::shared_ptr<Model> model) 
{
    /* Robot model. */
    this->model = model;

    /* Problem dimensions */
    this->n = model->getN();
    this->m = model->getM();

    /* Constraints */
    
    // Equalities
    n_eq += 2;                          // update the number of equalities 
    eq_idx = VectorXd::Zero(n_eq);
    eq_idx(0) = n;                      // first state = initial pose 
    eq_idx(1) = eq_idx(0) + Np*n;       // kinematics
    eq_tot = eq_idx(eq_idx.size()-1); 

    // Inequalities

    /* Update the number of inequalities */ 
    n_ineq += 1;                                    // first control input        
    if (model->getObsFlag() == true) n_ineq += 1;   // obstacle avoidance

    /* Build the inequality indices vector */
    ineq_idx = VectorXd::Zero(n_ineq);       

    /* Set inequality constraint indices */       
    size_t i = 0;             
    
    ineq_idx(i) = m;                                            // first control input velocity 
    i++;
        
    for (size_t j = 0; j < model->getIneq("x").size(); j++)     // state
    {
        ineq_idx(i) = ineq_idx(i-1) + (Np+1);
        i++;
    }

    for (size_t j = 0; j < model->getIneq("u").size(); j++)     // control input
    {
        ineq_idx(i) = ineq_idx(i-1) + Nc;
        i++;
    }

    for (size_t j = 0; j < model->getIneq("du").size(); j++)    // control input derivative
    {
        ineq_idx(i) = ineq_idx(i-1) + (Nc-1);
        i++;
    }
    
    if (model->getObsFlag() == true)                            // obstacle avoidance
    {
        ineq_idx(i) = ineq_idx(i-1) + Np;                       
        i++;

        for (size_t j = 0; j < model->getIneq("w").size(); j++) // slack variables
        {
            ineq_idx(i) = ineq_idx(i-1) + Np;
            i++;
        }
    }

    ineq_tot = ineq_idx(ineq_idx.size()-1);

    /* Decision variables */
    x_start = 0;
    u_start = x_start + (Np+1)*n;
    w_start = u_start + Nc*m;
    n_dvars = (Np+1)*n + Nc*m + (Np+1); 
    
    /* ProxQP */
    H = MatrixXd::Zero(n_dvars, n_dvars);
    E = MatrixXd::Zero(eq_tot, n_dvars);
    C = MatrixXd::Zero(ineq_tot, n_dvars);
    c = VectorXd::Zero(n_dvars); 
    b = VectorXd::Zero(eq_tot); 
    upp = VectorXd::Zero(ineq_tot); 
    low = VectorXd::Zero(ineq_tot); 

    /* Optimal results */
    result_x = VectorXd::Zero(H.cols());
    result_lambda = VectorXd::Zero(E.rows());
    result_mu = VectorXd::Zero(C.rows());

    /* Settings */ 
    setH(); // Hessian matrix
    
    qp_dim = n_dvars;
    qp_eq = eq_tot; 
    qp_ineq = ineq_tot;

    /* Sparse problem */ 
    if (qp_type == false) 
    {
        // Sparse matrices
        H_sparse = H.sparseView();
        E_sparse = E.sparseView();
        C_sparse = C.sparseView();
        
        // QP settings
        qp_sparse = sparse::QP<double, isize>(qp_dim, qp_eq, qp_ineq); 
        qp_sparse.settings.max_iter = max_out_iter; 
        qp_sparse.settings.max_iter_in = max_inn_iter; 
        qp_sparse.settings.compute_timings = true;
    }

    // Dense problem
    else 
    {
        // QP settings
        qp_dense = dense::QP<double>(qp_dim, qp_eq, qp_ineq); 
        qp_dense.settings.max_iter = max_out_iter; 
        qp_sparse.settings.max_iter_in = max_inn_iter; 
        qp_dense.settings.compute_timings = true;
    }
}

/*!
 * Solve the quadratic program using ProxQP solver and compute the optimal state and control input evolution, 
 * resulting in an optimal trajectory.
 * @param x states matrix.
 * @param u control inputs matrix.
 * @param u_prev last control input sent to the vehicle.
 * @param w slack variables vector for obstacle avoidance.
 * @param goal_x state's goals vector.
 * @param goal_u control's goals vector.
*/
std::tuple<MatrixXd, MatrixXd, VectorXd, proxsuite::proxqp::Info<double>> 
    ProxQP::solve(MatrixXd x, MatrixXd u, VectorXd u_prev, VectorXd w, MatrixXd goal_x, MatrixXd goal_u)
{
    /* Configure QP problem */  
    setc(x, u, w, goal_x, goal_u);  // c
    setE(x, u);                     // E
    setb(x, u);                     // b
    setC(x);                        // C
    setd(x, u, u_prev, w);          // d

    /* Sparse problem */
    if (qp_type == false)
    {   
        // Make constraints matrices sparse 
        E_sparse = E.sparseView();
        C_sparse = C.sparseView();

        // QP problem initialization
        qp_sparse.init(H_sparse, c, E_sparse, b, C_sparse, low, upp);
            
        // Solve
        if (guess == true) // warm start
            qp_sparse.solve(result_x, result_lambda, result_mu);
        else 
            qp_sparse.solve();
            
        // Take results
        result_x = qp_sparse.results.x;
        result_lambda = qp_sparse.results.y;
        result_mu = qp_sparse.results.z;

        // Take QP info
        qp_info = qp_sparse.results.info;
    }

    /* Dense problem */
    else 
    {   
        // Initialize the problem 
        qp_dense.init(H, c, E, b, C, low, upp); 
        
        // Solve
        if (guess == true) // warm start
            qp_dense.solve(result_x, result_lambda, result_mu); 
        else
            qp_dense.solve();

        // Take results
        result_x = qp_dense.results.x;
        result_lambda = qp_dense.results.y;
        result_mu = qp_dense.results.z;

        // Take QP info
        qp_info = qp_dense.results.info;
    }

    /* Update decision variables */
    for (size_t i = x_start; i < u_start; i += n) x.row(i/n) = result_x.segment(i,n);           // states
    for (size_t i = u_start; i < w_start; i += m) u.row((i-u_start)/m) = result_x.segment(i,m); // control inputs
    for (size_t i = w_start; i < n_dvars; i++) w(i-w_start) = result_x(i);                      // slack variable

    return {x, u, w, qp_info}; 
}

/* Fill the objective function Hessian matrix H. */
void ProxQP::setH() 
{
    // Intermediate states
    for (size_t i = x_start; i < u_start-n; i += n)   
    { 
        H.block(i,i,n,n) = 2*Q;
    }

    // Final state
    H.block(u_start-n,u_start-n,n,n) = 2*S;        

    // Control inputs
    for (size_t i = u_start; i < w_start-m; i += m)  
    { 
        H.block(i,i,m,m) = 2*R;
    }

    // Slack variable
    if (model->getObsFlag() == true)
    {
        for (size_t i = w_start; i < n_dvars; i++)  
        { 
            H.block(i,i,1,1) = 2*W; 
        }
    }
}

/*! 
 * Fill the objective function coefficients vector c. 
 * @param x states matrix.
 * @param u controls matrix.
 * @param w slack variables vector.
 * @param goal_x state's goals matrix.
 * @param goal_u control's goals matrix.
 */
void ProxQP::setc(MatrixXd x, MatrixXd u, VectorXd w, MatrixXd goal_x, MatrixXd goal_u)
{    
    size_t count = 0;    
    
    // Intermediate states
    for (size_t i = x_start; i < u_start-n; i += n)              
    { 
        c.segment(i,n) = 2*Q*(x.row(count) - goal_x.row(count)).transpose();
        count++; 
    }

    // Final state
    c.segment(u_start-n,n) = 2*S*(x.row(count) - goal_x.row(count)).transpose();

    // Control inputs
    count = 0;
    for (size_t i = u_start; i < w_start-m; i += m)            
    { 
        c.segment(i,m) = 2*R*(u.row(count).transpose() - goal_u.row(count).transpose());
        count++; 
    }

    // Slack variable
    if (model->getObsFlag() == true)
    {
        count = 0;
        for (size_t i = w_start; i < n_dvars; i++)  
        { 
            c.segment(i,1) = 2*W*w(count); 
            count++; 
        }
    }
}

/*! 
 * Fill the equality constraints coefficient matrix E. 
 * @param x states matrix.
 * @param u controls matrix.
 */
void ProxQP::setE(MatrixXd x, MatrixXd u)
{
    /* Initial state */
    for (size_t i = x_start; i < eq_idx(0); i += n) 
    { 
        E.block(i,i,n,n) = MatrixXd::Identity(n,n);
    }
   
    /* Model kinematics (Euler) */
    size_t count = 0;
    size_t col; 
    for (size_t i = eq_idx(0); i < eq_idx(1); i += n) 
    { 
        // Update for the current step 
        model->setX(x.row(count));  
        model->setU(u.row(std::clamp((int)(count), 0, (int)(u.rows()-1))));

        model->updateA(dt);
        model->updateB();

        // State
        E.block(i,i-eq_idx(0),n,n) = model->getA();                 // current
        E.block(i,i-eq_idx(0)+n,n,n) = -MatrixXd::Identity(n,n);    // next

        // Control input
        col = u_start+(i-eq_idx(0))/n*m;
        E.block(i,std::clamp((int)(col), 0, (int)(w_start-m)),n,m) = model->getB() * dt;

        count++;
    }
}

/*! 
 * Fill the equality constraints vector b. 
 * @param x states matrix.
 * @param u controls matrix.
 */
void ProxQP::setb(MatrixXd x, MatrixXd u) 
{   
    size_t count = 0;

    /* Model kinematics */
    for (size_t i = eq_idx(0); i < eq_idx(1); i += n)
    {
        // Update for the current step 
        model->setX(x.row(count));                                          // state vector
        model->setU(u.row(std::clamp((int)(count), 0, (int)(u.rows()-1)))); // control input
        model->updatec(dt, x.row(count+1));                                 // local function approximation.
        
        b.segment(i,n) = -model->getc();

        count++;
    }
}

/*! 
 * Fill the inequality constraints coefficients matrix C. 
 * @param x states matrix.
 */
void ProxQP::setC(MatrixXd x)
{
    size_t i = 0;
    size_t idx;
    size_t col;

    /* First control input's constraint */
    for (size_t j = 0; j < model->getIneq("du").size(); j++) 
        C(j,u_start+j) = 1;

    /* States' constraints */
    for (size_t j = 0; j < model->getIneq("x").size(); j++) // for each state inequality constraint
    {
        for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
        {
            idx = x_start + model->getIneq("x")[j][0] + n * (l-ineq_idx(i));
            C(l,idx) = 1;
        }
        i++;
    }

    /* Control inputs' constraints */
    for (size_t j = 0; j < model->getIneq("u").size(); j++) // for each control inequality constraint
    {
        for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
        {
            idx = u_start + model->getIneq("u")[j][0] + m * (l-ineq_idx(i));
            C(l,idx) = 1;
        }
        i++;
    }

    /* Control inputs derivatives' constraints */
    for (size_t j = 0; j < model->getIneq("du").size(); j++) // for each control derivative inequality constraint
    {
        for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
        {
            idx = u_start + model->getIneq("du")[j][0] + m * (l-ineq_idx(i));
            C(l,idx) = -1;
            C(l,idx+m) = 1;
        }
        i++;
    }

    /* Obstacle avoidance */
    if (model->getObsFlag() == true)
    {    
        for (size_t j = ineq_idx(i); j < ineq_idx(i+1); j++) 
        {
            /* Indeces */
            idx = j-ineq_idx(i);
            col = x_start + n * (idx+1);
            
            /* Robot box computation */  
            robot_box = computeBox( x.row(idx), 
                                    model->getBoxLength(), 
                                    model->getBoxWidth(), 
                                    model->getPoseLength(), 
                                    model->getPoseWidth(), 
                                    model->getBoxPoints()); 
             
            obs_box = computeBox(   obs.row((std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 0)),
                                    obs_l, 
                                    obs_w, 
                                    obs_pose_l, 
                                    obs_pose_w, 
                                    model->getBoxPoints()); 

            /* Compute the closest point */
            auto [robot_idx, obs_idx] = closestBoxPoints(robot_box, obs_box); {}

            /* Compute distance on x and y */
            dist_x = x(idx, 0) - obs(std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 0);
            dist_y = x(idx, 1) - obs(std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 1);
            
            /* Fill the matrix with the obstacle avoidance constraint */
            C(j,col) = dist_x / sqrt(pow(dist_x, 2) + pow(dist_y, 2));      // x
            C(j,col+1) = dist_y / sqrt(pow(dist_x, 2) + pow(dist_y, 2));    // y
            C(j,w_start+idx+1) = 1;                                         // w
        }
        i++;

        /* Slack variable */
        for (size_t j = 0; j < model->getIneq("w").size(); j++) // for each slack variable inequality constraint
        {
            for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
            {
                idx = w_start + model->getIneq("w")[j][0] + 1 * (l-ineq_idx(i)); 
                C(l,idx+1) = 1;
            }
            i++;
        }
    }   
}

/*! 
 * Fill the inequality constraints vector d. 
 * @param x states matrix.
 * @param u controls matrix.
 * @param u_prev last control input sent to the vehicle.
 * @param w slack variables vector.
 */
void ProxQP::setd(MatrixXd x, MatrixXd u, VectorXd u_prev, VectorXd w)
{   
    size_t i = 0;
    size_t row;
    size_t col;
    size_t idx;

    /* First control input's bounds */
    for (size_t j = 0; j < model->getIneq("du").size(); j++) // first control input bounds
    {      
        idx = model->getIneq("du")[j][0];
        low(j) = model->getIneq("du")[j][1]*dt + u_prev(idx) - u(0, idx);
        upp(j) = model->getIneq("du")[j][2]*dt + u_prev(idx) - u(0, idx);
    }

    /* States' bounds */
    for (size_t j = 0; j < model->getIneq("x").size(); j++) // for each state inequality constraint
    {
        for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
        {
            row = l-ineq_idx(i);
            col = model->getIneq("x")[j][0];
            low(l) = model->getIneq("x")[j][1] - x(row,col);
            upp(l) = model->getIneq("x")[j][2] - x(row,col);
        }
        i++;
    }

    /* Control inputs' bounds */
    for (size_t j = 0; j < model->getIneq("u").size(); j++) // for each control inequality constraint
    {
        for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
        {
            row = l-ineq_idx(i);
            col = model->getIneq("u")[j][0];
            low(l) = model->getIneq("u")[j][1] - u(row,col);
            upp(l) = model->getIneq("u")[j][2] - u(row,col);
        }
        i++;
    }

    /* Control inputs derivatives' bounds */
    for (size_t j = 0; j < model->getIneq("du").size(); j++) // for each control derivative inequality constraint
    {
        for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
        {
            row = l-ineq_idx(i);
            col = model->getIneq("du")[j][0];
            
            low(l) = model->getIneq("du")[j][1]*dt - u(row+1,col) + u(row,col);
            upp(l) = model->getIneq("du")[j][2]*dt - u(row+1,col) + u(row,col);
        }
        i++;
    }

    /* Obstacle avoidance */
    if (model->getObsFlag() == true)
    {
        /* Obstacle distance */
        for (size_t j = ineq_idx(i); j < ineq_idx(i+1); j++) // obstacle distance
        {      
            idx = j-ineq_idx(i);

            /* Robot box computation */      
            robot_box = computeBox( x.row(idx), 
                                    model->getBoxLength(), 
                                    model->getBoxWidth(), 
                                    model->getPoseLength(), 
                                    model->getPoseWidth(), 
                                    model->getBoxPoints());
    
            obs_box = computeBox(   obs.row((std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 0)),
                                    obs_l, 
                                    obs_w, 
                                    obs_pose_l, 
                                    obs_pose_w, 
                                    model->getBoxPoints()); 

            /* Compute the closest point between*/
            auto [robot_idx, obs_idx] = closestBoxPoints(robot_box, obs_box); {}

            /* Compute distance on x and y */
            dist_x = x(idx, 0) - obs(std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 0);
            dist_y = x(idx, 1) - obs(std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 1);

            /* Compute euclidean distance */
            robot_edist = sqrt(pow(robot_box(robot_idx, 0) - x(idx, 0),2) + pow(robot_box(robot_idx, 1) - x(idx, 1),2));
            obs_edist = sqrt(pow(obs_box(obs_idx, 0) - obs(std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 0),2) 
                                + pow(obs_box(obs_idx, 1) - obs(std::clamp((int)(idx), 0, (int)(obs.rows()-1)), 1),2));
            dist = model->getObsDist() + robot_edist + obs_edist;

            low(j) = dist - sqrt(pow(dist_x, 2) + pow(dist_y, 2)) - w(idx);
            upp(j) = std::numeric_limits<double>::infinity();
        }
        i++;

        /* Slack variables' bounds */
        for (size_t j = 0; j < model->getIneq("w").size(); j++) // for each slack variable inequality constraint
        {
            for (size_t l = ineq_idx(i); l < ineq_idx(i+1); l++) // for each step 
            {
                row = l-ineq_idx(i);
                low(l) = model->getIneq("w")[j][1] - w(row);
                upp(l) = model->getIneq("w")[j][2] - w(row);
            }
            i++;
        }
    }
}

/*!
 * Set the intermediate states weight matrix.
 * @param Q weight matrix.
 */
void ProxQP::setQ(MatrixXd Q) {this->Q = Q;}

/*!
 * Set the control input weight matrix.
 * @param R weight matrix.
 */
void ProxQP::setR(MatrixXd R) {this->R = R;}

/*!
 * Set the final state weight matrix.
 * @param S weight matrix.
 */
void ProxQP::setS(MatrixXd S) {this->S = S;}

/*!
 * Set the slack variables weight matrix.
 * @param W weight matrix.
 */
void ProxQP::setW(MatrixXd W) {this->W = W;}

/*!
 * Set the number of shooting nodes (state).
 * If T is set, dt is automatically updated.
 * @param Np shooting nodes (> 0).
 */
void ProxQP::setNp(size_t Np) 
{
    assert(Np > 0);
    this->Np = Np;
}

/*!
 * Set the number of shooting nodes (control).
 * @param Nc shooting nodes (> 0).
 */
void ProxQP::setNc(size_t Nc) 
{
    assert(Nc > 0);
    this->Nc = Nc;
}

/*!
 * Set the step size.
 * @param dt step size (> 0).
 */
void ProxQP::setdt(double dt) 
{
    assert(dt > 0.0);
    this->dt = dt;
}

/*!
 * Set the number of equalities.
 * @param n_eq equalities (>= 0).
 */
void ProxQP::setNEq(size_t n_eq) 
{
    assert(n_eq >= 0);
    this->n_eq = n_eq;
}

/*!
 * Set the number of inequalities.
 * @param n_ineq inequalities (>= 0).
 */
void ProxQP::setNIneq(size_t n_ineq) 
{
    assert(n_ineq >= 0);
    this->n_ineq = n_ineq;
}

/*!
 * Set the maximum number of inner iterations.
 * @param max_inn_iter maximum iteration (> 0).
 */
void ProxQP::setMaxInIter(size_t max_inn_iter) 
{
    assert(max_inn_iter > 0);
    this->max_inn_iter = max_inn_iter;
}

/*!
 * Set the maximum number of outer iterations.
 * @param max_out_iter maximum iteration (> 0).
 */
void ProxQP::setMaxOutIter(size_t max_out_iter) 
{
    assert(max_out_iter > 0);
    this->max_out_iter = max_out_iter;
}

/*!
 * Set the QP type (sparse or dense).
 * @param qp_type sparse (false) or dense (true).
 */
void ProxQP::setQPType(bool qp_type) {this->qp_type = qp_type;}

/*!
 * Choose if using initial guesses for warm start.
 * @param guess no (false) or yes (true).
 */
void ProxQP::setGuess(bool guess) {this->guess = guess;}

/*! 
 * Set desired goals for obstacle avoidance.
 * @param obs goals matrix.
 */
void ProxQP::setObs(MatrixXd obs) {this->obs = obs;}

/*! 
 * Set obstacle box dimensions.
 * @param obs_l obstacle box length (>= 0).
 * @param obs_w obstacle box width (>= 0).
 * @param obs_pose_l obstacle pose distance by back side (>= 0).
 * @param obs_pose_w obstacle pose distance by right side (>= 0).
 */
void ProxQP::setObsDim(double obs_l, double obs_w, double obs_pose_l, double obs_pose_w) 
{
    assert(obs_l >= 0.);
    assert(obs_w >= 0.);
    assert(obs_pose_l >= 0.);
    assert(obs_pose_w >= 0.);

    this->obs_l = obs_l;
    this->obs_w = obs_w;
    this->obs_pose_l = obs_pose_l;
    this->obs_pose_w = obs_pose_w;
}
