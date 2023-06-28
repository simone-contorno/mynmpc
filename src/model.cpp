#include <mynmpc/model.h>

/********************************************************************************/
/************************************* Get **************************************/
/********************************************************************************/

/* Get the model name. */
std::string Model::getName() {return name;}

/* Get the state dimension. */
size_t Model::getN() {return n;}

/* Get the control input dimension. */
size_t Model::getM() {return m;}

/* Get the model parameters. */
VectorXd Model::getParams() {return params;}

/* Get the state vector. */
VectorXd Model::getX() {return x;}

/* Get the control input vector. */
VectorXd Model::getU() {return u;}

/* Get the local function approximation.. */
VectorXd Model::getc() {return c;}

/* Get the state matrix. */
MatrixXd Model::getA() {return A;}

/* Get the control matrix. */
MatrixXd Model::getB() {return B;}

/*! 
 * Get inequality constraints. 
 * @param var "x", "u", "du" or "w" respectively for state, control, control derivative and slack variable.
 */
std::map<int, std::vector<double>> Model::getIneq(std::string var) 
{
    assert(var == "x" || var == "u" || var == "du" || var == "w"); // check that the variable is 'x', 'u', 'du' or 'w' 
    if (var == "x") return ineq_x;
    else if (var == "u") return ineq_u;
    else if (var == "du") return ineq_du;
    else if (var == "w") return ineq_w; 
}

/* Get obstacle avoidance flag. */
bool Model::getObsFlag() {return obs_flag;}

/* Get obstacle avoidance minimum distance. */
double Model::getObsDist() {return obs_dist;}

/* Get the model box width. */
double Model::getBoxWidth() {return width;}

/* Get the model box length. */
double Model::getBoxLength() {return length;}

/* Get the pose distance from the model box right side. */
double Model::getPoseWidth() {return pose_width;}

/* Get the pose distance from the model box back side. */
double Model::getPoseLength() {return pose_length;}

/* Get the number of box points for each side. */
double Model::getBoxPoints() {return points;}

/********************************************************************************/
/************************************* Set **************************************/
/********************************************************************************/

/*! 
 * Set the model name. 
 * @param name name.
 */
void Model::setName(std::string name) {this->name = name;}

/*! 
 * Set the state vector dimension. 
 * @param n dimension (> 0).
 */
void Model::setN(size_t n) 
{
    assert(n > 0);
    this->n = n;
    this->x = VectorXd::Zero(n);
}

/*! 
 * Set the control vector dimension. 
 * @param m dimension (> 0).
 */
void Model::setM(size_t m) 
{
    assert(m > 0);
    this->m = m;
    this->u = VectorXd::Zero(m);
}

/*! 
 * Set the model parameters. 
 * @param params vector of parameters.
 */
void Model::setParams(VectorXd params) {this->params = params;}

/*! 
 * Set local function approximation. 
 * @param c 
 */
void Model::setc(VectorXd c) {this->c = c;}

/*! 
 * Set the state matrix. 
 * @param A state matrix n x n.
 */
void Model::setA(MatrixXd A) {this->A = A;}

/*! 
 * Set the control matrix. 
 * @param B control matrix n x m.
 */
void Model::setB(MatrixXd B) {this->B = B;}

/*!
 * Set the state vector.
 * @param x 
 */
void Model::setX(VectorXd x) {this->x = x;}

/*!
 * Set the control vector.
 * @param u
 */
void Model::setU(VectorXd u) {this->u = u;}

/*! 
 * Set inequality constraint.
 * @param var 'x', 'u', 'du' or 'w' respectively for the state, the control, the control derivative and the slack variable. 
 * @param idx_vec vector index (> 0).
 * @param low lower bound value.
 * @param upp upper bound value.
 */
void Model::setIneq(std::string var, size_t idx_vec, double low, double upp) 
{
    assert(low <= upp); 
    assert(idx_vec > 0); 
    assert(var == "x" || var == "u" || var == "du" || var == "w"); 

    std::vector<double> bounds; 
    bounds.push_back(idx_vec); 
    bounds.push_back(low); 
    bounds.push_back(upp); 

    if (var == "x") 
    {
        ineq_x[map_idx_x] = bounds;
        map_idx_x++;
    }
    else if (var == "u") 
    {
        ineq_u[map_idx_u] = bounds;
        map_idx_u++;
    }
    else if (var == "du") 
    {
        ineq_du[map_idx_du] = bounds;
        map_idx_du++;
    }
    else if (var == "w") 
    {
        ineq_w[map_idx_w] = bounds;
        map_idx_w++;        
    }
}

/*! 
 * Update inequality constraint.
 * @param var 'x', 'u', 'du' or 'w' respectively for the state, the control, the control derivative and the slack variable. 
 * @param idx_vec vector index (> 0).
 * @param low lower bound value.
 * @param upp upper bound value.
 */
void Model::updateIneq(std::string var, size_t idx_vec, double low, double upp) 
{
    assert(low <= upp); 
    assert(idx_vec > 0); 
    assert(var == "x" || var == "u" || var == "du" || var == "w"); 

    std::vector<double> bounds; 
    bounds.push_back(idx_vec); 
    bounds.push_back(low); 
    bounds.push_back(upp); 

    size_t idx;
    if (var == "x") 
    {
        for (size_t i = 0; i < ineq_x.size(); i++)
            if (ineq_x[i][0] == idx_vec) idx = i;
        ineq_x[idx] = bounds;
    }
    else if (var == "u") 
    {
        for (size_t i = 0; i < ineq_u.size(); i++)
            if (ineq_u[i][0] == idx_vec) idx = i;
        ineq_u[idx] = bounds;
    }
    else if (var == "du") 
    {
        for (size_t i = 0; i < ineq_du.size(); i++)
            if (ineq_du[i][0] == idx_vec) idx = i;
        ineq_du[idx] = bounds;
    }
    else if (var == "w") 
    {
        for (size_t i = 0; i < ineq_w.size(); i++)
            if (ineq_w[i][0] == idx_vec) idx = i;
        ineq_w[idx] = bounds;
    }
}

/*! 
 * Set the obstacle avoidance constraint.
 * @param obs_flag false (inactive) or true (active) (default: false). 
 * @param obs_dist minimum distance by the closest obstacle.
 */
void Model::setObsAvoid(bool obs_flag, double obs_dist) 
{
    this->obs_flag = obs_flag;
    this->obs_dist = obs_dist;
}

/*! 
 * Set the model box width.
 * @param width width (>= 0).
 */
void Model::setBoxWidth(double width) 
{
    assert(width >= 0.);
    this->width = width;
}

/*! 
 * Set the model box length.
 * @param length length (>= 0).
 */
void Model::setBoxLength(double length) 
{
    assert(length >= 0.);
    this->length = length;
}

/*! 
 * Set the pose distance from the model box right side.
 * @param pose_width distance (>= 0).
 */
void Model::setPoseWidth(double pose_width) 
{
    assert(pose_width >= 0.);
    this->pose_width = pose_width;
}

/*! 
 * Set the pose distance from the model box back side.
 * @param pose_length distance (>= 0).
 */
void Model::setPoseLength(double pose_length) 
{
    assert(pose_length >= 0.);
    this->pose_length = pose_length;
}

/*! 
 * Set the number of model box points for each side.
 * @param points number (>= 0).
 */
void Model::setBoxPoints(size_t points) 
{
    assert(points >= 0);
    this->points = points;
}

