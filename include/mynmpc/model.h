#ifndef ROBOT_H
#define ROBOT_H

#include <mynmpc/utils.h>
#include <mynmpc/structs.h>

/* Robot model class. */
class Model : public ModelInfo, Constraints
{
public:

    /* Constructor. */
    Model() 
    {
        this->name = "";
        this->obs_flag = false;
        this->map_idx_x = 0;
        this->map_idx_u = 0;
        this->map_idx_du = 0;
        this->map_idx_w = 0;
    } 

    /* Set */

    void setName(std::string name);
    void setN(size_t n);
    void setM(size_t m);
    void setParams(VectorXd params);
    void setc(VectorXd c);
    void setA(MatrixXd A);
    void setB(MatrixXd B);
    void setIneq(std::string var, size_t idx_vec, double low, double upp);
    void updateIneq(std::string var, size_t idx_vec, double low, double upp);
    void setObsAvoid(bool obs_flag, double obs_dist=2.);
    void setX(VectorXd x);
    void setU(VectorXd u);

    /* Set model box dimensions */

    void setBoxWidth(double width);
    void setBoxLength(double length);
    void setPoseWidth(double pose_width);
    void setPoseLength(double pose_length);
    void setBoxPoints(size_t points);
    
    /* Virtual set functions */

    /*!
     * Set the local function approximation of the objective function for the QP sub-problem used by the SQP.
     * @param dt step size.
     * @param x_next next state.
     */
    virtual void updatec(double dt, VectorXd x_next) {}

    /*!
     * Set the state matrix in the kinematics equality constraint for the QP sub-problem used by the SQP.
     */
    virtual void updateA(double dt) {}

    /*!
     * Set the control matrix in the kinematics equality constraint for the QP sub-problem used by the SQP.
     */
    virtual void updateB() {}

    /* Get */

    std::string getName();
    size_t getN();  
    size_t getM();
    VectorXd getParams();
    VectorXd getX();
    VectorXd getU();
    VectorXd getc();
    MatrixXd getA();
    MatrixXd getB();
    std::map<int, std::vector<double>> getIneq(std::string var);
    bool getObsFlag();
    double getObsDist();

    /* Get model box dimensions */

    double getBoxWidth();
    double getBoxLength();
    double getPoseWidth();
    double getPoseLength();
    double getBoxPoints();

private:
    /* Model box */
    double width;       // Model box width.
    double length;      // Model box length.
    double pose_width;  // Pose distance from the model box right side.
    double pose_length; // Pose distance from the model box back side.
    size_t points;      // Box points for each side.
};

#endif
