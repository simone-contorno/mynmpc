#include <mynmpc/model.h>
#include <mynmpc/utils.h>

class Bicycle : public Model 
{
public:
    Bicycle() 
    {
        /* Model info */
        setName("bike"); // set model name
        
        /* Dimensions */
        size_t n = 4;   // state dimension
        size_t m = 2;   // control dimension
        
        setN(n); // set state vector dimension
        setM(m); // set control vector dimension

        /* Parameters */
        double L = 1.6; // bike length 

        VectorXd params(1);
        params << L;
        setParams(params); // set model parameters

        /* Kinematics */
        MatrixXd A = MatrixXd::Zero(getN(),getN()); // define state matrix
        MatrixXd B = MatrixXd::Zero(getN(),getM()); // define control matrix
        MatrixXd c = VectorXd::Zero(getN());        // define c vector

        setA(A);    // set state matrix
        setB(B);    // set control matrix
        setc(c);    // set c vector

        /* Constraints */
        setIneq("x", 3, -M_PI/2, M_PI/2);   // steering angle [rad]
        setIneq("u", 0, -3.0, 3.0);         // linear velocity [m/s]
        setIneq("u", 1, -1., 1.);           // angular velocity [rad/s]
        setIneq("du", 0, -.5, .5);          // linear acceleration [m/s^2]
        setIneq("du", 1, -.5, .5);          // angular acceleration [rad/s^2]

        /* Obstacle avoidance */
        setObsAvoid(true, 0.2); // Euclidean distance [m]

        /* Model box */
        setBoxWidth(1.);
        setBoxLength(2.7);
        setPoseWidth(getBoxWidth()/2);
        setPoseLength(L);        
        setBoxPoints(20);
    }

    void updatec(double dt, VectorXd x_next) 
    {
        c <<    getX()(0) - x_next(0) + dt*getU()(0)*cos(getX()(2) + getX()(3)),
                getX()(1) - x_next(1) + dt*getU()(0)*sin(getX()(2) + getX()(3)),
                getX()(2) - x_next(2) + dt*getU()(0)*sin(getX()(3)) / getParams()[0],
                getX()(3) - x_next(3) + dt*getU()(1);
    }

    void updateA(double dt) 
    {
        A <<    1.0,    0.0,    -dt*getU()(0)*sin(getX()(2) + getX()(3)),   -dt*getU()(0)*sin(getX()(2) + getX()(3)),
                0.0,    1.0,    +dt*getU()(0)*cos(getX()(2) + getX()(3)),   +dt*getU()(0)*cos(getX()(2) + getX()(3)),
                0.0,    0.0,    1.0,                                        +dt*getU()(0)*cos(getX()(3)) / getParams()[0],
                0.0,    0.0,    0.0,                                        1.0;
    }

    void updateB() 
    {
        B <<    cos(getX()(2) + getX()(3)),         0.0,
                sin(getX()(2) + getX()(3)),         0.0,
                sin(getX()(3)) / getParams()[0],    0.0,
                0.0,                                1.0;
    }
};
