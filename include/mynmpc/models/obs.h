#include <mynmpc/model.h>
#include <mynmpc/utils.h>

class Obs : public Model 
{
public:
    Obs() 
    {
        /* Model info */ 
        setName("r2d2"); // set model name
        
        /* Dimensions */
        size_t n = 3;   // state dimension
        size_t m = 2;   // control dimension
        
        setN(n); // set state vector dimension
        setM(m); // set control vector dimension

        /* Parameters */
        double L = 0.0; // length

        VectorXd params(1);
        params << L;
        setParams(params); // set model parameters
        
        /* Dynamics */
        MatrixXd A = MatrixXd::Zero(getN(),getN()); // define state matrix
        MatrixXd B = MatrixXd::Zero(getN(),getM()); // define control matrix
        MatrixXd c = VectorXd::Zero(getN());        // define c vector

        setc(c);    // set c vector
        setA(A);    // set state matrix
        setB(B);    // set control matrix

        /* Constraints */
        setIneq("u", 0, -3.0, 3.0); // linear velocity [m/s]
        setIneq("u", 1, -1., 1.);   // angular velocity [rad/s]
        setIneq("du", 0, -.5, .5);  // linear acceleration [m/s^2]
        setIneq("du", 1, -.5, .5);  // angular acceleration [rad/s^2]
        
        /* Model box */
        setBoxWidth(.5);
        setBoxLength(.4);
        setPoseWidth(getBoxWidth()/2);
        setPoseLength(getBoxLength()/2);  
        setBoxPoints(20);      
    }

    void updatec(double dt, VectorXd x_next) 
    {
        c <<    getX()(0) - x_next(0) + dt*getU()(0)*cos(getX()(2)),
                getX()(1) - x_next(1) + dt*getU()(0)*sin(getX()(2)),
                getX()(2) - x_next(2) + dt*getU()(1);
    }

    void updateA(double dt) 
    {
        A <<    1.0,    0.0,    -dt*getU()(0)*sin(getX()(2)),
                0.0,    1.0,    +dt*getU()(0)*cos(getX()(2)),
                0.0,    0.0,    1.0;
    }

    void updateB() 
    {
        B <<    cos(getX()(2)), 0.0,
                sin(getX()(2)), 0.0,
                0.0, 1.0;
    }
};
