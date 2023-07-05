#include <mynmpc/mpc.h>
#include <mynmpc/utils.h>
#include <mynmpc/model.h>

class MPC_Bicycle : public MPC 
{
public: 
    MPC_Bicycle(std::shared_ptr<Model> model) 
    {
        /* Problem dimension */
        size_t Np = 50;     // prediction horizon
        size_t Nc = 10;     // control horizon
        double dt = 0.1;    // sample time [s]
        double T = Np * dt; // prediction horizon time [s]

        setNp(Np);
        setNc(Nc);
        setdt(dt);
        
        /* Weight matrices */
        MatrixXd Q = MatrixXd::Identity(model->getN(),model->getN());   // intermediate states
        MatrixXd R = MatrixXd::Zero(model->getM(),model->getM());       // control inputs
        MatrixXd S = MatrixXd::Identity(model->getN(),model->getN());   // final state
        MatrixXd W = MatrixXd::Identity(1,1);                           // slack variable

        Q(3,3) = 0.0;
        S(3,3) = 0.0;
        W *= 1000;

        setQ(Q);
        setR(R);
        setS(S);
        setW(W);

        /* ProxQP setting */
        setMaxExtIterQP(20);
        
        /* SQP setting */
        setMaxIterSQP(5);
    };
};
