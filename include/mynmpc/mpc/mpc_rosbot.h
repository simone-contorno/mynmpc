#include <mynmpc/mpc.h>
#include <mynmpc/utils.h>
#include <mynmpc/model.h>

class MPC_Unicycle : public MPC 
{
public:
    MPC_Unicycle(std::shared_ptr<Model> model) 
    {
        /* Problem dimension */
        size_t Np = 50;
        size_t Nc = 50;
        double dt = .2; 
        double T = Np * dt;

        setNp(Np);
        setNc(Nc);
        setdt(dt);
        
        /* Weight matrices */
        MatrixXd Q = MatrixXd::Identity(model->getN(),model->getN());   // intermediate states
        MatrixXd R = MatrixXd::Identity(model->getM(),model->getM());   // control inputs
        MatrixXd S = MatrixXd::Identity(model->getN(),model->getN());   // final state
        MatrixXd W = MatrixXd::Identity(1,1);                           // slack variables 

        R *= 0.01;
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
