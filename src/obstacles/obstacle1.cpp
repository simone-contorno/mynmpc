// NMPC
#include <mynmpc/utils.h>
#include <mynmpc/mpc.h>
#include <mynmpc/model.h>

// Models
#include <mynmpc/models/obs.h>

// MPC configs
#include <mynmpc/mpc/mpc_obs.h>

/********************************************************************************/
/******************************* Global variables *******************************/
/********************************************************************************/

/* Global publishers */
PubTwist pub_twist;
PubFloat32MultiArray pub_box;

/* Global messages */
geometry_msgs::msg::Twist twist;
std_msgs::msg::Float32MultiArray box_msg;

/* Global data */
VectorXd pose;
MatrixXd x;
MatrixXd goal_x;
MatrixXd goal_u;

/********************************************************************************/
/********************************** Functions ***********************************/
/********************************************************************************/

/*!
 * Catch the CTRL+C command and stop the robot before shutting down the node.
 * @param signal type of signal.
 */
void signalHandler(int signal)
{
    RCLCPP_INFO(rclcpp::get_logger("shutdown"), "Stopping the robot...");
    
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    pub_twist->publish(twist);

    rclcpp::shutdown();
}

/********************************************************************************/
/************************************ Main **************************************/
/********************************************************************************/

int main(int argc, char** argv) 
{     
    /********************************************************************************/
    /************************************ Node **************************************/
    /********************************************************************************/

    rclcpp::init(argc, argv); 
    auto node = std::make_shared<rclcpp::Node>("obs1"); 

    /********************************************************************************/
    /****************************** Model and MPC ***********************************/
    /********************************************************************************/

    /* Model */
    auto model = std::make_shared<Model>();    
    model = std::make_shared<Obs>(); 

    /* MPC */
    auto mpc =  std::make_shared<MPC>();
    mpc = std::make_shared<MPC_Obs>(model);
    mpc->init(model); 

    /********************************************************************************/
    /********************************* Controller ***********************************/
    /********************************************************************************/

    pose = VectorXd::Zero(model->getN());                   // current pose
    x = MatrixXd::Zero(mpc->getNp()+1,model->getN());       // states matrix
    goal_x = MatrixXd::Zero(mpc->getNp()+1,model->getN());  // state goals 
    goal_u = MatrixXd::Zero(mpc->getNc()+1,model->getM());  // control goals 

    /********************************************************************************/
    /************************************ Box ***************************************/
    /********************************************************************************/

    MatrixXd robot_box;    
    pub_box = node->create_publisher<std_msgs::msg::Float32MultiArray>("/obs1/box", 1);

    size_t points = 2;  // points for each side
    double box_dim = (points + 1) * 4 * 2 + 2;
    box_msg.data.resize(box_dim);   // resize the box data dimension  
    double pose_length = model->getPoseLength();

    /********************************************************************************/
    /********************************* Subscribers **********************************/
    /********************************************************************************/

    /* Joints */
    double carrot_x = 0.0;  // carrot distance

    auto sub_joint = 
        node->create_subscription<sensor_msgs::msg::JointState>("obs1/joint_states", 1, [&](sensor_msgs::msg::JointState::UniquePtr carr)
    {
        if (carr->name[0] == "carrot") {carrot_x = carr->position[0];} // carrot position
    });

    /* Pose */
    auto sub_pose = 
        node->create_subscription<nav_msgs::msg::Odometry>("/obs1/odom", 1, [&](nav_msgs::msg::Odometry::UniquePtr odom)
    {
        /* Get pose */
        pose(2) = 2*atan2(odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        normalizeAngle(pose(2));
        pose(1) = odom->pose.pose.position.y + model->getParams()[0]*sin(pose(2)) + carrot_x*sin(pose(2));
        pose(0) = odom->pose.pose.position.x + model->getParams()[0]*cos(pose(2)) + carrot_x*cos(pose(2));

        /* PlotJuggler */

        // Update 
        double new_pose_length = pose_length + carrot_x;
        model->setPoseLength(pose_length);

        // Box computation          
        robot_box = computeBox( pose, 
                                model->getBoxLength(), 
                                model->getBoxWidth(), 
                                model->getPoseLength(), 
                                model->getPoseWidth(), 
                                points);

        // Fill poses in the box vector
        box_msg.data[0] = pose(0);
        box_msg.data[1] = pose(1);

        for (size_t i = 2; i < box_dim; i += 2) 
        {
            box_msg.data[i] = robot_box(i/2,0);
            box_msg.data[i+1] = robot_box(i/2,1);    
        }
        
        // Publish the box 
        pub_box->publish(box_msg); 
    });

    /* Simulator goals */
    MatrixXd goals = MatrixXd::Zero(mpc->getNp(),3);
    auto sub_goals = 
        node->create_subscription<std_msgs::msg::Float32MultiArray>("obs1/goals", 1, [&](std_msgs::msg::Float32MultiArray::UniquePtr msg)
    {
        size_t j = 0;
        for (size_t i = 0; i < goals.rows(); i++)
        {
            goals(i,0) = msg->data[j];
            goals(i,1) = msg->data[j+1];
            goals(i,2) = msg->data[j+2];
            
            j += goals.cols();
        }
    });

    /* Robot goals */
    auto sub_goal = 
        node->create_subscription<geometry_msgs::msg::PoseStamped>("obs1/goal", 1, [&](geometry_msgs::msg::PoseStamped::UniquePtr pos)
    {
        /* Current pose */
        goal_x(0,0) = pos->pose.position.x;  
        goal_x(0,1) = pos->pose.position.y;
        goal_x(0,2) = 2*atan2(pos->pose.orientation.z, pos->pose.orientation.w);

        for (size_t i = 0; i < goals.rows(); i++)
        {
            goal_x(i+1,0) = goals(i,0);
            goal_x(i+1,1) = goals(i,1);
            goal_x(i+1,2) = goals(i,2);
        }
    });

    /********************************************************************************/
    /********************************* Publishers ***********************************/
    /********************************************************************************/
    
    pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("/obs1/cmd_vel", 1);

    /********************************************************************************/
    /*********************************** Control ************************************/
    /********************************************************************************/

    double dt = 0.05;
    auto robot_timer = node->create_wall_timer(std::chrono::milliseconds{(int)(dt*1000)}, [&]()
    {
        auto start = std::chrono::high_resolution_clock::now();

        /* Avoid orientation discontinuity */
        x = mpc->getX();
        for (size_t i = 0; i < mpc->getNp()+1; i++)
        { 
            // Goal
            normalizeAngle(goal_x(i,2));
            if (goal_x(i,2)-pose(2) > M_PI) goal_x(i,2) -= 2*M_PI;
            else if (goal_x(i,2)-pose(2) < -M_PI) goal_x(i,2) += 2*M_PI;

            // States
            normalizeAngle(x(i,2));
            if (x(i,2)-pose(2) > M_PI) x(i,2) -= 2*M_PI;
            else if (x(i,2)-pose(2) < -M_PI) x(i,2) += 2*M_PI;
        }
        mpc->setX(x);

        /* Solve the problem */
        mpc->setPose(pose);         // set the pose  
        mpc->setGoalX(goal_x);      // set the state goals
        mpc->setGoalU(goal_u);      // set the control goals
        auto [x_sol, u_sol] = mpc->solve(); {}

        /* Send the control input to the robot */
        twist.linear.x = u_sol(0,0);
        twist.angular.z = u_sol(0,1);
        pub_twist->publish(twist);
    });

    /* CTRL+C signal handling */
    std::signal(SIGINT, signalHandler);
    
    /* Spin */
    rclcpp::spin(node);
}