/* 
 * Controller for the ROSbot 2R based on the unicycle model. 
 * The optimal control input is computed by a Nonlinear Model Predictive Control
 * based on the Local Sequential Quadratic Programming method, iteratively calling
 * the ProxQP solver.
 * 
 * Author: Simone Contorno
 */

/* NMPC */
#include <mynmpc/utils.h>
#include <mynmpc/mpc.h>
#include <mynmpc/model.h>
#include <mynmpc/proxqp.h>
#include <mynmpc/structs.h>

/* Models */
#include <mynmpc/models/rosbot.h>
#include <mynmpc/models/obs.h>

/* MPC configs */
#include <mynmpc/mpc/mpc_rosbot.h>
#include <mynmpc/mpc/mpc_obs.h>

/* Transform */
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/********************************************************************************/
/******************************* Global variables *******************************/
/********************************************************************************/

// Global publishers
PubTwist pub_twist;

// Global messages
geometry_msgs::msg::Twist twist;

// Global data
VectorXd pose_odom;
VectorXd pose;
MatrixXd x;
MatrixXd goal_x;
MatrixXd goal_u;
MatrixXd goal_x_obs;

/* Colours */
std::string cyanColour = "\033[96m";
std::string yellowColour = "\033[93m";
std::string redColour = "\033[91m";
std::string endColour = "\033[0m";

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

/*!
 * Initialize the controller.
 * @param mpc Model Predictive Control pointer.
 * @param model model pointer.
 */
void controllerInit(std::shared_ptr<MPC> mpc, std::shared_ptr<Model> model) 
{
    pose_odom = VectorXd::Zero(model->getN());              // odometry
    pose = VectorXd::Zero(model->getN());                   // odometry corrected with AMCL TF
    x = MatrixXd::Zero(mpc->getNp()+1,model->getN());       // states matrix
    goal_x = MatrixXd::Zero(mpc->getNp()+1,model->getN());  // state goals 
    goal_u = MatrixXd::Zero(mpc->getNc()+1,model->getM());  // control goals 

    /* Obstacle avoidance */
    goal_x_obs = MatrixXd::Zero(mpc->getNp()+1, 2);
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
    auto node = std::make_shared<rclcpp::Node>("controller"); 

    /********************************************************************************/
    /****************************** Model and MPC ***********************************/
    /********************************************************************************/

    auto model = std::make_shared<Model>();
    auto model_obs = std::make_shared<Model>();  
    auto mpc =  std::make_shared<MPC>();

    model = std::make_shared<Unicycle>(); 
    model_obs = std::make_shared<Obs>(); 
    mpc = std::make_shared<MPC_Unicycle>(model);

    // Initialize the MPC
    mpc->init(node, model); 

    // If the argument is passed, activate or deactivate the obstacle avoidance
    if (argc > 1) 
    {
        if (static_cast<string>(argv[1]) == "0") 
        {
            model->setObsAvoid(false);
            std::cout << "[MODEL] Obstacle avoidance inactive." << std::endl;
        }
        else if (static_cast<string>(argv[1]) == "1")
        {
            model->setObsAvoid(true);
            std::cout << "[MODEL] Obstacle avoidance active." << std::endl;
        }
    }

    /********************************************************************************/
    /********************************* Controller ***********************************/
    /********************************************************************************/

    controllerInit(mpc, model);

    /* Obstacles data */
    double obs_l = model_obs->getBoxLength();
    double obs_w = model_obs->getBoxWidth();
    double obs_l_down = model_obs->getPoseLength();
    double obs_l_right = model_obs->getPoseWidth(); 

    /********************************************************************************/
    /************************************* Box **************************************/
    /********************************************************************************/

    std_msgs::msg::Float32MultiArray box_msg;
    MatrixXd robot_box;    
    auto pub_box = node->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/box", 1);
    
    size_t points = 2;                          // points for each side
    double box_dim = (points + 1) * 4 * 2 + 2;
    box_msg.data.resize(box_dim);               // resize the box data dimension  
    double pose_length = model->getPoseLength();

    /********************************************************************************/
    /********************************* Subscribers **********************************/
    /********************************************************************************/

    /* Transform */
    VectorXd T_vector = VectorXd::Zero(model->getN());
    MatrixXd R_matrix = MatrixXd::Zero(model->getN(),model->getN());
    VectorXd tf = VectorXd::Zero(7);
    double roll, pitch, yaw;
    auto sub_tf = 
        node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 1, [&](tf2_msgs::msg::TFMessage::UniquePtr msg)
    {
        if (msg->transforms[0].header.frame_id == "map" && msg->transforms[0].child_frame_id == "robot_2/odom")
        {   
            // Translation
            tf(0) = msg->transforms[0].transform.translation.x;
            tf(1) = msg->transforms[0].transform.translation.y;
            tf(2) = msg->transforms[0].transform.translation.z;
            T_vector << tf(0), tf(1), tf(2);

            // Rotation (Quaternion)
            tf(3) = msg->transforms[0].transform.rotation.x;
            tf(4) = msg->transforms[0].transform.rotation.y;
            tf(5) = msg->transforms[0].transform.rotation.z;
            tf(6) = msg->transforms[0].transform.rotation.w;

            tf2::Quaternion tf_quaternion(tf(3), tf(4), tf(5), tf(6));
            tf2::Matrix3x3 tf_orientation(tf_quaternion);
            tf_orientation.getRPY(roll, pitch, yaw);    

            R_matrix << cos(yaw),   -sin(yaw),  0,
                        sin(yaw),   +cos(yaw),  0,
                        0,          0,          1;
        }
    });

    /* Odometry */
    bool pose_received = 0;
    auto sub_odom = 
        node->create_subscription<nav_msgs::msg::Odometry>("/robot_2/odom", 1, [&](nav_msgs::msg::Odometry::UniquePtr msg)
    {
        // Get pose 
        pose_odom(2) = 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        pose_odom(1) = msg->pose.pose.position.y;
        pose_odom(0) = msg->pose.pose.position.x;

        pose = R_matrix * pose_odom + T_vector;
        normalizeAngle(pose(2));

        /* PlotJuggler Box */

        // Update 
        double new_pose_length = pose_length;
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

        pose_received = true;
    });

    /* Simulation goals */
    MatrixXd goals = MatrixXd::Zero(mpc->getNp(),3);
    auto sub_goals = 
        node->create_subscription<std_msgs::msg::Float32MultiArray>("robot/goals", 1, [&](std_msgs::msg::Float32MultiArray::UniquePtr msg)
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
        node->create_subscription<geometry_msgs::msg::PoseStamped>("robot/goal", 1, [&](geometry_msgs::msg::PoseStamped::UniquePtr msg)
    {
        goal_x(0,0) = msg->pose.position.x;  
        goal_x(0,1) = msg->pose.position.y;
        goal_x(0,2) = 2*atan2(msg->pose.orientation.z, msg->pose.orientation.w);

        for (size_t i = 0; i < goals.rows(); i++)
        {
            goal_x(i+1,0) = goals(i,0);
            goal_x(i+1,1) = goals(i,1);
            goal_x(i+1,2) = goals(i,2);
        }

    });

    /* Obstacle */
    MatrixXd goals_obs = MatrixXd::Zero(mpc->getNp(),3);
    auto sub_goals_obs = 
        node->create_subscription<std_msgs::msg::Float32MultiArray>("obs1/goals", 1, [&](std_msgs::msg::Float32MultiArray::UniquePtr msg)
    {
        size_t j = 0;
        for (size_t i = 0; i < goals_obs.rows(); i++)
        {
            goals_obs(i,0) = msg->data[j];
            goals_obs(i,1) = msg->data[j+1];
            goals_obs(i,2) = msg->data[j+2];
            
            j += goals_obs.cols();
        }
    });

    auto sub_goal_obs = 
        node->create_subscription<geometry_msgs::msg::PoseStamped>("obs1/goal", 1, [&](geometry_msgs::msg::PoseStamped::UniquePtr msg)
    {
        goal_x_obs(0,0) = msg->pose.position.x;  
        goal_x_obs(0,1) = msg->pose.position.y;

        for (size_t i = 0; i < goals_obs.rows(); i++)
        {
            goal_x_obs(i+1,0) = goals_obs(i,0);
            goal_x_obs(i+1,1) = goals_obs(i,1);
        }
    });    

    /********************************************************************************/
    /********************************* Publishers ***********************************/
    /********************************************************************************/

    /* Control and Simulation */
    auto path_pub = node->create_publisher<Path>("/robot/optim_path", 1);
    pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("/robot_2/cmd_vel", 1);

    /********************************************************************************/
    /************************************ Table *************************************/
    /********************************************************************************/

    uint table_offset = 2;
    uint table_w = 0;
    uint max_count = 30;
    uint count = max_count;
    uint loop = 0;
    
    vector<std::string> titles(8);
    titles[0] = "loop";
    titles[1] = "v [m/s]";
    titles[2] = "w [rad/s]";
    titles[3] = "time [ms]";
    titles[4] = "SQP iter";
    titles[5] = "QP iter";
    titles[6] = "deviation";
    titles[7] = "J";
    for (size_t i = 0; i < titles.size(); i++)
        table_w += titles[i].size()+table_offset;

    /********************************************************************************/
    /*********************************** Control ************************************/
    /********************************************************************************/
    
    double deviation = 0.0;
    MatrixXd err = MatrixXd::Zero(mpc->getNp()+1, 2);
    double dt = 0.1;
    auto robot_timer = [&]()
    {
        if (pose_received == true)
        {
            /* Start optimization */
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

            /* Print table header */
            if (count == max_count) 
            {
                std::cout << cyanColour;
                std::cout << setfill('-') << setw(table_w) << "-" << std::endl;
                std::cout << setfill(' ') << fixed; 

                for (uint i = 0; i < titles.size(); i++)
                    std::cout << left << setw(titles[i].size()+table_offset) << titles[i];
                
                std::cout << "" << std::endl; 
                std::cout << setfill('-') << setw(table_w) << "-" << std::endl; 
                std::cout << setfill(' ') << fixed; 
                std::cout << endColour;

                count = 0;
            }
            
            /* Solve the problem */
            mpc->setPose(pose);                                     // set the pose  
            mpc->setGoalX(goal_x);                                  // set the state goals
            mpc->setGoalU(goal_u);                                  // set the control goals
            mpc->setObs(goal_x_obs);                                // set obstacle goals
            mpc->setObsDim(obs_l, obs_w, obs_l_down, obs_l_right);  // set obstacle box dimensions
            auto [x_sol, u_sol] = mpc->solve(); {}

            /* Optimal Path */
            path_pub->publish(optimPath(x_sol, node->get_clock()->now()));

            /* Send the control input to the robot */
            twist.linear.x = u_sol(0,0);
            twist.angular.z = u_sol(0,1);

            pub_twist->publish(twist);

            /* Deviation */
            err = goal_x.block(0,0,goal_x.rows(),2) - x_sol.block(0,0,x_sol.rows(),2);
            deviation = (err.array().abs()).mean(); 

            /* Print results */
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            
            std::cout << left << setw(titles[0].size()+table_offset) << loop  
                << left << setw(titles[1].size()+table_offset) << twist.linear.x 
                << left << setw(titles[2].size()+table_offset) << twist.angular.z 
                << left << setw(titles[3].size()+table_offset) << duration.count() 
                << left << setw(titles[4].size()+table_offset) << mpc->sqp_iter 
                << left << setw(titles[5].size()+table_offset) << mpc->qp_iter_ext 
                << left << setw(titles[6].size()+table_offset) << deviation 
                << left << setw(titles[7].size()+table_offset) << mpc->qp_info.objValue 
                << std::endl;
            
            count++;

            /* Update */
            loop++;
            pose_received = false;
        }
        else std::cout << yellowColour << "[WARNING] " << endColour << "Updated pose has not been received yet!" << std::endl;
    };

    rclcpp::TimerBase::SharedPtr one_off_timer = 
        node->create_wall_timer(std::chrono::milliseconds{(int)(dt*1000)}, robot_timer);

    /* CTRL+C signal handling */
    std::signal(SIGINT, signalHandler);

    /********************************************************************************/
    /*********************************** WaitSet ************************************/
    /********************************************************************************/

    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub_odom);

    while (rclcpp::ok()) 
    {
        const auto wait_result = wait_set.wait(std::chrono::seconds(1));
        rclcpp::spin_some(node);
        
        if (wait_result.kind() == rclcpp::WaitResultKind::Ready) one_off_timer->execute_callback();
        else std::cout << yellowColour << "[WARNING] " << endColour << "Waiting time expired!" << std::endl;
    }
}