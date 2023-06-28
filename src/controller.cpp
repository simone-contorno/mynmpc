/* 
 * Controller for the R2D2 and Bike models based respectively on the unicycle and bicycle models. 
 * The optimal control input is computed by a Nonlinear Model Predictive Control
 * based on the Local Sequential Quadratic Programming method, iteratively calling
 * the ProxQP solver.
 * 
 * Author: Simone Contorno
 */

// NMPC
#include <mynmpc/utils.h>
#include <mynmpc/mpc.h>
#include <mynmpc/model.h>

// Models
#include <mynmpc/models/r2d2.h>
#include <mynmpc/models/bike.h>

// MPC configs
#include <mynmpc/mpc/mpc_r2d2.h>
#include <mynmpc/mpc/mpc_bike.h>

/********************************************************************************/
/******************************* Global variables *******************************/
/********************************************************************************/

/* Global publishers */
PubTwist pub_twist;
PubFloat32MultiArray pub_cmd;

/* Global messages */
geometry_msgs::msg::Twist twist;
std_msgs::msg::Float32MultiArray cmd;

/* Global data */
VectorXd pose;
MatrixXd x;
MatrixXd goal_x;
MatrixXd goal_u;
MatrixXd goal_x_obs1;
MatrixXd goal_x_obs2;
MatrixXd goal_x_obs;

/* Colours */
std::string cyanColour = "\033[96m";
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
    
    cmd.data[0] = 0.0;
    cmd.data[1] = 0.0;
    pub_cmd->publish(cmd);

    rclcpp::shutdown();
}

/*!
 * Initialize the controller.
 * @param mpc Model Predictive Control pointer.
 * @param model model pointer.
 */
void controllerInit(std::shared_ptr<MPC> mpc, std::shared_ptr<Model> model) 
{
    pose = VectorXd::Zero(model->getN());  // current pose

    x = MatrixXd::Zero(mpc->getNp()+1,model->getN());   // states matrix

    goal_x = MatrixXd::Zero(mpc->getNp()+1,model->getN()); // state goals 
    goal_u = MatrixXd::Zero(mpc->getNc()+1,model->getM()); // control goals 

    /* Obstacle avoidance */
    goal_x_obs1 = MatrixXd::Zero(mpc->getNp()+1, 2);
    goal_x_obs2 = MatrixXd::Zero(mpc->getNp()+1, 2);
    goal_x_obs = MatrixXd::Zero(mpc->getNp()+1, 2);
}

/*! 
 * Read the robot model name. 
 * @param rsp_param client instance to the robot state publisher.
 */
std::string readModel(rclcpp::SyncParametersClient::SharedPtr rsp_param) 
{
    std::cout << "Reading robot description... " << std::endl;  
    if (rsp_param->wait_for_service(1s) == false) 
    {
        std::cout << "Nothing found." << std::endl;
        return "";
    } 
    else 
    {
        const auto urdf_xml{rsp_param->get_parameter<std::string>("robot_description")};
        auto model{std::make_unique<urdf::Model>()};
        model->initString(urdf_xml);
        std::cout << "Found: " << model->getName() << "." << std::endl << std::endl;
        return model->getName();
    }
}

/*!
 * Print program information.
 * @param mpc Model Predictive Control pointer.
 * @param model model pointer.
 */
void printInfo(std::shared_ptr<MPC> mpc, std::shared_ptr<Model> model) 
{
    std::cout << "[MODEL] " << model->getName() << " set." << std::endl << std::endl;

    if (model->getIneq("u").size() > 0)
    {
        std::cout << "Max v = " << model->getIneq("u")[0][2] << " [m/s]" << std::endl;
        std::cout << "Min v = " << model->getIneq("u")[0][1] << " [m/s]" << std::endl;
    }

    if (model->getIneq("u").size() > 1)
    {
        std::cout << "Max w = " << model->getIneq("u")[1][2] << " [rad/s]" << std::endl;
        std::cout << "Min w = " << model->getIneq("u")[1][1] << " [rad/s]" << std::endl << std::endl;
    }

    if (model->getIneq("du").size() > 0)
    {
        std::cout << "Max dv = " << model->getIneq("du")[0][2] << " [m/s^2]" << std::endl;
        std::cout << "Min dv = " << model->getIneq("du")[0][1] << " [m/s^2]" << std::endl;
    }

    if (model->getIneq("du").size() > 1)
    {
        std::cout << "Max dw = " << model->getIneq("du")[1][2] << " [rad/s^2]" << std::endl;
        std::cout << "Min dw = " << model->getIneq("du")[1][1] << " [rad/s^2]" << std::endl << std::endl;
    }
    
    std::cout << "[MPC]" << std::endl << std::endl;
    
    std::cout << "Np = " << mpc->getNp() << std::endl;
    std::cout << "Nc = " << mpc->getNc() << std::endl;
    std::cout << "dt = " << mpc->getdt() << " [s]" << std::endl;
    std::cout << "T = " << mpc->getT() << " [s]" << std::endl << std::endl;

    std::cout << "Q = \n" << mpc->getQ() << std::endl << std::endl;
    std::cout << "R = \n" << mpc->getR() << std::endl << std::endl;
    std::cout << "S = \n" << mpc->getS() << std::endl << std::endl;
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
    auto mpc =  std::make_shared<MPC>();

    // Get and Set model name
    auto rsp_param{std::make_shared<rclcpp::SyncParametersClient>(node, "/robot/robot_state_publisher")}; 
    std::string name = ""; 
    if (argc > 2) name = static_cast<string>(argv[2]);
    else name = readModel(rsp_param); 
    if (name == "") 
    {
        std::cout << "Please, insert the model name: ";
        std::cin >> name;
    }
    model->setName(name); 

    // Set the corresponding model and mpc
    if (model->getName() == "r2d2") 
    {
        model = std::make_shared<Unicycle>(); 
        mpc = std::make_shared<MPC_Unicycle>(model);
    }
    else if (model->getName() == "bike") 
    {
        model = std::make_shared<Bicycle>(); 
        mpc = std::make_shared<MPC_Bicycle>(model);
    }
    else
    {
        std::cerr << "[MODEL] Error: model not implemented." << std::endl;
        rclcpp::shutdown();
        return 0;
    }
    
    // Initialize the MPC
    mpc->init(node, model); 
    
    // Print Model and MPC informations
    printInfo(mpc,model);
    
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
    double obs_l = 1.;
    double obs_w = 1.;
    double obs_l_down = 0.2;
    double obs_l_right = 0.2; 

    /********************************************************************************/
    /************************************ Box ***************************************/
    /********************************************************************************/

    MatrixXd robot_box;   
    std_msgs::msg::Float32MultiArray box_msg; 
    auto pub_box = node->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/box", 1);

    size_t points = 2;                          // points for each side
    double box_dim = (points + 1) * 4 * 2 + 2;
    box_msg.data.resize(box_dim);               // resize the box data dimension  
    double pose_length = model->getPoseLength();

    /********************************************************************************/
    /********************************* Subscribers **********************************/
    /********************************************************************************/

    /* Joints */
    double carrot_x = 0.0;  // carrot distance
    double beta = 0.0;      // steering angle   

    auto sub_joint = 
        node->create_subscription<sensor_msgs::msg::JointState>("robot/joint_states", 1, [&](sensor_msgs::msg::JointState::UniquePtr carr)
    {
        if (carr->name[0] == "carrot") {carrot_x = carr->position[0];} // carrot position
        else if (carr->name[0] == "handlebar_to_frontwheel") {beta = carr->position[2];} // bike position
    });

    /* Pose */
    auto sub_pose = 
        node->create_subscription<nav_msgs::msg::Odometry>("/robot/odom", 1, [&](nav_msgs::msg::Odometry::UniquePtr odom)
    {
        /* Get pose */
        if (model->getName() != "r2d2") pose(3) = beta;
        pose(2) = 2*atan2(odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        normalizeAngle(pose(2));
        pose(1) = odom->pose.pose.position.y + model->getParams()[0]*sin(pose(2)) + carrot_x*sin(pose(2)+beta);
        pose(0) = odom->pose.pose.position.x + model->getParams()[0]*cos(pose(2)) + carrot_x*cos(pose(2)+beta);

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

    /*********/
    /* Goals */
    /*********/

    /* Robot */
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

    auto sub_goal = 
        node->create_subscription<geometry_msgs::msg::PoseStamped>("robot/goal", 1, [&](geometry_msgs::msg::PoseStamped::UniquePtr pos)
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

    /* Obstacle 1 */
    MatrixXd goals_obs1 = MatrixXd::Zero(mpc->getNp(),3);
    auto sub_goals_obs1 = 
        node->create_subscription<std_msgs::msg::Float32MultiArray>("obs1/goals", 1, [&](std_msgs::msg::Float32MultiArray::UniquePtr msg)
    {
        size_t j = 0;
        for (size_t i = 0; i < goals_obs1.rows(); i++)
        {
            goals_obs1(i,0) = msg->data[j];
            goals_obs1(i,1) = msg->data[j+1];
            goals_obs1(i,2) = msg->data[j+2];
            
            j += goals_obs1.cols();
        }
    });

    auto sub_goal_obs1 = 
        node->create_subscription<geometry_msgs::msg::PoseStamped>("obs1/goal", 1, [&](geometry_msgs::msg::PoseStamped::UniquePtr pos)
    {
        // Current pose
        goal_x_obs1(0,0) = pos->pose.position.x;  
        goal_x_obs1(0,1) = pos->pose.position.y;

        for (size_t i = 0; i < goals_obs1.rows(); i++)
        {
            goal_x_obs1(i+1,0) = goals_obs1(i,0);
            goal_x_obs1(i+1,1) = goals_obs1(i,1);
        }
    });    

    /* Obstacle 2 */
    MatrixXd goals_obs2 = MatrixXd::Zero(mpc->getNp(),3);
    auto sub_goals_obs2 = 
        node->create_subscription<std_msgs::msg::Float32MultiArray>("obs2/goals", 1, [&](std_msgs::msg::Float32MultiArray::UniquePtr msg)
    {
        size_t j = 0;
        for (size_t i = 0; i < goals_obs2.rows(); i++)
        {
            goals_obs2(i,0) = msg->data[j];
            goals_obs2(i,1) = msg->data[j+1];
            goals_obs2(i,2) = msg->data[j+2];
            
            j += goals_obs2.cols();
        }
    });

    auto sub_goal_obs2 = 
        node->create_subscription<geometry_msgs::msg::PoseStamped>("obs2/goal", 1, [&](geometry_msgs::msg::PoseStamped::UniquePtr pos)
    {
        // Current pose
        goal_x_obs2(0,0) = pos->pose.position.x;  
        goal_x_obs2(0,1) = pos->pose.position.y;

        for (size_t i = 0; i < goals_obs2.rows(); i++)
        {
            goal_x_obs2(i+1,0) = goals_obs2(i,0);
            goal_x_obs2(i+1,1) = goals_obs2(i,1);
        }
    });    

    /********************************************************************************/
    /********************************* Publishers ***********************************/
    /********************************************************************************/

    auto path_pub = node->create_publisher<Path>("/robot/optim_path", 1);
    pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);
    pub_cmd = node->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/cmd", 1);
    cmd.data.resize(2);

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
    double diff_x;
    double diff_y;
    bool root_flag = 1;
    double dt = 0.05;
    auto robot_timer = node->create_wall_timer(std::chrono::milliseconds{(int)(dt*1000)}, [&]()
    {
        /* Obstacle avoidance */
        if (model->getObsFlag() == 1)
        {
            // Check the closest obstacle
            double dist_obs1 = sqrt(pow(goal_x_obs1(0,0)-pose(0),2) + pow(goal_x_obs1(0,1)-pose(1),2));
            double dist_obs2 = sqrt(pow(goal_x_obs2(0,0)-pose(0),2) + pow(goal_x_obs2(0,1)-pose(1),2));
            if (dist_obs1 < dist_obs2) goal_x_obs = goal_x_obs1;
            else goal_x_obs = goal_x_obs2;
            
            // Check that the vehicle is not already on the obstacle (crash)
            for (size_t i = 0; i < mpc->getNp()+1; i++)
            {
                diff_x = x(i, 0) - goal_x_obs(i, 0);
                diff_y = x(i, 1) - goal_x_obs(i, 1);
                if (diff_x == 0 && diff_y == 0) 
                {
                    root_flag = 0;
                    break;
                }
            }
        }

        /* Start optimization */
        if (root_flag == 1 || model->getObsFlag() == 0)
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
            mpc->setPose(pose);         // set the pose  
            mpc->setGoalX(goal_x);      // set the state goals
            mpc->setGoalU(goal_u);      // set the control goals
            mpc->setObs(goal_x_obs);    // set obstacle goals
            mpc->setObsDim(obs_l, obs_w, obs_l_down, obs_l_right);  // set obstacle box dimensions
            auto [x_sol, u_sol] = mpc->solve(); {}

            /* Optimal Path */
            path_pub->publish(optimPath(x_sol, node->get_clock()->now()));

            /* Send the control input to the robot */
            if (model->getName() == "r2d2")
            {
                twist.linear.x = u_sol(0,0);
                twist.angular.z = u_sol(0,1);
            }
            else 
            {
                twist.linear.x = u_sol(0,0)*cos(beta);
                twist.angular.z = u_sol(0,0)*sin(beta)/model->getParams()[0];
            }
            pub_twist->publish(twist);
            
            /* Publish the front wheel velocity and the steering angle */
            cmd.data[0] = u_sol(0,0);
            cmd.data[1] = u_sol(0,1);
            pub_cmd->publish(cmd);

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
            
            loop++;
            count++;
        }
        else root_flag = 1;
    });

    /* CTRL+C signal handling */
    std::signal(SIGINT, signalHandler);
    
    /* Spin */
    rclcpp::spin(node);
}