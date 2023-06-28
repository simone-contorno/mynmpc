#include <mynmpc/utils.h>

/*!
 * Normalize the angle in the range [-pi, pi].
 * @param angle angle to normalize.
 */
void normalizeAngle(double &angle)
{
    angle = atan2(sin(angle), cos(angle));
} 

/*!
 * Compute the box around the object as a vector of points.
 * @param pose current vehicle pose.
 * @param l vehicle length.
 * @param w vehicle width.
 * @param l_down distance bewteen vehicle pose and back side.
 * @param l_right distance between vehicle pose and right side.
 * @param points number of points for each side (precision).
 */
MatrixXd computeBox(VectorXd pose, double l, double w, double l_down, double l_right, size_t points)
{
    points++;

    /* Compute the middle-down point */
    Eigen::Vector2d middle_down; 
    middle_down <<  pose(0) - l_down * cos(pose(2)), 
                    pose(1) - l_down * sin(pose(2));

    /* Compute the four corners */ 
    Eigen::Vector2d right_down;
    right_down  <<  middle_down(0) + l_right * cos(pose(2) - M_PI/2), 
                    middle_down(1) + l_right * sin(pose(2) - M_PI/2);

    /* Compute the box points */
    MatrixXd box = MatrixXd::Zero(4 * points + 1, 2);
    size_t i = 0; 
    double step;
    
    /* First point (rigt-down) */
    box(0,0) = right_down(0);
    box(0,1) = right_down(1);
    i++;

    /* Right side */
    step = l / points;
    for (size_t j = 0; j < points; j++)
    {
        box(i,0) = box(i-1,0) + step * cos(pose(2));
        box(i,1) = box(i-1,1) + step * sin(pose(2));
        i++;  
    }

    /* Up side */
    step = w / points;
    for (size_t j = 0; j < points; j++)
    {
        box(i,0) = box(i-1,0) + step * cos(pose(2) + M_PI/2);
        box(i,1) = box(i-1,1) + step * sin(pose(2) + M_PI/2);
        i++;  
    }

    /* Left side */
    step = l / points;
    for (size_t j = 0; j < points; j++)
    {
        box(i,0) = box(i-1,0) - step * cos(pose(2));
        box(i,1) = box(i-1,1) - step * sin(pose(2));
        i++;  
    }
    
    /* Down side */
    step = w / points;
    for (size_t j = 0; j < points; j++)
    {
        box(i,0) = box(i-1,0) + step * cos(pose(2) - M_PI/2);
        box(i,1) = box(i-1,1) + step * sin(pose(2) - M_PI/2);
        i++;  
    } 

    return box;
}

/*!
 * Compute the closest points between two boxes.
 * @param box_1 first box.
 * @param box_2 second box.
 */
std::tuple<size_t, size_t> closestBoxPoints(MatrixXd box_1, MatrixXd box_2)
{
    /* Initialize first minimum distance */
    double min_dist = sqrt(pow(box_1(0,0)-box_2(0,0),2) + pow(box_1(0,1)-box_2(0,1),2));
    double dist = 0.;

    /* Initialize the box indeces to return */
    size_t box_1_idx = 0;
    size_t box_2_idx = 0;

    /* Compare all the points to find the minum one */
    for (size_t i = 0; i < box_1.rows(); i++)
    {
        for (size_t j = 0; j < box_2.rows(); j++)
        {
            /* Compute Euclidean distance */
            dist = sqrt(pow(box_1(i,0)-box_2(j,0),2) + pow(box_1(i,1)-box_2(j,1),2));
            
            /* Check if the distance is less than the current minimum one */
            if (dist < min_dist) 
            {
                min_dist = dist;
                box_1_idx = i;
                box_2_idx = j;
            }
        }
    }

    return {box_1_idx, box_2_idx};
}

/*!
 * Get the optimal path computed by the MPC.
 * @param x optimal states.
 * @param now current clock time.
 */
Path optimPath(MatrixXd x, const rclcpp::Time &now) 
{
    Path path;

    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.;

    for (size_t i = 0; i < x.rows(); i++)
    {
        pose.pose.position.x = x.row(i)[0];
        pose.pose.position.y = x.row(i)[1];
        path.poses.push_back(pose);
    }

    for (auto &pose: path.poses) 
        pose.header.stamp = now;

    return path;
}