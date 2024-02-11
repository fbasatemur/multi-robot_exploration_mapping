/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

/// SUBSCRIBES:
///     /map (nav_msgs/OccupancyGrid): Reads the map created by SLAM to determine frontiers
   

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>
#include <cmath> 
#include <typeinfo>

#include <string>
#include <thread>
#include <mutex>

using namespace std::literals::string_literals;
std::mutex print_m;

class FrontExpl
{
    public:
        FrontExpl(int tb_num)
        {
            tb3_name = "tb3_"s + std::to_string(tb_num);
            map_frame = tb3_name + "/map";
            body_frame = tb3_name + "/base_footprint";
            map_sub = nh.subscribe("/map", 10, &FrontExpl::mapCallback, this);
            start_flag = true;
            print_info("Started");
        }


        /// \brief Reads the map data published from slam_toolbox
        /// \param msg - map message
        /// \returns nothing
        void mapCallback(const nav_msgs::OccupancyGrid & msg)
        {
            fe_map = msg;
        }

        /// \brief Stores a vector of index values of the 8 cell neighborhood relative to the input cell
        /// \param cell - map cell
        /// \returns nothing
        void neighborhood(int cell)
        {
            // Clear any previous values in the vectors
            neighbor1_index.clear();
            neighbor1_value.clear();

            // Store neighboring values and indexes
            neighbor1_index.push_back(cell - map_width - 1);
            neighbor1_index.push_back(cell - map_width);
            neighbor1_index.push_back(cell - map_width + 1);
            neighbor1_index.push_back(cell-1);
            neighbor1_index.push_back(cell+1);
            neighbor1_index.push_back(cell + map_width - 1);
            neighbor1_index.push_back(cell + map_width);
            neighbor1_index.push_back(cell + map_width + 1);

            sort( neighbor1_index.begin(), neighbor1_index.end() );
            neighbor1_index.erase( unique( neighbor1_index.begin(), neighbor1_index.end() ), neighbor1_index.end() );
        }

        /// \brief Stores a vector of frontier edges
        /// \returns nothing
        void find_all_edges()
        {
            // Starting one row up and on space in on the map so there are no indexing issues
            for (double x = map_width + 1; x < (map_width * map_height) - map_width - 1; x++)
            {
                // For all cells in the map, check if a cell is unknown
                if (fe_map.data.at(x) == -1)
                {
                    // If there is an unknown cell, then check neighboring cells to find a potential frontier edge (free cell)
                    neighborhood(x);

                    for(int i = 0; i < neighbor1_index.size(); i++) // For all neighboring cells
                    {
                        if (fe_map.data.at(neighbor1_index.at(i)) == 0)
                        {
                            // If one of the neighboring cells is free, store it in the edges vector
                            fe_map.data.at(neighbor1_index.at(i)) = 10; // Visualize the frontier edge cells
                            edge1_vec.push_back(neighbor1_index.at(i));

                        }
                    }

                }
            } 
        }

        /// \brief Compares two cells to see if the are identical or unique
        /// \param - curr_cell: The current cell being evaluated
        /// \param - next_cell: The next cell to be evaluated
        /// \returns true or false
        bool check_edges(int curr_cell, int next_cell)
        {
            return curr_cell != next_cell;
        }

        /// \brief Given the frontier edges, group the neighboring edges into regions
        /// \returns nothing
        void find_regions()
        {
            for (int q = 0; q < edge1_vec.size() - 1; q++)
            {
                // For each frontier edge, check that the next value is unique and not a repeat
                unique_flag = check_edges(edge1_vec.at(q), edge1_vec.at(q+1));

                if (unique_flag == true)
                {
                    // If we have an original frontier edge, check the neighboring cells
                    neighborhood(edge1_vec.at(q));

                    for(int i = 0; i < neighbor1_index.size(); i++)
                    {
                        // For all the cells nighrboring the frontier edge, check to see if there is another frontier edge
                        if (neighbor1_index.at(i) == edge1_vec.at(q+1))
                        {
                            // If a frontier edge is in the neightborhood of another frontier edge, add it to a region
                            edge_index = edge1_vec.at(q); 
                            temp_group0.push_back(edge1_vec.at(q));;

                            sort( temp_group0.begin(), temp_group0.end() );
                            temp_group0 .erase( unique( temp_group0.begin(), temp_group0.end() ), temp_group0.end() );

                            // Increase the counter to keep track of the region's size
                            group1_c++;
                        }
                    }

                    if (group1_c == prev_group1_c) // If we didnt any any edeges to our region, region is complete
                    {
                        if (group1_c < 5) // frontier region too small
                        {
                            // If the frontier region is smaller than 5 cells, dont use it
                        }
                        
                        else
                        {
                            // If the frontier region is larger than 5 cells, find the regions centroid
                            centroid0 = (temp_group0.size()) / 2;
                            centroid1_index = temp_group0.at(centroid0);
                            centroids0.push_back(centroid1_index);
                        }

                        // Reset the region vector and size counter
                        group1_c = 0;
                        temp_group0.clear();
                    }

                    else
                    {
                        // If we found a frontier edge, increase the group size
                        prev_group1_c = group1_c;
                    }
                }

                else
                {
                    // If we got a duplicate cell, do nothing
                }

            }
        }

        /// \brief Finds the transform between the map frame and the robot's base_footprint frame
        /// \returns nothing
        void find_transform()
        {
            // Find current location and move to the nearest centroid
            // Get robot pose
            transformS = tfBuffer.lookupTransform(map_frame, body_frame, ros::Time(0), ros::Duration(3.0));

            transformS.header.stamp = ros::Time();
            transformS.header.frame_id = body_frame;
            transformS.child_frame_id = map_frame;

            robot1_pose_.position.x = transformS.transform.translation.x;
            robot1_pose_.position.y = transformS.transform.translation.y;
            robot1_pose_.position.z = 0.0;
            robot1_pose_.orientation = transformS.transform.rotation;

            std::cout << "[ " << tb3_name << " ] Robot pose is  " << robot1_pose_.position.x << " , " << robot1_pose_.position.y << std::endl;
        }

        /// \brief Given a centroid cell, convert the centroid to x-y coordinates in the map frame and determine its distance from the robot
        /// \returns nothing
        void centroid_index_to_point()
        {
            for (int t = 0; t < centroids0.size(); t++)
            {
                // For all the centorid cells, find the x and y coordinates in the map frame
                point.x = (centroids0.at(t) % fe_map.info.width)*fe_map.info.resolution + fe_map.info.origin.position.x;
                point.y = floor(centroids0.at(t) / fe_map.info.width)*fe_map.info.resolution + fe_map.info.origin.position.y;

                for (int w = 0; w < prev_cent_1x.size(); w++)
                {
                    // Compare the previous centroids to the current calculated centroid
                    if ( (fabs( prev_cent_1x.at(w) - point.x) < 0.01) && (fabs( prev_cent_1y.at(w) - point.y) < 0.01) )
                    {
                        // If the current centroid is too close to a previous centroid, skip
                        std::cout << "[ " << tb3_name << " ] Already went to this centroid " << prev_cent_1x.at(w) << " , " << prev_cent_1y.at(w) << std::endl;
                        goto bad_centroid;
                    }
                }

                if((point.x < fe_map.info.origin.position.x + 0.05) && (point.y < fe_map.info.origin.position.y + 0.05))
                {
                    // If the centroid is too close to the map orgin, skip
                    goto bad_centroid;
                }

                else
                {
                    // If the centroid is valid, add its x and y values to their respective vectors
                    centroid1_Xpts.push_back(point.x);
                    centroid1_Ypts.push_back(point.y);

                    // Determine the distance between the current centroid and the robot's position
                    double delta_x = point.x - robot1_pose_.position.x; 
                    double delta_y = point.y - robot1_pose_.position.y; 
                    double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                    dist0 = pow( sum , 0.5 );

                    // Store the distance value in a vector
                    dist1_arr.push_back(dist0);
                }

                // Skip to the end of the for loop if there was an invalid centroid
                bad_centroid: 
                print_info("Ignore bad centroid");
            }
        }

        /// \brief Given all the centroid distance values, find the closest centroid to move to
        /// \returns nothing
        void find_closest_centroid()
        {
            // Set the first smallest distance to be large
            smallest = 9999999.0;
            for(int u = 0; u < dist1_arr.size(); u++)
            {
                // For each centroid distance, determine if the current centroid is closer than the previous closest centroid

                if (dist1_arr.at(u) < 0.1)
                {
                    // If the centroid distance is less than 0.1m, its too close. Ignore it
                }
                else if (dist1_arr.at(u) < smallest)
                {
                    // If the current distance element is smaller than the previous closest centroid
                    // replace the smallest distance variable with the current distance 
                    smallest = dist1_arr.at(u);
                    
                    // Update the centroid that the robot will move to
                    move_to_pt = u;
                }

                else
                {
                    // If the current distance value is larger than 0.1 and the smallest_dist value, then ignore it
                }
            }
        }

        /// \brief Given the frontier edges, convert the cell to x-y coordinates in the map frame and determine its distance from the robot
        /// \returns nothing
        void edge_index_to_point()
        {
            for (int t = 0; t < edge1_vec.size(); t++)
            {
                // For all the frontier edge cells, find the x and y coordinates in the map frame
                point.x = (edge1_vec.at(t) % fe_map.info.width)*fe_map.info.resolution + fe_map.info.origin.position.x;
                point.y = floor(edge1_vec.at(t) / fe_map.info.width)*fe_map.info.resolution + fe_map.info.origin.position.y;

                // Add the cells x and y values to their respective vectors
                centroid1_Xpts.push_back(point.x);
                centroid1_Ypts.push_back(point.y);

                // Determine the distance between the current frontier edge and the robot's position
                double delta_x = point.x - robot1_pose_.position.x; 
                double delta_y = point.y - robot1_pose_.position.y; 
                double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
                dist0 = pow( sum , 0.5 );

                // Store the distance value in a vector
                dist1_arr.push_back(dist0);
            }
        }

        void print_info(const std::string& msg)
        {
            std::lock_guard<std::mutex> lock(print_m);
            std::cout << "[ " << tb3_name << " ] " << msg << std::endl; 
        }

        /// \brief Calls all other functions to find frontier edges, regions and a goal to move to. Then uses the action server to move to that goal
        /// \returns nothing
        void main_loop()
        {
            std::cout << "Entered the main loop for " << tb3_name << std::endl;
            MoveBaseClient ac(tb3_name + "/move_base", true);

            // Wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Connected to move base server");

            tf2_ros::TransformListener tfListener(tfBuffer);
            ros::Rate loop_rate(40);

            // To perform a new explorer target assignment for each robot
            while (ros::ok())
            {
                ros::spinOnce();
                std::cout << "\n| ******************* " << tb3_name << " ******************* |" << std::endl;

                // If there is no map data and / or start service isnt called, do nothing and instead start the loop over again
                if (fe_map.data.size()!=0 && start_flag == true)
                {
                    map_width = fe_map.info.width;
                    map_height = fe_map.info.height;

                    // Find the frontier edges
                    find_all_edges();

                    // If there are no frontier edges, skip to the end of the main_loop and try again
                    if (edge1_vec.size() == 0)
                    {
                        goto skip;
                    }

                    sort( edge1_vec.begin(), edge1_vec.end() );
                    edge1_vec.erase( unique( edge1_vec.begin(), edge1_vec.end() ), edge1_vec.end() );

                    // fe_map_pub.publish(fe_map);
                    // Given the forntier edges, find the frontier regions and their centroids
                    find_regions();

                    // Find the transfrom between the map frame the base_footprint frame
                    // in order to determine the robot's position
                    find_transform();

                    // Given the centroid vector, convert the controids from map cells to x-y coordinates in the map frame
                    // so a goal can be sent to move_base
                    centroid_index_to_point();

                    // If there are no values in the centroid x or y vectors, find the closest frontier edge to move to
                    if ( ( centroid1_Xpts.size() == 0 ) || ( centroid1_Ypts.size() == 0) )
                    {
                        centroid1_Xpts.clear();
                        centroid1_Ypts.clear();
                        dist1_arr.clear();
                        print_info("Couldnt find a centroid, move to closest edge instead");

                        // Given the edge vector, convert the edges from map cells to x-y coordinates in the map frame
                        // so a goal can be sent to move_base                       
                        edge_index_to_point(); 
                    }

                    // Of all the centroids, determine which centroid is the closest
                    // Choose the closest centroid to move to
                    find_closest_centroid();

                    std::cout << "[ " << tb3_name << " ] Moving to centroid " << centroid1_Xpts.at(move_to_pt) << " , " <<  centroid1_Ypts.at(move_to_pt)
                    << ". At a distance of " << smallest << std::endl;

                    // Add the current centroid to the vector of previously visited centroids
                    prev_cent_1x.push_back(centroid1_Xpts.at(move_to_pt));
                    prev_cent_1y.push_back(centroid1_Ypts.at(move_to_pt));

                    // Determine the move_base goal 
                    goal.target_pose.header.frame_id = tb3_name + "/map"; // Needs to be AN ACTUAL FRAME
                    goal.target_pose.header.stamp = ros::Time();
                    goal.target_pose.pose.position.x = centroid1_Xpts.at(move_to_pt);
                    goal.target_pose.pose.position.y = centroid1_Ypts.at(move_to_pt);
                    goal.target_pose.pose.orientation.w = 1.0;

                    print_info("The planning for " + tb3_name + " has been done. Sending goal ...");
                    // Direct the robot towards the target
                    ac.sendGoal(goal);

                    // Wait for the action to return
                    ac.waitForResult(ros::Duration(60));

                    // Did it reach the target ?
                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        print_info("tb3 has reached the goal !");
                    }
                    else
                    {
                        print_info("The base failed for some reason");
                    }
                }

                // If the size of the map data is 0, then run through the loop agai
                else
                {
                    print_info("Waiting for first map");
                }

                // Skip to the end of the loop if there are no frontier regions
                skip:
                sleep(1.0);
                centroid0 = 0;
                centroid1_index = 0;
                dist1_arr.clear();
                edge1_vec.clear();
                neighbor1_index.clear();
                neighbor1_value.clear();
                centroids0.clear();
                centroid1_Xpts.clear();
                centroid1_Ypts.clear();
                ros::spinOnce();
                loop_rate.sleep();
                
                std::cout << "| ***** Asleep and done with main_loop ***** |" << std::endl;
            }
        }

    private:
        std::string tb3_name;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        tf2_ros::Buffer tfBuffer;
        nav_msgs::OccupancyGrid fe_map;
        geometry_msgs::Pose robot1_pose_;
        geometry_msgs::Point point;
        geometry_msgs::TransformStamped transformS;
        move_base_msgs::MoveBaseGoal goal;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        std::string map_frame;
        std::string body_frame;
        std::vector<signed int> edge1_vec, neighbor1_index, neighbor1_value;
        std::vector<unsigned int> centroids0, temp_group0;
        std::vector<double> centroid1_Xpts, centroid1_Ypts, dist1_arr, prev_cent_1x, prev_cent_1y;
        int group1_c=0, prev_group1_c=0, centroid0=0, centroid1_index=0, move_to_pt=0, map_width=0, map_height=0, mark_edge=0, edge_index=0;
        double smallest = 9999999.0, dist0= 0.0;
        bool unique_flag = true;
        bool start_flag = false;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tb3_frontier_explorer_node");
    FrontExpl fe0(0);
    FrontExpl fe1(1);
    FrontExpl fe2(2);
    FrontExpl fe3(3);

    std::thread  t0(&FrontExpl::main_loop, &fe0) ;
    t0.detach();

    std::thread  t1(&FrontExpl::main_loop, &fe1) ;
    t1.detach();

    std::thread  t2(&FrontExpl::main_loop, &fe2) ;
    t2.detach();

    fe3.main_loop();

    ros::spin();

    return 0;
}
