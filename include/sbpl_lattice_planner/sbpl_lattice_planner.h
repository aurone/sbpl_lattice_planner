#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

//global representation
#include <nav_core/base_global_planner.h>

namespace sbpl_lattice_planner{

class SBPLLatticePlanner : public nav_core::BaseGlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlanner();

  
  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief Given a path compute the cost
   * @param _plan A vector of PoseStamped objects
   * @return The total cost of the path (according to costmap_ros_), -1 if the path is invalid
   */
  int checkCost(std::vector<geometry_msgs::PoseStamped> _plan);


  /**
   * @brief Given the last goal and the current goal return true if they are very similar
   * @param _last_goal The last goal pose
   * @param _new_goal The new goal pose
   * @return True if the goals are similar, otherwise false
   */
  bool compareGoals(geometry_msgs::PoseStamped _last_goal, geometry_msgs::PoseStamped _new_goal);


  /**
   * @brief Given the last starting pose return true if still near the last path
   * @param _last_start The pose of the last starting point
   * @return True if the last start pose is still close to the path, otherwise false
   */
  bool compareStartToPath(geometry_msgs::PoseStamped _last_start);
  
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~SBPLLatticePlanner(){};

private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETALAT* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;

  geometry_msgs::PoseStamped last_start_;
  geometry_msgs::PoseStamped last_goal_;
  std::vector<geometry_msgs::PoseStamped> last_plan_;
  bool has_prev_;

  // Parameters
  bool check_replan_;
  int lethal_cost_;
  double replan_margin_;
  double goal_x_tol_;
  double goal_y_tol_;
  double goal_t_tol_;
  double start_x_tol_;
  double start_y_tol_;
  double start_t_tol_;
  //

  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */

  ros::Publisher plan_pub_;
  ros::Publisher stats_publisher_;
  
  std::vector<geometry_msgs::Point> footprint_;

};
};

#endif

