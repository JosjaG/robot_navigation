/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <dlux_global_planner/dlux_global_planner.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <nav_2d_utils/tf_help.h>
#include <nav_2d_utils/path_ops.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace dlux_global_planner
{
DluxGlobalPlanner::DluxGlobalPlanner() :
  calc_loader_("dlux_global_planner", "dlux_global_planner::PotentialCalculator"),
  traceback_loader_("dlux_global_planner", "dlux_global_planner::Traceback"),
  potential_grid_(HIGH_POTENTIAL), cached_goal_x_(-1), cached_goal_y_(-1), potential_pub_(potential_grid_)
{
}

void DluxGlobalPlanner::initialize(const ros::NodeHandle& parent, const std::string& name,
                                   TFListenerPtr tf, nav_core2::Costmap::Ptr costmap)
{
  ros::NodeHandle planner_nh(parent, name);
  costmap_ = costmap;
  potential_grid_.setInfo(costmap_->getInfo());

  cost_interpreter_ = std::make_shared<CostInterpreter>();
  cost_interpreter_->initialize(planner_nh, costmap_);

  std::string plugin_name;
  planner_nh.param("potential_calculator", plugin_name, std::string("dlux_plugins::AStar"));
  ROS_INFO_NAMED("DluxGlobalPlanner", "Using PotentialCalculator \"%s\"", plugin_name.c_str());
  calculator_ = calc_loader_.createInstance(plugin_name);
  calculator_->initialize(planner_nh, costmap, cost_interpreter_);

  planner_nh.param("traceback", plugin_name, std::string("dlux_plugins::GradientPath"));
  ROS_INFO_NAMED("DluxGlobalPlanner", "Using Traceback \"%s\"", plugin_name.c_str());
  traceback_ = traceback_loader_.createInstance(plugin_name);
  traceback_->initialize(planner_nh, cost_interpreter_);

  planner_nh.param("path_caching", path_caching_, false);
  planner_nh.param("improvement_threshold", improvement_threshold_, -1.0);
  cached_path_cost_ = -1.0;

  bool publish_potential;
  planner_nh.param("publish_potential", publish_potential, false);
  if (publish_potential)
    potential_pub_.init(planner_nh, "potential_grid", "potential");

  interp_sub_ = planner_nh.subscribe<geometry_msgs::PoseStamped>("/dory/interpolator_pose", 1,  //UGLY HARDCODED NAMESPACE
                                                         boost::bind(&DluxGlobalPlanner::interpCB, this, _1));
  replan_pub_ = planner_nh.advertise<geometry_msgs::Pose2D>("replan", 1000);
  max_interp_distance_=1.25;

  planner_nh.param("print_statistics", print_statistics_, false);
}

void DluxGlobalPlanner::interpCB(const geometry_msgs::PoseStamped::ConstPtr& interp)
  {
    interp_ = nav_2d_utils::poseStampedToPose2D(*interp);
    // publishPointMarker(interp_, false);
    // plan();
  }

bool DluxGlobalPlanner::isPlanValid(const nav_2d_msgs::Path2D& path) const
{
  // NB: Only checks for obstacles at each pose
  unsigned int x, y;
  nav_core2::Costmap& costmap = *costmap_;
  const nav_grid::NavGridInfo& info = costmap.getInfo();
  for (geometry_msgs::Pose2D pose : path.poses)
  {
    if (!worldToGridBounded(info, pose.x, pose.y, x, y) || costmap(x, y) >= costmap.INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
  }
  return true;
}

nav_2d_msgs::Path2D DluxGlobalPlanner::makePlan(const nav_2d_msgs::Pose2DStamped& start,
                                                const nav_2d_msgs::Pose2DStamped& goal)
{
  if (potential_grid_.getInfo() != costmap_->getInfo())
    potential_grid_.setInfo(costmap_->getInfo());

  dist_interp_ = sqrt(pow(start.pose.x-interp_.pose.x,2)+pow(start.pose.y-interp_.pose.y,2));
  if (dist_interp_>max_interp_distance_)
    {
    if (print_statistics_)
      {
      ROS_INFO_NAMED("DluxGlobalPlanner",
                   "USING BASE LINK, dist = %.2f .", dist_interp_);
      }
    temp_start_.pose.x = start.pose.x;
    temp_start_.pose.y = start.pose.y;
    } else {
      if (print_statistics_)
        {
        ROS_INFO_NAMED("DluxGlobalPlanner",
                   "USING INTERP NODE, dist = %.2f .", dist_interp_);
        }
      temp_start_.pose.x = interp_.pose.x;
      temp_start_.pose.y = interp_.pose.y;
    }
    temp_start_.header = start.header;
  
  // start = temp_start_;
  geometry_msgs::Pose2D local_start = nav_2d_utils::transformStampedPose(tf_, temp_start_, potential_grid_.getFrameId());
  geometry_msgs::Pose2D local_goal = nav_2d_utils::transformStampedPose(tf_, goal, potential_grid_.getFrameId());

  replan_pub_.publish(local_start);

  nav_core2::Costmap& costmap = *costmap_;
  const nav_grid::NavGridInfo& info = costmap.getInfo();

  // Check Start / Goal Quality
  unsigned int x, y;
  if (!worldToGridBounded(info, local_start.x, local_start.y, x, y))
  {
    cached_path_cost_ = -1.0;
    throw nav_core2::StartBoundsException(temp_start_);
  }
  if (costmap(x, y) >= costmap.INSCRIBED_INFLATED_OBSTACLE)
  {
    cached_path_cost_ = -1.0;
    throw nav_core2::OccupiedStartException(temp_start_);
  }
  if (!worldToGridBounded(info, local_goal.x, local_goal.y, x, y))
  {
    cached_path_cost_ = -1.0;
    throw nav_core2::GoalBoundsException(goal);
  }
  if (costmap(x, y) >= costmap.INSCRIBED_INFLATED_OBSTACLE)
  {
    cached_path_cost_ = -1.0;
    throw nav_core2::OccupiedGoalException(goal);
  }

  bool cached_plan_available = false;
  if (path_caching_ && hasValidCachedPath(local_goal, x, y))
  {
    if (shouldReturnCachedPathImmediately())
    {
      return cached_path_;
    }
    cached_plan_available = true;
  }

  // Commence path planning.
  unsigned int n_updated = calculator_->updatePotentials(potential_grid_, local_start, local_goal);
  potential_pub_.publish();
  double path_cost = 0.0;  // right now we don't do anything with the cost
  nav_2d_msgs::Path2D path = traceback_->getPath(potential_grid_, temp_start_.pose, goal.pose, path_cost);
  if (print_statistics_)
  {
    ROS_INFO_NAMED("DluxGlobalPlanner",
                   "Got plan! Cost: %.2f, %d updated potentials, path of length %.2f with %zu poses.",
                   path_cost, n_updated, nav_2d_utils::getPlanLength(path), path.poses.size());
  }

  // If there is a cached path available and the new path cost has not sufficiently improved
  if (cached_plan_available && !shouldReturnNewPath(path, path_cost))
  {
    return cached_path_;
  }
  cached_path_cost_ = path_cost;
  cached_path_ = path;
  return path;
}


bool DluxGlobalPlanner::hasValidCachedPath(const geometry_msgs::Pose2D& local_goal,
                                           unsigned int goal_x, unsigned int goal_y)
{
  bool ret = cached_path_cost_ >= 0 && cached_goal_x_ == goal_x && cached_goal_y_ == goal_y &&
             isPlanValid(cached_path_);
  cached_goal_x_ = goal_x;
  cached_goal_y_ = goal_y;
  return ret;
}

bool DluxGlobalPlanner::shouldReturnCachedPathImmediately() const
{
  // If we don't care if the plan improves, return immediately.
  return improvement_threshold_ < 0.0;
}

bool DluxGlobalPlanner::shouldReturnNewPath(const nav_2d_msgs::Path2D& new_path, const double new_path_cost) const
{
  return new_path_cost + improvement_threshold_ <= cached_path_cost_;
}

PLUGINLIB_EXPORT_CLASS(dlux_global_planner::DluxGlobalPlanner, nav_core2::GlobalPlanner)

}  // namespace dlux_global_planner
