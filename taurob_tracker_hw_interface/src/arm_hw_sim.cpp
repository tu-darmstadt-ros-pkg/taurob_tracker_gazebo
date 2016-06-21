/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  Copyright (c) 2016, Stefan Kohlbrecher, TU Darmstadt
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// This is a copy of default_robot_hw_sim with the only difference being
// that the plugin macro is commented out.


#include <taurob_tracker_hw_interface/arm_hw_sim.h>


namespace tracker_gazebo_ros_control
{


bool ArmHwSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  std::vector<transmission_interface::TransmissionInfo> filtered_transmissions;
  
  for (size_t i = 0; i < transmissions.size(); ++i){

    //const std::string& name = transmissions[i].name_;
    const std::string& name = transmissions[i].joints_[0].name_;


    if (name == "arm_joint_0" || name == "arm_joint_1" || name == "arm_joint_2" || name == "arm_joint_3"){
      filtered_transmissions.push_back(transmissions[i]);
    }
    
  }
  
  return RobotHwSimBase::initSim(robot_namespace,
                          model_nh,
                          parent_model,
                          urdf_model,
                          filtered_transmissions);
}

}

PLUGINLIB_EXPORT_CLASS(tracker_gazebo_ros_control::ArmHwSim, gazebo_ros_control::RobotHWSim)
