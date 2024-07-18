// Modifications made to code from ign_ros2_control
// distributed with the Original License
//
// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DENTAQT_IGN_ROS2_CONTROL_PLUGIN_HPP
#define DENTAQT_IGN_ROS2_CONTROL_PLUGIN_HPP

#include <ignition/gazebo/System.hh>
#include <memory>

namespace dentaqt_ign_ros2_control {
// Forward declarations.
class DentaqtIgnitionROS2ControlPluginPrivate;

class DentaqtIgnitionROS2ControlPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate {
public:
  /// \brief Constructor
  DentaqtIgnitionROS2ControlPlugin();

  /// \brief Destructor
  ~DentaqtIgnitionROS2ControlPlugin() override;

  // Documentation inherited
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

  // Documentation inherited
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

  void
  PostUpdate(const ignition::gazebo::UpdateInfo &_info,
             const ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<DentaqtIgnitionROS2ControlPluginPrivate> dataPtr;
};
} // namespace dentaqt_ign_ros2_control

#endif // DENTAQT_IGN_ROS2_CONTROL_PLUGIN_HPP
