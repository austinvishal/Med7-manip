# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

### Added

- Backport `ApplyForceTorque` GUI System Plugin and `WrenchVisualizer` rendering class to `dentaqt_gazebo` package. 
- Add modified `ign_ros2_control` package with Igntion ROS2 control system modified to include force-torque sensing and correct joint torque computations  
- Import LBR Med7 robot model specific to dentaqt project using `dentaqt_ign_ros2_control` plugin to load custom modified Igntiion-ROS2 bridge for simulation 
- Add  dentaqt controllers implementing gravity compensation and cartesian impedance control 



### Removed

### Changed
