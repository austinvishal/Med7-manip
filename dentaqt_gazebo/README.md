# Dentaqt Gazebo Package

This package contains all the Gazebo-specific code related to systems and worlds (based on Gazebo Fortress - Ignition Gazebo 6). Note that there is no optional compatibility provided. This package will compile only if Gazebo Fortress is installed.

## Plugins

- `ApplyForceTorque`: This is a GUI System Plugin that is back-ported from the Gazebo Garden version to Gazebo Fortress. The main changes are related to changing gz -> ign. With this plugin and additionally the back-ported `WrenchVisualizer` class, we are able to support apply force/torque to the links from the GUI.

Note that since hooks are provided to update the Gazebo-related environment variables, once the workspace is built using `colcon build` and the `install/setup.bash` file is sourced, the environment variables are automatically updated to reflect the plugin and resources paths. This way, while running `ign gazebo`, the GUI system plugins are already available in the GUI plugin drop-down menu.

The `config` and `launch` folders are not used. However, we keep them so that `colcon build --symlink-install` does not throw any error.
