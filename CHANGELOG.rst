^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2024-09-19)
------------------
* Changes.
* Add meshes and URDF for robotiq 2f 140
* Standard mesh names and height parameter for tower shoulder
* R100 attachment rework
* Add Dingo plate to generator
* 0.3.0 Release Candidate with Main Changes (`#81 <https://github.com/clearpathrobotics/clearpath_common/issues/81>`_)
  * Added tests
  * Added action to build from release and source
  * Generator linting erros
  * Customization linting errors
  * Linting
  * Fix: Remove IP address from discovery server launch so it listens on all NICs
  * Changes.
  * 0.2.8
  * Add sysctl config file that changes ipfrag settings to support receiving large messages
  * Added Zed URDF
  * Added Zed to description generator
  * Modified common parameter generation to always flatten
  * Changes.
  * 0.2.9
  * Missing important remapping to mirror hardware topics
  * Added topic to gazebo plugins
  * Updated topic names to match gazebo message types
  * Topics of simulated onboard sensors
  * Realsense adds optical links when in simulator
  * Changes.
  * 0.2.10
  * Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
  * Fixes styling issues
  * Set spawner as super client
  * Changes.
  * 0.2.11
  * Removed duplicate class
  * Use ROS1 covariance values
  * Updated renamed macanum drive controller
  * Enable gazebo friction plugin on DingoO
  ---------
  Co-authored-by: Hilary Luo <hluo@clearpathrobotics.com>
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
  Co-authored-by: Steve Macenski <stevenmacenski@gmail.com>
  Co-authored-by: robbiefish <rob.fisher@hbkworld.com>
* 0.2.8
* Changes.
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* 0.2.7
* Changes.
* ARM_MOUNT to ARM_PLATE
* Linting issues
* Use if statement
* Fixed all license headers
* Fixed linting issues of collision updater node
* Pass parameters to Kinova URDF
* Updated generators to deal with grippers as part of arms
* Create control file for manipulator controller manager
* Only add manipulator controllers if simulation
* Added virtual method for manipulator launch generation
* Added semantic description generator
* Added manipulators to parameter generator
* Add manipulators to description generator
* Modifications to allow arms to function
* Added simple package writer to copy package from template
* Check terminal to set ROS_SUPER_CLIENT
* Generate script to start the discovery server
* Updated setup.bash generation for discovery server
* 0.2.6
* Changes.
* 0.2.5
* Changes.
* switch finding meshes to use the package:// command
* 0.2.4
* Changes.
* [clearpath_generator_common] Added package description.
* 0.2.3
* Changes.
* Handle file paths with no directory (files in root directory of the package)
* 0.2.2
* Changes.xx
* Enable extras urdf and meshes to be linked by package (`#53 <https://github.com/clearpathrobotics/clearpath_common/issues/53>`_)
* 0.2.1
* Changes.
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski, luis-camero

* Add meshes and URDF for robotiq 2f 140
* Standard mesh names and height parameter for tower shoulder
* R100 attachment rework
* Add Dingo plate to generator
* Added tests
* Added action to build from release and source
* Generator linting erros
* Customization linting errors
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* Add sysctl config file that changes ipfrag settings to support receiving large messages
* Added Zed URDF
* Added Zed to description generator
* Modified common parameter generation to always flatten
* Missing important remapping to mirror hardware topics
* Added topic to gazebo plugins
* Updated topic names to match gazebo message types
* Topics of simulated onboard sensors
* Realsense adds optical links when in simulator
* Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
* Fixes styling issues
* Set spawner as super client
* Removed duplicate class
* Use ROS1 covariance values
* Updated renamed macanum drive controller
* Enable gazebo friction plugin on DingoO
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski, luis-camero

0.2.11 (2024-08-08)
-------------------
* Fixes styling issues
* Modifies platform param to add GQ7 IMU data to ekf_localization and adds GQ7 URDF
* Contributors: robbiefish

0.2.10 (2024-07-25)
-------------------

0.2.9 (2024-05-28)
------------------
* Modified common parameter generation to always flatten
* Added Zed to description generator
* Add sysctl config file that changes ipfrag settings to support receiving large messages
* Linting
* Generator linting erros
* Added tests
* Contributors: Hilary Luo, Luis Camero

0.2.8 (2024-05-14)
------------------
* Fix: Remove IP address from discovery server launch so it listens on all NICs
* Contributors: Hilary Luo

0.2.7 (2024-04-08)
------------------
* ARM_MOUNT to ARM_PLATE
* Added simple package writer to copy package from template
* Check terminal to set ROS_SUPER_CLIENT
* Generate script to start the discovery server
* Updated setup.bash generation for discovery server
* Contributors: Hilary Luo, Luis Camero

0.2.6 (2024-01-18)
------------------

0.2.5 (2024-01-15)
------------------
* switch finding meshes to use the package:// command
* Contributors: Hilary Luo

0.2.4 (2024-01-11)
------------------
* [clearpath_generator_common] Added package description.
* Contributors: Tony Baltovski

0.2.3 (2024-01-08)
------------------
* Handle file paths with no directory (files in root directory of the package)
* Contributors: Hilary Luo

0.2.2 (2024-01-04)
------------------
* Enable extras urdf and meshes to be linked by package (`#53 <https://github.com/clearpathrobotics/clearpath_common/issues/53>`_)
* Contributors: Hilary Luo

0.2.1 (2023-12-21)
------------------

0.2.0 (2023-12-08)
------------------
* Added wheel parameters to all robot
* Wheel is now parameter
* Adds Blackfly camera to sensor description (`#33 <https://github.com/clearpathrobotics/clearpath_common/issues/33>`_)
  * Adds Blackfly camera to sensor description
  ---------
  Co-authored-by: fazzrazz <danielduranrojas@gmail.com>
* Removed print in platform description generator
* Add imu0 to ekf_node for all platforms except A200
* Added W200 attachments to generator
* Platform no longer required
* Added  to materials
* Removed unecessary SimpleDescription
* Attachments not restricted by platform
* Simplified attachment generation
* Removed debug print
* Removed gazebo include from generator
* Read control.yaml directly from clearpath config specified file
* Allow for no macro to be added
* Moved gazebo controller to common
* Added Generic platform
* Contributors: Hilary Luo, Luis Camero, Roni Kreinin

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-02)
------------------
* Adds Blackfly camera to sensor description (`#33 <https://github.com/clearpathrobotics/clearpath_common/issues/33>`_)
  * Adds Blackfly camera to sensor description
  ---------
  Co-authored-by: fazzrazz <danielduranrojas@gmail.com>
* Contributors: Hilary Luo

0.1.1 (2023-08-25)
------------------

0.1.0 (2023-08-17)
------------------
* Removed joy_teleop namespace, remap topics to that namespace instead
* Added fenders for J100
* Renamed UST10 to UST
  Added parameter node list
* Removed disk import
* Added disk and post
  Set default values to model dictionaries
* Inverted and upright sick stand
* Added UM6/7
* Contributors: Roni Kreinin

0.0.9 (2023-07-31)
------------------
* Added Garmin 18x, Novatel smart 6 and 7
* Update platform nodes from extra ros parameters
  Flattened default parameter files
* Contributors: Roni Kreinin

0.0.8 (2023-07-24)
------------------
* Linting
* use_sim_time support
* Description and Bash generator cleanup
* Minor cleanup
* Param generator
* Launch generator cleanup
* Contributors: Roni Kreinin

0.0.7 (2023-07-19)
------------------
* Renamed description to attachments
* Rnamed accessories to links
* Contributors: Luis Camero

0.0.6 (2023-07-13)
------------------
* Merge pull request `#18 <https://github.com/clearpathrobotics/clearpath_common/issues/18>`_ from clearpathrobotics/updated-config
  Updated common generators to match config
* Fixed getters
* Updated common generators to match config
* Contributors: Luis Camero, Roni Kreinin

0.0.5 (2023-07-12)
------------------

0.0.4 (2023-07-07)
------------------

0.0.3 (2023-07-05)
------------------
* Linters
* Updated localization configs
* Updated husky track value
* Wheel slip plugin
  Significantly improved jackal odom in sim
* Contributors: Roni Kreinin

0.0.2 (2023-07-04)
------------------

0.0.1 (2023-06-21)
------------------
* Updated launch writer make writing different object types easier
  Localization parameter fixes
  Updated gazebo wheel friction
* Added namespacing support
* Updated dependencies
* Added clearpath_generator_common
  Moved clearpath_platform to clearpath_common
  Fixed use_sim_time parameter issue with ekf_node
* Contributors: Roni Kreinin
