# ROS Maritime Robotics Working Group

This document defines the scope and governance of the Working Group (WG).

Mission: To unite, create and upgrade generic marine robotics solutions that can be easily deployed in marine robots.

Scope: ROS packages related to marine robotics specificities of any robotics field: controls, simulation, drivers...
All marine robot types are included, whether they are USV (Uncrewed Surface Vehicle), UUV (Uncrewed Underwater Vehicles), underwater manipulators...

## Governance

### Meetings

Everyone is welcome to attend the Working Group meetings.
Just show up.

* Regular WG Meeting: Usually once a month on a Tuesday, at 8AM PST. Subjected to change.
  * Meetings Announcement: Usually at least one week before the meeting in the [Maritime Robotics category](https://discourse.ros.org/c/maritime/36) on [ROS Discourse](https://discourse.ros.org/)
  * Meeting agendas and minutes: [Google Doc](https://docs.google.com/document/d/1Wnddq4xRXR6HF2XFWeejfUGII_hj7DilKrGcFj1qlEA/edit?usp=drive_link)
  * Outputs of the meetings: Recording as reply to the meeting announcement post

### Communication Channels

* [ROS Discourse Maritime Robotics category](https://discourse.ros.org/c/maritime/36)
  * Discourse tag: [wg-maritime-robotics](https://discourse.ros.org/tag/wg-maritime-robotics)
* Github organization: [ros-maritime](https://github.com/ros-maritime)
  * [Project board](https://github.com/orgs/ros-maritime/projects/1)
* Google Group (only used to send meeting calendar invites): [Meeting invite group](https://groups.google.com/g/maritime-robotics-working-group-invites)
* Instant messaging: [Matrix chat](https://matrix.to/#/#ros-maritime:matrix.org) (Matrix is an open network for secure, decentralized communication).

### Participation and Organization

Working Group participants may act in one or more of the following roles:

* Participants / Contributors
  * Attend meetings when convenient
  * All attendees are welcome to contribute and review pull requests, and respond to issues, including but not limited to tickets in the [project board]((https://github.com/orgs/ros-maritime/projects/1))
* Maintainers
  * Prerequisite: Proven track record of high-quality contributions and reviews to repositories related to WG
  * Responsible for approving and merging pull requests
* Organizers
  * Responsible for organizing and moderating working group meetings
  * Responsible for posting meeting materials (minutes, recordings, etc.)
  * Responsible for breaking ties

### Modifying this governance document

Changes to this document will be made via Pull Request.
The PR will be merged on 2 agreement from Approvers.

## Reference projects

The following projects may be generally useful for those working in marine
robotics with ROS.

Some projects are affiliated with the Working Group, and their progress may be
discussed in WG meetings.
The [Project board](https://github.com/orgs/ros-maritime/projects/1) keeps
track of a subset of those projects.

### Projects List

The list is not meant to be exhaustive.

* Communication
  * Description: Packages for underwater network wireless communication.
  * Motive: Since underwater communications relies on acoustic waves, there are some specificities regarding these types of waves that should be taken into account during data transmission.
  * References
    * [ns UAN](https://www.nsnam.org/docs/models/html/uan.html)
    * [WHOI `ros_acomms`](https://git.whoi.edu/acomms/ros_acomms)

* Common Messages
  * Description: Specific ROS messages for maritime robotics.
  * Motive: There are maritime robotics sensors that give specific types of messages.
  * References
    * [UW APL Hydrographic Messages](https://github.com/apl-ocean-engineering/hydrographic_msgs)

* Drivers
  * Description: Documentation and index for underwater and surface vehicles sensors and actuators drivers.
  * Motive: There are specific components used only for maritime robotics, which requires drivers to interact with them. Although, there already is an ROS organization to develop ROS drivers, so instead of developing inside this organization, we would use it just to have an index describing maritimate focused ROS drivers.
  * Proposal
    * **[Proposal]** ROS Maritime Robotics Drivers Index
    * **[Proposal]** Sonar and radar drivers
  * References
    * [ROS Drivers](https://github.com/ros-drivers)

* Documentation
  * Description: Documentation and index for maritime robotics packages and organizations.
  * Motive: There are already many organizations and repos for maritime robotics. It is important to map what it is already developed in the area as well as make tutorials for newcomers or people that want to contribute with maritime robotics development.
  * Proposal
    * **[Proposal]** ROS Maritime Robotics Documentation Index
    * **[Proposal]** ROS Maritime Robotics Tutorials
  * References
    * Marine-specific software architecture
      * [COLA2](https://iquarobotics.com/cola2): ROS and Gazebo

* Dynamic Model
  * Description: Packages related to estimation and iteration of maritime robots models.
  * Motive: Maritime robots dynamical models have to take into account buoyancy, hydrodynamics... Therefore some packages regarding math models are needed.
  * Proposal
    * **[Proposal]** Maritime vehicle model estimation based experiments
      * Use control, pose and velocity bags and URDF to estimate the Fossen equation of motion parameters
    * **[Proposal]** Maritime vehicle model iterator
      * Use vehicle equations of motion to predict the robot's next position and velocity. This can be used with localization and control algorithms.

* Guidance, Navigation and Control (GNC)
  * Description: Packages for GNC of maritime robots.
  * Motive: There are some controls and tasks specifics to maritime robots, such as thruster control allocation, surveillance, vessels' handling of tides...
  * Proposal
    * **[Proposal]** Thruster manager
  * References
    * [Plankton thruster manager](https://github.com/Liquid-ai/Plankton/blob/master/uuv_control/uuv_thruster_manager/src/uuv_thrusters/thruster_manager.py)

* Localization
  * Description: Packages to localize maritime vehicles.
  * Motive: Underwater and surface vehicles have a lot of differences regarding the sensors they can use for localizing themselves. GPS does not work underwater, as well as lidars or electromagnetic waves for long distances... But acoustic waves work pretty well, so underwater localization rely more on sensors like DVL and Sonar, that uses acoustic waves to measure distance and velocity.
  * Proposal
    * **[Proposal]**: Multilateration of acoustic beacons
  * References
    * [Mesh Navigation](https://github.com/uos/mesh_navigation)
    * [Fuse Localization](https://github.com/uos/mesh_navigation)
    * [MagNav](https://www.gpsworld.com/us-air-force-to-explore-navigating-with-magnetism/)

* Perception
  * Description: Packages for perception algorithms for maritime robotics.
  * Motive: Since there are different sensors such as sonars, there is a need to process this data to obtain information. Moreover, it can be created some common structures detection.
  * Proposal
    * **[Proposal]**: Common Maritime Environments Detections (buoys, valves, pipes...)
    * **[Proposal]**: Fusion of sonar and camera data

* Robots
  * Description: Packages with maritime robots and structures.
  * Motive: All the software being developed shall have not only unit tests but integration tests, so robots and structures shall be developed for this purpose. These shall include vcstool for specifying packages to be used with the robot as well as ros_ign_bridge to test it in simulation.
  * Proposal
    * **[Proposal]** Hover type UW vehicle
    * **[Proposal]** Torpedo shaped UW vehicle
    * **[Proposal]** USV
    * **[Proposal]** Intervention UW vehicle
  * References
    * Intervention UW sample [RexROV2](https://uuvsimulator.github.io/packages/rexrov2/intro/)
    * Torpedo Shaped UW vehicle [MBARI LRAUV](https://github.com/osrf/lrauv/)

* Simulation
  * Description: Documentation and index for maritime robotics Gazebo plugins.
  * Motive: There is no need to mix ROS code with simulation, also there is already some maritime robotics development in new Gazebo, so it is a good thing to contribute to ign-gazebo repo instead of creating other in this organization.
  * Proposal
    * **[Proposal]** Maritime Robotics Gazebo Plugins Index
    * **[Proposal]** Wave Simulation
    * **[Proposal]** Acoustics Simulation (include salinity and temperature effects)
    * **[Proposal]** Radar Simulation
    * **[Proposal]** Sonar Simulation
  * References
    * General
      * [Gazebo-classic](https://classic.gazebosim.org/)
      * [Gazebo (formerly Ignition)](https://github.com/gazebosim/gz-sim)
    * Surface
      * [ASV Wave Simulator](https://github.com/srmainwaring/asv_wave_sim)
      * [Virtual RobotX (VRX) competition](https://github.com/osrf/vrx): ROS and Gazebo-classic / new Gazebo
      * [MBZIRC Maritime Grand Challenge Simulator](https://github.com/osrf/mbzirc): ROS 2 and Gazebo
      * [MBARI Wave Energy Converter Simulator](https://github.com/osrf/mbari_wec): ROS 2 and Gazebo
      * Sailboat and wind simulation of [Racing Sparrow 750](https://github.com/srmainwaring/rs750)
    * Underwater
      * [UUV Simulator](https://uuvsimulator.github.io/): ROS and Gazebo-classic
      * [NPS Project DAVE](https://github.com/Field-Robotics-Lab/dave/wiki): ROS and Gazebo-classic
      * [Plankton](https://github.com/Liquid-ai/Plankton): ROS 2 and Gazebo-classic
      * [Gazebo native underwater vehicles](https://gazebosim.org/api/gazebo/6.4/underwater_vehicles.html): Gazebo
      * [MBARI LRAUV simulation](https://github.com/osrf/lrauv/): Gazebo
      * [WHOI Deep Submergence Lab ds_sim](https://bitbucket.org/whoidsl/ds_sim/src/master/): ROS and Gazebo-classic

* Tools
  * Description: Extra category for packages that do something specific that don't fit in any other category
  * Motive: There might be some tools that can be developed for maritime robotics that doesn't fit another category, such as geographic information convertion.
  * Proposal
    * **[Proposal]** Geographic convertions

* Data sets
  * Description: Marine-specific data sets
  * Motive: Underwater sensors differ a great deal from land sensors, and data sets obtained using underwater sensors are necessary for realistic underwater algorithmic development.
  * References
    * [LearnOpenCV tutorial on Underwater Trash Detection](https://learnopencv.com/yolov6-custom-dataset-training/)
