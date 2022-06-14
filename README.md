# ROS Maritime Robotics Working Group

This document defines the scope and governance of the Working Group (WG).

Mission: To unite, create and upgrade generic maritime robotics solutions that can be easily deployed in maritime robots. 

Scope: ROS packages related to maritime robotics specificities of any robotics field: control, simulation, drivers... All maritime robots types are included, whether they are USV (Ummaned Surface Vehicle), AUV (Autonomous Underwater Vehicles), underwater manipulators...

## Subprojects

This Working Group owns and maintains the following Subprojects.
Its meetings and membership are largely focused on the direction, design, and work on the projects.

### Subproject List

The following subprojects are owned by the Working Group:

* Communication
  * Description: Packages for underwater network wireless communication.
  * Motive: Since underwater communications relies on acoustic waves, there are some specificities regarding these types of waves that should be taken into account during data transmission.
  * Repositories
    * Suggestions...
  * References: 
    *  [ns UAN](https://www.nsnam.org/docs/models/html/uan.html)

* Common Messages
  * Description: Specific ROS messages for maritime robotics.
  * Motive: There are maritime robotics sensors that give specific types of messages.
  * Unique Repo
    *  Assist the development of [UW APL Hydrographic Messages](https://github.com/apl-ocean-engineering/hydrographic_msgs)

* Drivers
  * Description: Documentation and index for underwater and surface vehicles sensors and actuators drivers.
  * Motive: There are specific components used only for maritime robotics, which requires drivers to interact with them. Although, there already is an ROS organization to develop ROS drivers, so instead of developing inside this organization, we would use it just to have an index describing maritimate focused ROS drivers.
  * Unique Repo
    * **[Proposal]** ROS Maritime Robotics Drivers Index
    * **[Proposal]** Sonar and radar drivers
  * References: 
    *  [ROS Drivers](https://github.com/ros-drivers)

* Documentation
  * Description: Documentation and index for maritime robotics packages and organizations.
  * Motive: There are already many organizations and repos for maritime robotics. It is important to map what it is already developed in the area as well as make tutorials for newcomers or people that want to contribute with maritime robotics development.
  * Repositories
    * **[Proposal]** ROS Maritime Robotics Documentation Index
    * **[Proposal]** ROS Maritime Robotics Tutorials
  * References: 
    * [COLA2](https://iquarobotics.com/cola2): ROS and Gazebo
    * [Virtual RobotX (VRX)](https://github.com/osrf/vrx): ROS and Gazebo
    * [UUV Simulator](https://uuvsimulator.github.io/): ROS and Gazebo
    * [Plankton](https://github.com/Liquid-ai/Plankton): ROS2 and Gazebo
    * [Ignition native underwater vehicles](https://ignitionrobotics.org/api/gazebo/6.4/underwater_vehicles.html): Ignition 
    * [MBARI LRAUV simulated in Ignition](https://github.com/osrf/lrauv/): Ignition 
    * [NPS Project DAVE](https://github.com/Field-Robotics-Lab/dave/wiki): ROS and Gazebo
    * [WHOI Deep Submergence Lab ds_sim](https://bitbucket.org/whoidsl/ds_sim/src/master/): ROS and Gazebo
    * [UW APL Hydrographic Messages](https://github.com/apl-ocean-engineering/hydrographic_msgs): ROS
  
* Dynamic Model
  * Description: Packages related to estimation and iteration of maritime robots models.
  * Motive: Maritime robots dynamical models have to take into account buoyancy, hydrodynamics... Therefore some packages regarding math models are needed. 
  * Repositories
    * **[Proposal]** Maritime vehicle model estimation based experiments
      * Use control, pose and velocity bags and URDF to estimate the Fossen equation of motion parameters
    * **[Proposal]** Maritime vehicle model iterator
      * Use vehicle equations of motion to predict the robot's next position and velocity. This can be used with localization and control algorithms.

* Guidance, Navigation and Control (GNC)  
  * Description: Packages for GNC of maritime robots.
  * Motive: There are some controls and tasks specifics to maritime robots, such as thruster control allocation, surveillance, vessels' handling of tides...
  * Repositories
    * **[Proposal]** Thruster manager
  * References: 
    *  [Plankton thruster manager](https://github.com/Liquid-ai/Plankton/blob/master/uuv_control/uuv_thruster_manager/src/uuv_thrusters/thruster_manager.py)
 
* Localization
  * Description: Packages to localize maritime vehicles.
  * Motive: Underwater and surface vehicles have a lot of differences regarding the sensors they can use for localizing themselves. GPS does not work underwater, as well as lidars or electromagnetic waves for long distances... But acoustic waves work pretty well, so underwater localization rely more on sensors like DVL and Sonar, that uses acoustic waves to measure distance and velocity.  
  * Repositories:
    * **[Proposal]**: Multilateration of acoustic beacons
  * References: 
    *  [Mesh Navigation](https://github.com/uos/mesh_navigation)
    *  [Fuse Localization](https://github.com/uos/mesh_navigation)
    *  [MagNav](https://www.gpsworld.com/us-air-force-to-explore-navigating-with-magnetism/)

* Perception
  * Description: Packages for perception algorithms for maritime robotics.
  * Motive: Since there are different sensors such as sonars, there is a need to process this data to obtain information. Moreover, it can be created some common structures detection.
  * Repositories:
    * **[Proposal]**: Common Maritime Environments Detections (buoys, valves, pipes...)
    * **[Proposal]**: Fusion of sonar and camera data
  * References: 
    * Suggestions...
   
* Robots
  * Description: Packages with maritime robots and structures.
  * Motive: All the software being developed shall have not only unit tests but integration tests, so robots and structures shall be developed for this purpose. These shall include vcstool for specifying packages to be used with the robot as well as ros_ign_bridge to test it in simulation.
  * Repositories
    * **[Proposal]** Hover type UW vehicle
    * **[Proposal]** Torpedo shaped UW vehicle
    * **[Proposal]** USV
    * **[Proposal]** Intervention UW vehicle 
  * References: 
    *  Intervention UW sample [RexROV2](https://uuvsimulator.github.io/packages/rexrov2/intro/)
    *  Torpedo Shaped UW vehicle [LRAUV](https://github.com/osrf/lrauv/)
   
* Simulation
  * Description: Documentation and index for maritime robotics Gazebo plugins.
  * Motive: There is no need to mix ROS code with simulation, also there is already some maritime robotics development in Ignition, so it is a good thing to contribute to ign-gazebo repo instead of creating other in this organization.
  * Repositories
    * **[Proposal]** Maritime Robotics Gazebo Plugins Index
    * **[Proposal]** Wave Simulation (repo shall be in ign-plugins)
    * **[Proposal]** Acoustics Simulation (repo shall be in ign-plugins) (Include salinity and temperature effects)
    * **[Proposal]** Radar Simulation (repo shall be in ign-plugins)
    * **[Proposal]** Sonar Simulation (repo shall be in ign-plugins)
  * References: 
    *  [ASV Wave Simulator](https://github.com/srmainwaring/asv_wave_sim/tree/feature/fft_waves)
    *  [Gazebo Sim](https://github.com/gazebosim/gz-sim)

* Tools
  * Description: Extra category for packages that do something specific that don't fit in any other category
  * Motive: There might be some tools that can be developed for maritime robotics that doesn't fit another category, such as geographic information convertion. 
  * Repositories
    * **[Proposal]** Geographic convertions
  * References: 
    *  Suggestions...

### Standards for subprojects

Subprojects must meet the following criteria (and the WG agrees to maintain them upon adoption).

* Build passes against ROS master
* The ROS standard linter set is enabled and adhered to
* If packages are part of nightly builds on the ROS build farm, there are no reported warnings or test failures
* Quality builds are green (address sanitizer, thread sanitizer, clang thread safety analysis)
* Test suite passes
* Code coverage is measured, and non-decreasing level is enforced in PRs
* Issues and pull requests receive prompt responses
* Releases go out regularly when bugfixes or new features are introduced
* The backlog is maintained, avoiding longstanding stale issues

### Adding new subprojects

To request introduction of a new subproject, add a list item to the "Subprojects" section and open a Pull Request to this repository, following the default Pull Request Template to populate the text of the PR.

PR will be merged on unanimous approval from Approvers.

### Subproject changes

Modify the relevant list item in the "Subprojects" section and open a Pull Request to this repository, following the default Pull Request Template to populate the text of the PR.

PR will be merged on unanimous approval from Approvers.

### Deprecating subprojects

Projects cease to be useful, or the WG can decide it is no longer in their interest to maintain.
We do not commit to maintaining every subproject in perpetuity.

To suggest removal of a subproject, remove the relevant list item in the "Subprojects" section and open a Pull Request in this repository, following instructions in the Pull Request Template to populate the text of the PR.

PR will be merged on unanimous approval from Approvers.

If the repositories of the subproject are under the WG's GitHub organization, they will be transferred out of the organization or deleted at this time.

## Governance

### Meetings

* Regular WG Meeting: First Tuesday of every month at 8AM PST
  * Meetings Announcement: One week before the meeting at ROS Discourse
  * Outputs of the meetings: Recording and minutes of the meeting at ROS Discourse

### Communication Channels

* Instant messaging: [Matrix community](https://matrix.to/#/#ros-maritime:matrix.org) (Matrix is an open network for secure, decentralized communication).
* Google Group: [Meeting invite group](https://groups.google.com/g/maritime-robotics-working-group-invites)
* Github organization: [ros-maritime](https://github.com/ros-maritime)
* Discourse tag: [wg-maritime-robotics](https://discourse.ros.org/tag/wg-maritime-robotics)

### Backlog Management

Each project shall discuss and define its goals in element.

### Membership, Roles and Organization Management

Working Group members may act in one or more of the following roles:

* **Member**
  * Prerequisite: Attend at least one out of the last three Working Group meetings
  * Responsible for triaging issues
* **Reviewer**
  * All reviewers are members
  * Prerequisite: Proven track record of high-quality reviews to WG Subprojects
  * Responsible for reviewing pull requests
* **Approver**
  * All approvers are reviewers
  * Prerequisite: Proven track record of high-quality contributions and reviews to WG Subprojects
  * Responsible for approving and merging pull requests
  * Responsible for vetting and accepting new projects into the Working Group
* **Lead**
  * Responsible for organizing and moderating working group meetings
  * Responsible for posting meeting materials (minutes, recordings, etc.)
  * Responsible for breaking ties

To become a member or change role, create an issue in this repository using the appropriate issue template.
Such applications are accepted upon unanimous agreement from Approvers, and are typically based on the applicant's history with the subprojects of the Working Group. The Lead role cannot be applied for.

### Modifying this governance document

Changes to this document will be made via Pull Request.
The PR will be merged on 2 agreement from Approvers.
