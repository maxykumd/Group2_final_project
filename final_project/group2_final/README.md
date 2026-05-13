# Course: ENPM605 
### Section: 0101
### Professor: Zeid Kootbally
### Assignment: Final Group Project - Group 2
### Date: 05/12/2026


## Group Members:
Member 1: Nam Facchetti

UID: 118215693

Member 2: Yossaphat Kulvatunyou

UID: 112362550

## Contributions:
Nam: I implemented the shared ZoneManager class used throughout the mission behavior tree to manage patrol order, zone progression, survivor IDs, and base station state. I also implemented the primary behavior tree navigation and control nodes, including NavigateToZone, NavigateToBase, AdvanceZone, LogNoDetection, ZonesRemaining, and IsSurvivorDetected. Additionally, I handled debugging and refinement of the asynchronous Nav2 action client workflow, BT node lifecycle behavior (setup(), initialise(), update(), and terminate()), and ROS 2 logging integration for the navigation subsystem.

Yossaphat: I created the package skeleton and the CMake interface package. Write the DetectSurvivorServer, ReportSurvivorserver service node. Built the map with slam_toolbox. Handle the 2 custom service interface(detect + report survivor.srv) and 2 config files(mission + nav2_param). Implemented 3 BT action nodes (DetectSurvivor, BroadcastTF, NotifyBase). Write the search_and_rescue.launch and Assemble the full BT in entry point script and run testing with Gazebo to verify.

## BT Design:
The root node of the behavior tree is a reactive Selector (memory=False) named Mission or NavigateToBase. This design allows the tree to continuously re-evaluate mission completion on every tick. As long as there are remaining search zones, the patrol branch executes. Once all zones have been visited and ZonesRemaining returns FAILURE, the selector falls through to the return-to-base branch. The patrol branch is implemented as a Sequence with memory=True. This memory setting is important because navigation and service calls are asynchronous operations that may require multiple ticks to complete. With memory=True, the patrol sequence resumes execution from the currently running child instead of restarting from the beginning each tick, preventing repeated navigation goals and unnecessary re-evaluation of completed steps. The HandleDetection composite is implemented as a reactive Selector (memory=False). Its first child checks whether a survivor was detected, and if not, the selector immediately falls back to the LogNoDetection action. Using memory=False ensures that the survivor detection condition is re-evaluated each tick instead of caching previous outcomes. The SurvivorFound branch is implemented as a Sequence with memory=True. This is necessary because the NotifyBase node performs an asynchronous service call that may remain in the RUNNING state across multiple ticks. Using memory=True prevents earlier siblings such as BroadcastSurvivorTF from being re-executed repeatedly, which would otherwise create duplicate survivor TF frames and allocate multiple survivor IDs for the same detection event. Finally, the NavigateToBase node is wrapped in a py_trees.decorators.OneShot decorator using the ON_COMPLETION policy. Since the root selector is reactive, this prevents the behavior tree from continuously re-sending navigation goals to the base station after the robot has already returned home and completed the mission.
