# Infromed Experience-driven Random Tree star (IERTCstar)
# Overview
IERTCstar is an experience-based optimal motion planning algorithm proposed by [1]. It is primarily based on the original ERTC algorithm [2] and has been modified by incorporating elements from RRTstar [3] and Informed RRTstar [4] to enhance the search for an optimal motion plan. The key advantage of IERTC* lies in its ability to efficiently find high-quality solutions for complex and challenging motion planning problems within a shorter computation time. For technical details, please refer to the original paper [1]. Note that the IERTCstar programs (IERTC.h and IERTC.cpp) were developed by modifying the ERTC programs [2]. Consequently, the license information is provided at the beginning of each program. 

# How to use
## Install Moveit1 and open motion planning library (OMPL)
1. Moevit1 source build
Follow the official instructions to build Moevit1 from source (https://moveit.ai/install/source/). When you add the planner later, you will edit the source code of some Moevit packages.

2. Install OMPL
Download the source code from the official website and install it (https://ompl.kavrakilab.org/installation.html).

## Add IERTCstar to OMPL
1. Add the files in the "programs" folder under the "/ompl-(version)/src/ompl/geometric/planners/experience". Specifically, add each cpp file under "/experience/src", and add each header file and ert folder under "/experience/".

2. Make the following changes to the planning_context_manager.cpp file under tha "/moveit/moveit_planners/ompl/ompl_interface/src/".

   2-1. Add "#include <ompl/geometric/planners/experience/IERTCstar.h>" to the header file.

   2-2. Add "registerPlannerAllocatorHelper<og::IERTCstar>(“geometric::IERTCstar”)" around line 300.

3. Modify the line "DESTINATION '${CMAKE_INSTALL_LIBDIR}'" in CMakeLists.txt located in /ompl-(version)/src/ompl/ to "DESTINATION '/opt/ros/noetic/lib/x86_64_linux_gnu'" (ensure that you check your PC environment for the exact file path). Then, perform a clean build with sudo privileges. Note that we used the sudo command to rebuild the library in the opt directory; however, there may be a more sophisticated approach. If you can successfully update limbompl.so.1.6.0 in the relevant folder by following these steps, the process has been completed correctly.

   As a point of caution, the following errors were observed when implementing this on several PCs.

   3-1. A build error occurred, but the update of limbompl.so.1.6.0 was successful. ⇨ In this case, the issue was resolved by restoring the CMAKE_INSTALL_LIBDIR setting and rebuilding.

   3-2. Unable to overwrite limbompl.so.1.6.0 ⇨ The issue was caused by the pre-installed OMPL library in MoveIt interfering with the update. The problem was resolved by purging the existing libompl and then rebuilding.

## Run motion planning
To execute the added planner in the Moevit, follow the steps below.

1. Edit the ompl_planning.yaml file located in the configuration folder of the robot. Specifically, add the following parameters to planner_configs and include the planner name (- IERTCstar) under the planner_configs section.
   
   IERTCstar:
   
       type: geometric::IERTCstar
   
       experience_fraction_min: 0.05
   
       experience_fraction_max: 0.10
   
       experience_tubular_radius: 10    

2. Place the three files in the sample_data directory in an appropriate location. Then, perform the following steps:

   3-1. Modify the file paths in filename_for_ompl.txt to match the new location. If the number of trajectory data points changes, update the corresponding values in point_num_for_ompl.txt.

   3-2. Edit the file paths at lines around 362 and 366 in IERTCstar.cpp.

3. Specify the planner name as “IERTCstar” and run the motion plan.

## Environment
Ubuntu 20.04 with ROS noetic

Moevit1

OMPL 1.6.0

## References
[1] Comming soon

[2] Pairet, È., Chamzas, C., Petillot, Y., & Kavraki, L. E. (2021). Path planning for manipulation using experience-driven random trees. IEEE Robotics and Automation Letters, 6(2), 3295-3302.

[3] Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. The international journal of robotics research, 30(7), 846-894.

[4] Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014, September). Informed RRT*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic. In 2014 IEEE/RSJ international conference on intelligent robots and systems (pp. 2997-3004). IEEE.

