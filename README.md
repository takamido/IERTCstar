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

## 動作計画の実行方法
追加したプランナをMoevitの動作計画で実行するには、以下の手順を踏む.

1. 対象ロボットのconfigフォルダ内、ompl_planning.yamlを編集する. 具体的には、planner_configsに以下のパラメータを追加し、対象armのplanner_configs下に名前(- ERTConnect, - ERTCstar )を追加
   
   IERTCstar:
   
       type: geometric::IERTCstar
   
       experience_fraction_min: 0.05
   
       experience_fraction_max: 0.10
   
       experience_tubular_radius: 10    

3. sample_data内の3つのファイルを適当な場所に配置する. その後、以下の操作を行う.

   3-1. filename_for_ompl.txtファイルの中に記載のファイルパスを配置した場所に合わせて書き換える (軌道データの点数が変わる場合は、point_num_for_ompl.txtの中の数値も変える).

   3-2. ERTConnect.cppの328行目と321行目のファイルパスを編集して、配置した場所に合わせて書き換える。同様に、ERTCstar.cppの359行目と362行目も変更する。

4. プランナ名を"ERTConnect"または"ERTCstar"に指定して、動作計画を実行する

## 動作環境
Ubuntu 20.04 with ROS noetic

Moevit1

OMPL 1.6.0

## 参考文献
[1] Comming soon

[2] Pairet, È., Chamzas, C., Petillot, Y., & Kavraki, L. E. (2021). Path planning for manipulation using experience-driven random trees. IEEE Robotics and Automation Letters, 6(2), 3295-3302.

[3] RRTstar

[4] Informed RRTstar

