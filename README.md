# Infromed Experience-driven Random Tree star
# 
TBD：プログラムの使い方を記載ください

## Moveit1+OMPLライブラリのインストール
1. Moevit1のソースビルド
Moevit公式の案内に従い、Moevit1をソースビルドする(https://moveit.ai/install/source/). 後にプランナを追加する際に、いくつかのMoevitパッケージのソースコードを編集することになる.

2. OMPLライブラリのインストール
OMPL公式HPから、omplライブラリのソースコードをダウンロードし、インストールを行う(https://ompl.kavrakilab.org/installation.html). 

## OMPLライブラリへのプランナーの追加
1. /ompl-1.6.0/src/ompl/geometric/planners/experience下に、experience内のファイルを追加する. 具体的には、/experience/src下に各cppファイルを追し、/experience/下に各ヘッダファイルとertのフォルダを追加する.

   ERTConnect: 過去の動作計画結果を再利用して、両側探索を行う動作計画アルゴリズム[1][2]. 解を発見した時点でそれをリターンして探索を終了する. 最適性を考えないで解を高速に探索したい時はこちらを使う. 

   ERTCstar: 過去の動作計画結果を再利用して、最適(漸近最適)探索を行う動作計画アルゴリズム[3]. 事前に指定された時間内で解を更新し続け、指定時間経過後にそれをリターンする. 

2. /moveit/moveit_planners/ompl/ompl_interface/src/planning_context_manager.cppのファイルに関して、以下の変更を加える

   2-1. ヘッダファイルに#include <ompl/geometric/planners/experience/ERTConnect.h>及び#include <ompl/geometric/planners/experience/ERTCstar.h>を追加

   2-2. 300行目辺りにregisterPlannerAllocatorHelper<og::ERTConnect>("geometric::ERTConnect")及びregisterPlannerAllocatorHelper<og::ERTCstar>("geometric::ERTCstar")を追加

3. /ompl-1.6.0/src/ompl/下のCMakeLists.txt, 86行目あたりの「DESTINATION "${CMAKE_INSTALL_LIBDIR}"」を「DESTINATION "/opt/ros/noetic/lib/x86_64_linux_gnu" (ここのファイルパスは各自のPC環境を確認)」 に変更し、sudo権限でクリーンビルドを行う (optフォルダ内のライブラリを書き換えるためにsudoビルドを行ったが、しなくて良いやり方もある？). これらの操作を行なって、該当フォルダ内の"limbompl.so.1.6.0"が更新できていればok.また、その後は"CMAKE_INSTALL_LIBDIR"を元に戻してもok.

   注意点として、これまでにいくつかのPCで実装を行なった結果、以下のようなエラーと対処をする必要が出た.

   3-1. ビルドエラーが出るが、"limbompl.so.1.6.0"の更新には成功している ⇨ この場合は、"CMAKE_INSTALL_LIBDIR"を元に戻して、再度ビルドすることで解決した

   3-2. "limbompl.so.1.6.0"の書き換えができない ⇨ Moveitに元から入っているomplライブラリが邪魔をしていたため、既存のlibomplをpurgeしてからビルドを行ったところ、解決した

## 動作計画の実行方法
追加したプランナをMoevitの動作計画で実行するには、以下の手順を踏む.

1. 対象ロボットのconfigフォルダ内、ompl_planning.yamlを編集する. 具体的には、planner_configsに以下のパラメータを追加し、対象armのplanner_configs下に名前(- ERTConnect, - ERTCstar )を追加
   
   ERTConnect:
   
       type: geometric::ERTConnect
   
       experience_fraction_min: 0.05
   
       experience_fraction_max: 0.10
   
       experience_tubular_radius: 10
   
   ERTCstar:
   
       type: geometric::ERTCstar
   
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
[1] Pairet, È., Chamzas, C., Petillot, Y., & Kavraki, L. E. (2021). Path planning for manipulation using experience-driven random trees. IEEE Robotics and Automation Letters, 6(2), 3295-3302.

[2] Takamido, R., & Ota, J. (2023). Learning robot motion in a cluttered environment using unreliable human skeleton data collected by a single RGB camera. IEEE Robotics and Automation Letters.

[3] 高御堂良太, 太田順. (2024). 不確実な人教示データを活用した柔軟かつ高速な最適動作計画手法の提案, 日本ロボット学会学術講演会第42回学術講演会予稿集, RSJ2024AC3B2-01 1-3

