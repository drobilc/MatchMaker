# TUTORIAL FOR HOMEWORK 1

1. Create an empty package and build it
    1. Move to your catkin workspace in cmd:
    `cd ~/ROS_workspace/src`
    2. Create a package *hw1* with dependencies to *std_msg*, *rospy* and *roscpp*:
    `catkin_create_pkg hw1 std_msgs rospy roscpp` This should create a folder hw1 with 2 subfolders *include* and *src* and 2 files *CMakeLists.txt* and *package.xml*. The later contains the dependencies mentioned before.
    3. Build a package with `catkin_make`. Note this command should be run in the root folder of your catkin workspace, that is *ROS_workspace* in our example.
    4. Add workspace to ROS environment by sourcing the generated setup file. Run `.~/ROS_workspace/devel/setup.bash`
2. 