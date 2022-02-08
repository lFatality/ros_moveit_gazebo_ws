# ROS1 Moveit Gazebo URDF Example

## How to simulate an arm using Gazebo and MoveIt!

Repository with an example how to use Gazebo and MoveIt! to control a manipulator described in a URDF file.  
Based on this tutorial: https://www.youtube.com/watch?v=O7nBa7mnfW4

This Readme describes the different steps and principles that go into simulating a Robotic Arm defined in a URDF in Gazebo
and also moving it around using MoveIt!.

### How to start
```
roslaunch arm_moveit_config gazebo.launch
roslaunch arm_moveit_config move_group.launch
roslaunch arm_moveit_config moveit_rviz.launch
```

In Rviz choose the fixed frame `world`.  
Click on `Add` and choose `moveit_ros_visualization/MotionPlanning`.

### TLDR
1. Have a URDF (no transmissions / gazebo plugin required)
2. Create a package for the robot description (`catkin_create_pkg arm_description std_msgs rospy roscpp`)
3. Open the MoveIt! Setup Helper (`roslaunch moveit_setup_assistant setup_assistant.launch`)
4. Set up your MoveIt! package in the setup assistant (find details below)
5. In the `ros_controllers.launch` add the `joint_state_controller` to the other controllers
6. In the same file add the `robot_state_publisher` (`<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />`)
7. If you are using a xacro file: In your gazebo.launch, change the parameter loading of the `robot_description` to `<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find your_package_name)/urdf/your_urdf_file.urdf.xacro'" />`

## The long version

In this part of the readme.md the procedure is explained in more detail.

### Requirements:
A URDF of a manipulator

Make sure all the necessary information is contained in your URDF file.  
Note that you don't need any transmissions or Gazebo plugins yet, those will be created for you by the MoveIt! setup assistant.  
You can find templates for links and joints below:

Link:
```
    <link name="link1">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.05" />
            <geometry>
                <cylinder length="0.10" radius="0.10" />
            </geometry>
            <material name="silver">
                <color rgba="0.75 0.75 0.75 1" />
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.05" />
            <geometry>
                <cylinder length="0.10" radius="0.10" />
            </geometry>
        </collision>
        
        <inertial>
            <mass value="0.1" />
            <inertia ixx="0.03" iyy="0.03" izz="0.03" ixy="0.0" ixz="0.0" iyz="0.0" />
        </inertial>

    </link>
```

Joint:
```
    <joint name="joint1" type="continuous">
        <origin rpy="0 0 0" xyz="0 0 0.01" />
        <parent link="base_link" />
        <child link="link1" />
        <axis xyz="0 0 1" />
    </joint>
```

If you want to give the links a color in gazebo:
```
    <gazebo reference="link1">
        <material>Gazebo/Black</material>
        <turnGravityOff>false</turnGravityOff>
    </gazebo>
```

An invisible `base_link`:
```
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.01" />
            <material name="silver">
                <color rgba="0.75 0.75 0.75 1" />
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.01" />
            <geometry>
                <cylinder length="0.02" radius="0.5" />
            </geometry>
        </collision>
        
        <inertial>
            <mass value="0.1" />
            <inertia ixx="0.03" iyy="0.03" izz="0.03" ixy="0.0" ixz="0.0" iyz="0.0" />
        </inertial>

    </link>
```

You will then later get this error as there is no geometry defined for the `base_link`.  
You can ignore it.  
`[ERROR] [1605470815.062129407]: Could not parse visual element for Link [base_link]`

### Create a package with the robot description

In a workspace create a new package.  
Go into the `src` folder an run `catkin_create_pkg arm_description std_msgs rospy roscpp`.  
Create a folder `urdf` in the package and paste your urdf file there.

### MoveIt! Setup Assistant

Source your workspace (`source devel/setup.bash`)!

Then run the following command:  
`roslaunch moveit_setup_assistant setup_assistant.launch`

In the different section of the setup assistant do the following:

#### Start
Load your URDF file.

#### Self-collisions
Click `Generate Collision Matrix`

#### Virtual Joints:
If you want you can add an attachment point for the arm.  
This could be e.g. to attach the robot to the ground or a mobile base in gazebo.  

#### Planning groups
Group of the arm:  

- Click `Add group`
- Fill in the group name
- Choose Kinematic solver (most use the `kdl_kinematics_plugin/KDLKinematicsPlugin`)
- Click `Add joints`, select joint1 to joint6 (do not choose joints that attach the robot to the ground or end-effector joints)
- Click on save, then click on `Links` in the generated group
- Select everything from the `base_link` to the last link of the arm (but not from the end-effector)
- Click save

Group of the gripper:

- Click `Add group`
- Fill in the group name
- Choose Kinematic solver
- Click `Add joints` and select the joints of the end-effector
- Click on save, then click on `Links` in the generated group
- Select the links of the end-effector
- Click save

#### Robot Poses
Click `Add Pose`.  
Check that the robot joints are moving as you expect.  
Generate at least one pose, otherwise errors might occur later.

#### End effectors
Click `Add end-effector`.  
Give it a name, select the group name you've chosen for the end-effector.  
The parent link is the last link of the arm, the one the end-effector is attached to.

Passive joints: Can be ignored if your robot has no not-actuated joints.

#### ROS Control
Auto-generate the controllers with the button,  
then edit them and change them to a `JointTrajectoryController`.  
There are multiple ones, choose the one that makes the most sense.  
For the arm itself, a `position_controller` makes sense, for a gripper  
maybe you want to use an `effort_controller` instead.  
Once you did that, click the auto-generate button again and you will see  
additional `FollowJointTrajectory` controllers, one for the gripper and one for the arm.

#### Simulation
Click `Generate URDF` and copy the contents into your clipboard.  
Go to your URDF and overwrite it with the contents from your clipboard.

#### Author
Insert your name and email.

#### Configuration files
Choose a path to an empty folder in your workspace in which the package shall be created.

Click `Generate package`. It will give a warning that there are no virtual joints, click OK.

### After you generated the MoveIt! package

Go into the moveit package, into `launch` and open `ros_controllers.launch`  
In the controller spawner add the `joint_state_controller` to the `arm_controller gripper_controller`.

Also add a joint state publisher  
`<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />`

Now you can start the simulation with `roslaunch arm_moveit_config gazebo.launch`.

Problem 1:
```
[ERROR] [1605470815.463187, 8014.855000]: Spawn service failed. Exiting.
[gazebo-1] process has died [pid 28656, exit code 255, cmd /opt/ros/noetic/lib/gazebo_ros/gzserver -e ode worlds/empty.world __name:=gazebo __log:=/home/fynn/.ros/log/323d56e2-276b-11eb-a0d9-71fb9277f977/gazebo-1.log].
log file: /home/fynn/.ros/log/323d56e2-276b-11eb-a0d9-71fb9277f977/gazebo-1*.log
[ INFO] [1605470815.559039436]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1605470815.560213209]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[spawn_gazebo_model-3] process has died [pid 28666, exit code 1, cmd /opt/ros/noetic/lib/gazebo_ros/spawn_model -urdf -param robot_description -model robot -x 0 -y 0 -z 0 __name:=spawn_gazebo_model __log:=/home/fynn/.ros/log/323d56e2-276b-11eb-a0d9-71fb9277f977/spawn_gazebo_model-3.log].
log file: /home/fynn/.ros/log/323d56e2-276b-11eb-a0d9-71fb9277f977/spawn_gazebo_model-3*.log
[ERROR] [1605470816.340581, 8015.725000]: Failed to load arm_controller
[INFO] [1605470816.341727, 8015.725000]: Loading controller: gripper_controller
[ERROR] [1605470817.345272, 8016.721000]: Failed to load gripper_controller
```

Solution 1:  
There was still another gazebo / roscore running. Make sure to close everything.  
The second gazebo caused the controllers to be loaded twice and the second time failed.  
Also open the task manager and search for `gazebo`. If for example the `gzserver` is still running, kill it.

Problem 2:
```
[ERROR] [1605476003.698174399, 0.368000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/joint1
[ERROR] [1605476003.698872624, 0.368000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/joint2
[ERROR] [1605476003.699471603, 0.368000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/joint3
[ERROR] [1605476003.700111534, 0.368000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/joint4
[ERROR] [1605476003.700783622, 0.368000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/joint5
[ERROR] [1605476003.701341736, 0.368000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/joint6
```

Interestingly this does not happen for the finger joints.

Solution 2:  
As a workaround, you can add this to your `ros_controllers.yaml` file.  
It's not really a nice solution, maybe there is a better way?
```
gazebo_ros_control:   
  pid_gains:
    joint1:
      p: 5.0
      i: 1.0
      d: 1.0
    joint2:
      p: 5.0
      i: 1.0
      d: 1.0
    joint3:
      p: 5.0
      i: 1.0
      d: 1.0
    joint4:
      p: 5.0
      i: 1.0 
      d: 1.0
    joint5:
      p: 5.0
      i: 1.0 
      d: 1.0
    joint6:
      p: 5.0
      i: 1.0 
      d: 1.0
```

Problem 3:
```
[ERROR] [1605476414.925647371, 0.550000000]: Could not load controller 'arm_controller' because controller type 'position_controllers/JointTrajectoryController' does not exist.
```

Solution 3:
`sudo apt-get install ros-noetic-joint-trajectory-controller` (<- you might need to change to the testing repo to get access to this)

### Start procedure
```
roslaunch arm_moveit_config gazebo.launch
roslaunch arm_moveit_config move_group.launch
roslaunch arm_moveit_config moveit_rviz.launch
```

In Rviz choose the fixed frame `world`.  
Click on `Add` and choose `moveit_ros_visualization/MotionPlanning`.

You can now go into `Planning` and choose one of the poses you've specified earlier.  
Then click on `Plan` and then `Execute`. You will see how the manipulator moves both  
in Rviz and in Gazebo.

Problem 4:
After running `gazebo.launch` there is no model even though there is no error message.

Solution 4:
Check the way you are loading the `robot_description` parameter.
If you are using a xacro file you have to adjust it.
Instead of 
```
<arg name="urdf_path" default="$(find your_package_name)/urdf/your_urdf_file.urdf.xacro"/>
<param name="robot_description" textfile="$(arg urdf_path)" />
```
you need
```
<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find your_package_name)/urdf/your_urdf_file.urdf.xacro'" />
```

Problem 5:
When running `move_group.launch` you get:
`[ WARN] [1605612789.197556838]: No kinematics plugins defined. Fill and load kinematics.yaml!`

Solution 5:
You probably forget to select a kinematics plugin in the setup assistant.
In your `kinematics.yaml` you can fill it in still. The structure looks like this:
```
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
gripper:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

Problem 6:  
The marker with which you can normally move the robot end effector does not appear.

Solution 6:  
The marker should be there, it's probably just too small to see.  
Go into `Planning Request -> Interactive Marker Size` and increase it.

Problem 7:  
My robot is very unstable and jumping around or it breaks and all the parts snap into each other or it disappears completely.

Solution 7:  
You might have too high PID values in your `controllers.yaml`. Try setting them to a low value and see if that helps.  
Remember that too low values will make the robot unable to move though. So you have to find a good balance.

Another reason can be that the inertia values of your parts are incorrect. You can approximate them using these matrices here: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors  
Then you fill them into your URDF in the <inertia> tag. Because inertia tables are symmetric, there will be only 6 values instead of 9.  
Also be sure to adjust the <mass> in your URDF.
