# gazebo_useful_plugins

These plugins allow you to play with the physics concepts of Gazebo and its dynamic simulator.

1) gazebo_tf_injector.cpp: using joint state publisher gui it's possible to set the world pose of the joints. Gazebo will auto-generate the force for the  joints: 
useful link: https://github.com/acschaefer/gazebo_tf_injector
   
2) centauro_plugin.cpp: using XBotCore, it's possible to read the joint state and using the impedance control to generate the righ force.


## How to run

1) Adding the plugin libcentauro_plugin.so or libgazebo_tf_injector.so "Gazebo Plugin area" of the specific robot:

i.e: MultiDoF-superbuild/robots/centauro-simulator/centauro_gazebo/urdf/centauro.gazebo
    
     ......
    <xacro:if value="${middleware == 'ros_control'}">
	  <gazebo>
        <!-- XBOTCORE -->
            <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
                <!--<robotNamespace>/centauro</robotNamespace> -->
                <!-- <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType> -->
            </plugin>

	    <plugin name="centauro_plugin" filename="/home/user/catkin_ws_gazebo/devel/lib/libcentauro_plugin.so"> </plugin> 
      <plugin name="gazebo_tf_injector" filename="/home/user/catkin_ws_gazebo/devel/lib/libgazebo_tf_injector.so"> </plugin> 
      ......
2) roslaunch gazebo_centauro.launch present in this project.
3) Check the result on gazebo.
