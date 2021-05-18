# parachute
A standalone version of parachute plugin developed at https://github.com/PX4/PX4-SITL_gazebo/pull/389

Tested on ROS Melodic

Example of how to include a plugin in a model - box
```xml
<plugin name='parachute_plugin' filename='libtest_parachute.so'>
		<robotNamespace/>
		<!-- message type std_msgs Bool : publish True or 1 to trigger-->
		<parachuteTriggerTopic>box</parachuteTriggerTopic>
</plugin>
```

After launching the model in gazebo, use the following publish message to trigger the parachute
```sh
rostopic pub -1 /parachute_plugin/box std_msgs/Bool -- 1
```

dependencies : 
As of now it requires the liftdrag plugin ( to be installed by building the px4 package)
