# rs_turn_table
RoboSherlock modules for simplifying recording from a turn table

1) Connect a Kinnect/Xtion etc. and roslaunch

2) rosrun iai_scanning_table st_action.py

3) without any objects on the turning table estimate the plane of it with `roslaunch robosherlock rs.launch ae:=estimate_plane`
  * in the visualizer check that the correct plane is found; if not modify the min max parameters of the filter in `estimate_plane.xml`, until it's correct

4) use `roslaunch robosherlock rs.launch ae:=record` to record objects. 
  * set the name of the objects in the `SaveClusterCloudsAndImages.xml`
  * set angular resolution of the turn table in `MoveTurnTable.xml`
  * files are saved in the `rs_turn_table/data/[obj_name]` folder

## Necessary packages
The iai_scanning_table package can currently be found here: https://github.com/code-iai/iai_robot_drivers.git . Please take care that iai_robot_drivers might come with its own dependencies.
