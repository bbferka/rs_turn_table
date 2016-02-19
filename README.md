# rs_turn_table
RoboSherlock modules for simplifying recording from a turn table

1) Connect a Kinnect/Xtion etc. and roslaunch

2) rosrun scanning_table st_action.py

3) without any objects on the turning table estimate the plane of it with `rosrun robosherlock run estimate_plane`
  * in the visualizer check that the correct plane is found; if not modify the min max parameters of the filter in `estimate_plane.xml`, until it's correct

4) use `rosrun robosherlock run record` to record objects. 
  * set the name of the objects in the `SaveClusterCloudsAndImages.xml`
  * set angular resolution of the turn table in `MoveTurnTable.xml`
  * files are saved in the `rs_turn_table/data/[obj_name]` folder
