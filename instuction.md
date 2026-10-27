- mapping
```
docker exec -it dddmr_humble_dev bash

source /opt/ros/humble/setup.bash

source /root/dddmr_navigation/install/setup.bash

mkdir -p /root/dddmr_navigation/maps

ros2 launch lego_loam_bor x30_mid360_mapping.launch.py \
  mapping_dir:=/root/dddmr_navigation/maps/

ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty "{}"
```

- nav

docker exec -it dddmr_humble_dev bash
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/install/setup.bash


ros2 launch p2p_move_base x30_ig_loc_navigation.launch \
  map_pcd:=/root/dddmr_navigation/maps/x30/map.pcd \
  ground_pcd:=/root/dddmr_navigation/maps/x30/ground.pcd



- stop imu & lidar

bash /home/ysc/jy_cog/drivers/scripts/lidar_driver_stop.sh livox
bash /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh


