# lslidar_PointXYZ2PointXYZIR
This package convert lslidar's PointXYZ point clond to PointXYZIR type.  
Lidar type: Leishen C32 0.33-degree. (for 1-degree lidar, it's much simpler to modify the codes)  

## Usage
1. Compile the package.  
If you want to see each ring in rviz, undefine the `PUB_EACH_RING` in `main.cpp`.  
Otherwise, each ring will be published and viewed in rviz.

2. Remap the topics in `lslidar_conversion.launch` file.  
The `/pc_input` should mapped to the topic of PointCloud from lslidar's driver node, and `/pc_output` is any name you want.\

3. Run launch file:
```bash
# source the package if needed.
source devel/setup.bash

# launch the node
roslaunch lslidar_conversion lslidar_conversion.launch

# run rviz for visualization (optional)
roslaunch lslidar_conversion rviz.launch
```
The output `/pc_input` is `PointXYZIR` type.  
You can view each ring in rviz by select and modify the topic names (`ring_0` to `ring_31`) in `ring` item. 

## Attention
- The `ring` information was obtained by directly calculated by "arctan(z/distance)", `intensity` is not provided by lslidar.
- The `ring` is based on "0.33 degree mode" of lslidar. 
- You cannot just change the "0.33" mode to "1" mode to get 1-degree mode data. Because the lidar hardware doesn't support different configurations.

## CSDN blog (In Chinese)
[CSDN的总结与说明](https://blog.csdn.net/tfb760/article/details/129108936)
