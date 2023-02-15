# lslidar_PointXYZ2PointXYZIR
This package convert lslidar's PointXYZ point clond to PointXYZIR type.

## Usage
1. Compile the package

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

## Attention
- The `ring` information was obtained by directly calculated by "arctan(z/distance)", `intensity` is not provided by lslidar.
- The `ring` is based on "1 degree mode" of lslidar.
