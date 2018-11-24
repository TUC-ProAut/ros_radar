/** @mainpage ProAut Radar
 *
 * @section intro_sec Introduction
 *
 * This package was designed to read messages from the
 * <em>Bosch GPR V1.0 radar</em> using a <em>PEAK-USB</em> converter. This
 * package is not about handling low-level can communication. Therefore it
 * is assumed, that
 * <a href="https://wiki.ros.org/socketcan_bridge">socketcan_bridge_node</a>
 * is already running.
 *
 * The data from the sensor is published onto 'radar_messages' topic.
 * Additionally, the measurements for each single target are republished as
 * 'can_messages_A' and 'can_messages_B' respectively. For more details
 * see <a href="#radar_pa_node">Node radar_pa_node</a>.
 *
 * Moreover, it is possible to republish the data through a standard
 * pointcloud on 'radar_pcd'. Please refer to
 * <a href="#radar2pcd_pa_node">Node radar2pcd_pa_node</a>.
 *
 *
 * @section radar_pa_node Node radar_pa_node
 * @verbatim
rosrun radar_pa radar_pa_node@endverbatim
 *
 * <b>Input and Output Topics:</b>
 *
 * Topic Name          | Type                                                                              | Description
 * --------------------|-----------------------------------------------------------------------------------|-------------------------------------------------------------------------------------
 * "received_messages" | <a href="http://docs.ros.org/api/can_msgs/html/msg/Frame.html">can_msgs/Frame</a> | input from <a href="https://wiki.ros.org/socketcan_bridge">socketcan_bridge_node</a>
 * "radar_messages"    | radar_pa_msgs/radar_msg                                                           | output
 *
 *
 * @section radar2pcd_pa_node Node radar2pcd_pa_node
 * @verbatim
rosrun radar_pa radar2pcd_pa_node@endverbatim
 *
 * <b>Input and Output Topics:</b>
 *
 * Topic Name          | Type                                                                                              | Description
 * --------------------|---------------------------------------------------------------------------------------------------|-----------------------------------------------------------
 * "radar_messages"    | radar_pa_msgs/radar_msg                                                                           | input from <a href="#radar_pa_node">Node radar_pa_node</a>
 * "radar_pcd"         | <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html">sensor_msgs/PointCloud</a> | output
 *
 *
 * @section links_sec Links and packages
 *
 * Source code at github:
 *  + https://github.com/TUC-ProAut/ros_radar
 *
 * Related packages:
 *  + https://wiki.ros.org/socketcan_bridge
 *
 * ROS packages: (upcoming)
 *  + ros-indigo-radar-pa
 *  + ros-kinetic-radar-pa
 *  + ros-lunar-radar-pa
 *
 *
 * @section doc_sec ROS Documentation
 *
 * ROS-Distribution    | Documentation
 * --------------------|---------------
 * Indigo              | <a href="http://docs.ros.org/indigo/api/radar_pa/html/index.html">docs.ros.org</a>
 * Kinetic             | <a href="http://docs.ros.org/kinetic/api/radar_pa/html/index.html">docs.ros.org</a>
 * Lunar               | <a href="http://docs.ros.org/lunar/api/radar_pa/html/index.html">docs.ros.org</a>
 **/
