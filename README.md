# NAVQ ROS 2 OLED example using UXRCE DDS AGENT to PX4
## OLED examples using the NXP NavQ plus and ssd1306 oled via ROS 2

You will need to clone px4_msgs(https://github.com/PX4/px4_msgs) and colcon build with this package in the same overlay as px4_msgs

To run the node and init the OLED, pass in the paramters consisting i2c OLED device info (port and address):

`ros2 run px4_oled px4_status --ros-args -p i2c_port:=1 -p i2c_address:="0x3c" -p font_path:="/example/font/path.ttf" -p font_size:=18`

Pass in the font (TTF or OTF) absolute path to the file, as well as the font size. The default vehicle_status message has a lot of info, but I chose to use arming status as the ssd1306 is tiny and I would prefer minimal text display that is easily readable over tons of tiny messages and symbols.

Have fun. Enjoy. :)
