


. ../esp-idf/export.sh
echo "" > out
for NUM_PUB in 15; do 

    sed -i "33s/.*/\"-DRMW_UXRCE_MAX_SUBSCRIPTIONS=$NUM_PUB\",/" components/micro-ROS/colcon.meta 
    rm -rf components/micro-ROS/include/ components/micro-ROS/*.a components/micro-ROS/micro_ros_src/build components/micro-ROS/micro_ros_src/install
    idf.py build
    TEXT_USAGE=$(size -t build/micro_ros_idf.elf | grep TOTALS | awk -F ' ' '{print $1}')
    DATA_USAGE=$(size -t build/micro_ros_idf.elf | grep TOTALS | awk -F ' ' '{print $2}')
    BSS_USAGE=$(size -t build/micro_ros_idf.elf | grep TOTALS | awk -F ' ' '{print $3}')
    echo "$NUM_PUB,$TEXT_USAGE,$DATA_USAGE,$BSS_USAGE" >> out

done