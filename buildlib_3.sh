


. ../esp-idf/export.sh
echo "" > out
for HISTORIC in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do 

    sed -i "34s/.*/\"-DRMW_UXRCE_MAX_HISTORY=$HISTORIC\",/" components/micro-ROS/colcon.meta 
    rm -rf components/micro-ROS/include/ components/micro-ROS/*.a components/micro-ROS/micro_ros_src/build components/micro-ROS/micro_ros_src/install
    idf.py build
    TEXT_USAGE=$(size -t build/micro_ros_idf.elf | grep TOTALS | awk -F ' ' '{print $1}')
    DATA_USAGE=$(size -t build/micro_ros_idf.elf | grep TOTALS | awk -F ' ' '{print $2}')
    BSS_USAGE=$(size -t build/micro_ros_idf.elf | grep TOTALS | awk -F ' ' '{print $3}')
    echo "$HISTORIC,$TEXT_USAGE,$DATA_USAGE,$BSS_USAGE" >> out

done