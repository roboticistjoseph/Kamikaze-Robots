#!/bin/bash
#
# This script should be invoked by "ros2 run" and after the unit test
# has been run.
#

set -ue                         # stop at the first error

PROG_DIR=$(dirname $(readlink -f "$0")) # where is the program located
EXEC_DIR=$(pwd -P)                      # where are we executing from
PROG_NAME=$(basename "$0")              # the program name without the path

# 1.) Get the ros package name
ROS_PACKAGE_NAME=$(basename $PROG_DIR)

# 2.) Generat report info
BUILD_DIR=$EXEC_DIR/build/$ROS_PACKAGE_NAME/
rm -f $PROG_DIR/coverage.info
lcov --capture --directory $BUILD_DIR --output-file $PROG_DIR/coverage.info

# 3.) Exclude some files from the reoport
rm -f $PROG_DIR/coverage_cleaned.info
lcov --remove $PROG_DIR/coverage.info \
     '/opt/*' \
     '/usr/*' \
     '*rclcpp/*' \
     '*libstatistics_collector/*' \
     '*rosidl_runtime*' \
     '*rcl_interfaces*' \
     '*rmw/rmw/*' \
     '*tracetools/*' \
     '*_msgs/*' \
     '*/gtest*' \
     'gtest/*' \
     --output-file $PROG_DIR/coverage_cleaned.info


# 4.) Finally generate the coverage report
rm -rf $EXEC_DIR/install/$ROS_PACKAGE_NAME/coverage/
genhtml $PROG_DIR/coverage_cleaned.info --output-directory \
        $EXEC_DIR/install/$ROS_PACKAGE_NAME/coverage
echo "Code Coverage generated:"
echo "     $PROG_DIR/coverage_cleaned.info"
echo "     $EXEC_DIR/install/$ROS_PACKAGE_NAME/coverage/index.html"

