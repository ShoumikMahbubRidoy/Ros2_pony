#!/bin/bash

echo "Killing ROS2 nodes..."

ps aux | grep ros | grep -v grep | awk '{print "kill -9", $2}' | sh

ros2 node list

exit 0