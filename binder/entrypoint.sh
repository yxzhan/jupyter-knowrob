#!/bin/bash
# Launch ros web applications
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch &

# Start MongoDB and save data on working directory
MONGODB_URL=mongodb://127.0.0.1:27017
mkdir -p ${PWD}/mongodb/data
mongod --fork --logpath ${PWD}/mongodb/mongod.log --dbpath ${PWD}/mongodb/data

# Launch Knowrob
source ${KNOWROB_WS}/devel/setup.bash
export KNOWROB_MONGODB_URI=${MONGODB_URL}/?appname=knowrob
roslaunch --wait knowrob knowrob.launch &

exec "$@"