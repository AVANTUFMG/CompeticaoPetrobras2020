#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# change this to your liking
PROJECT_NAME=fase1_sprogram

# do not change this
MAIN_DIR=~/"bag_files"

# following commands will be executed first, in each window
pre_input="export UAV_NAME=uav1; export RUN_TYPE=simulation; export ODOMETRY_TYPE=hector"

# define commands
# 'name' 'command'
# DO NOT PUT spaces in the names

# 'Gazebo' "waitForRos; roslaunch competition_petrobras UFMG.launch gui:=true world_name:=fase1
# 'Gazebo' "waitForRos; roslaunch competition_petrobras UFMG.launch gui:=true world_name:=fase2
# 'Gazebo' "waitForRos; roslaunch competition_petrobras UFMG.launch gui:=true world_name:=fase3
# 'Gazebo' "waitForRos; roslaunch competition_petrobras UFMG.launch gui:=true world_name:=fase4
input=(
  'Roscore' "roscore
" 
  'camera' "waitForControl; rosrun rviz rviz -d `rospack find ufmg_desafio_petrobras`/camera.rviz
"
  'Gazebo' "waitForRos; roslaunch ufmg_desafio_petrobras simulation_arena.launch gui:=true world_name:=fase3
"
  'Spawn' "waitForSimulation; spawn_uav 1 --file uav1_pos.yaml --f450 --run --delete --enable-rangefinder --enable-rangefinder-up --enable-rplidar --enable-bluefox-camera --enable-realsense-front
"
  'Control' "waitForOdometry; roslaunch mrs_uav_general core.launch config_uav_manager:=./custom_configs/uav_manager.yaml
"
  'AutomaticStart' "waitForSimulation; roslaunch mrs_uav_general automatic_start.launch;
"
  'Hector Launch' "waitForSimulation; roslaunch mrs_uav_general hector_slam.launch
"
  "PrepareUAV" "waitForControl; rosservice call /uav1/control_manager/use_safety_area \"data: false\"; rosservice call /$UAV_NAME/mavros/cmd/arming 1; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard; rosservice call /uav1/control_manager/set_min_height \"value: 0.0\"
"
  'Camera_follow' "waitForOdometry; gz camera -c gzclient_camera -f $UAV_NAME"
  'gazebo_camera_follow' "waitForOdometry; gz camera -c gzclient_camera -f $UAV_NAME; history -s gz camera -c gzclient_camera -f $UAV_NAME"
  'waypoint' "waitForSimulation; cd ~/catkin_ws/src/Arena_Fase_1_waypoints/ros_uav_waypoints/launch; roslaunch waypoint_flier_arena.launch; "

)


init_window="Control"

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=mav

# prefere the user-compiled tmux
if [ -f /usr/local/bin/tmux ]; then
  export TMUX_BIN=/usr/local/bin/tmux
else
  export TMUX_BIN=/usr/bin/tmux
fi

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then

  echo "The session already exists"
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  touch "$ITERATOR_FILE"
  ITERATOR="0"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest"
rm "$MAIN_DIR/latest"
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;
${pre_input};
${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

$TMUX_BIN -2 attach-session -t $SESSION_NAME

clear
