#!/bin/zsh

echo "Starting gz!"
{
    gnome-terminal --tab --title="gz" -- zsh -c "gz sim -v4 -r piping.sdf; exec zsh"
} &

sleep 5

echo "Starting px4_sitl"
{
    gnome-terminal --tab --title="px4_sitl" -- zsh -c "cd ~/Projects/PX4-Autopilot;make px4_sitl gz_x500_mono_cam; exec zsh"
} &

sleep 1

echo "Starting launch"
{
    gnome-terminal --tab --title="launch" -- zsh -c "cd ~/Projects/colcon_ws;source install/setup.sh;ros2 launch px4_offboard simulation.launch.py; exec zsh"

} &
sleep 2

echo "Starting mavros"
{
    gnome-terminal --tab --title="mavros launch" -- zsh -c "ros2 launch px4_offboard mavros_px4.launch; exec zsh"

} &
echo "Done."
