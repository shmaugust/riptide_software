#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
rsync -tvrz ~/osu-uwrt/riptide_software/src ros@jetson:~/osu-uwrt/riptide_software
ssh ros@jetson 'cd ~/osu-uwrt/riptide_software && source /opt/ros/kinetic/setup.bash && catkin_make && source ~/osu-uwrt/riptide_software/devel/setup.bash'
ssh ros@jetson 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*'
