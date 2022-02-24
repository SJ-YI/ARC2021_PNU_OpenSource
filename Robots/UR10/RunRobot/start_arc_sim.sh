tmux kill-session
sleep 1
tmux new-session -d
tmux set -g mouse on
tmux split-window -h
tmux select-pane -L
tmux split-window -v
tmux select-pane -R
tmux select-pane -t 0
tmux send "source ~/.bashrc" C-m
tmux send "roscore" C-m

tmux select-pane -t 1
tmux send "source ~/.bashrc" C-m
tmux send "cd ~/Desktop/webots-ros-melodic-project" C-m
tmux send "./run_webots-ros_container.bash" C-m
tmux send "1234" C-m
tmux send "cd ~/" C-m



tmux select-pane -t 2
tmux send "source ~/.bashrc" C-m
tmux send "cd ~/Desktop/webots-ros-melodic-project" C-m
tmux send "./run_webots-ros_container.bash " C-m
tmux send "1234" C-m
tmux send "cd ~/" C-m
tmux attach-session -d
