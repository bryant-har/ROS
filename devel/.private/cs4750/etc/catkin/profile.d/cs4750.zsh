mviz() {
    rviz -d $(rospack find cs4750)/config/default.rviz
}

export CORNELL_TOOLS_SHELL_FLAVOR=zsh
. /home/bjh254/homework_ws/src/cs4750_student/cs4750/config/env.sh
export SOURCES_ROOT="/home/bjh254/homework_ws/src/cs4750_student/cs4750/src/cornell_tools"

export PATH="/home/bjh254/homework_ws/src/cs4750_student/cs4750/scripts/available_client/:$PATH"
. "/home/bjh254/homework_ws/src/cs4750_student/cs4750/scripts/cornell_tools_client"

