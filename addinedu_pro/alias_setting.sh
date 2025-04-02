# [중요] Ubuntu 24.04 ROS2 Jazzy 환경 상에서 진행

# ALIAS SETTINGS
alias tmp1="eval \"\$(register-python-argcomplete ros2)\""
alias tmp2="tmp1; eval \"\$(register-python-argcomplete colcon)\""

# 3. ROS 환경 변수 설정
ID=99
# export ROS_LOCALHOST_ONLY=1 # ROS 환경에서 로컬 네트워크 내에서만 메시지를 사용하도록 설정

# 4. 자주 사용하는 명령어 단축 (alias 설정)
alias sb="source ~/.bashrc; echo \"Bashrc is reloaded!\"" # source bashrc 줄여서 sb
alias ros_domain="export ROS_DOMAIN_ID=\$ID; echo \"ROS_DOMAIN_ID is set to \$ID!\"" # 변수 지정은 $로
alias jazzy="source /opt/ros/jazzy/setup.bash; ros_domain; echo \"ROS2 Jazzy is activated!\""

# 5. Bash 함수 선언
# (1) 워크스페이스 설정 함수
ws_setting()
{
	jazzy
	source ~/$1/install/local_setup.bash  # 입력 인자는 ${num}으로 받는다
	echo "$1 is activated!"
	tmp2
}

# (2) ROS 환경 상태 확인 함수
get_status()
{
	if [ -z $ROS_DOMAIN_ID ]; then
		echo "ROS_DOMAIN_ID : 0"
	else
		echo "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
	fi

	if [ -z $ROS_LOCALHOST_ONLY ]; then
		echo "ROS_LOCALHOST_ONLY : 0"
	else
		echo "ROS_LOCALHOST_ONLY : $ROS_LOCALHOST_ONLY"
	fi
}

# 6. 워크스페이스 단축 호출 alias 설정
alias ros2ws="ws_setting \"ros2_ws\""
alias pinky="ws_setting \"pinky_ws\""

# 7. 그외의 간단한 alias 설정
alias dl="source ~/venv/dl_venv/bin/activate; echo \"dl_venv is activated!\""
alias cw="cd ~/ros2_ws/"
alias cw="cd ~/ros2_ws/src/"

