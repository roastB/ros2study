# ALIAS SETTINGS
alias tmp1="eval \"\$(register-python-argcomplete ros2)\""
alias tmp2="tmp1; eval \"\$(register-python-argcomplete colcon)\""

# 3. ROS 환경 변수 설정
MY_ID=74
# export ROS_LOCALHOST_ONLY=1 # ROS 환경에서 로컬 네트워크 내에서만 메시지를 사용하도록 설정

# 4. 자주 사용하는 명령어 단축 (alias 설정)
alias sb="source ~/.bashrc; echo \"[PC] Bashrc is reloaded!\"" # source bashrc 줄여서 sb
alias jazzy="source /opt/ros/jazzy/setup.bash; echo \"[PC] ROS2 Jazzy is activated!\""
alias rwrw="source /home/addineud/ros2_ws/install/setup.bash; echo \"[PC] ROS2_WS is activated!\""
alias commp="jetcobot_aa85; jazzy; rwrw; echo \"📡 PC Communication preparation\""

alias my_domain="export ROS_DOMAIN_ID=\$MY_ID; echo \"[PC] ROS_DOMAIN_ID is set to \$MY_ID!\"" # 변수 지정은 $로
alias my_jazzy="source /opt/ros/jazzy/setup.bash; my_domain; echo \"[PC] ROS2 Jazzy is activated!\""

alias pinky_deb1="export ROS_DOMAIN_ID=81; echo \"[PC] ROS_DOMAIN_ID is set to 81!(pinky_deb1)\""
alias pinky_dfc6="export ROS_DOMAIN_ID=82; echo \"[PC] ROS_DOMAIN_ID is set to 82!(pinky_dfc6)\""
alias pinky_dce2="export ROS_DOMAIN_ID=83; echo \"[PC] ROS_DOMAIN_ID is set to 83!(pinky_dce2)\""
alias jetcobot_aa3b="export ROS_DOMAIN_ID=84; echo \"[PC] ROS_DOMAIN_ID is set to 84!(jetcobot_aa3b)\""
alias jetcobot_aa85="export ROS_DOMAIN_ID=85; echo \"[PC] ROS_DOMAIN_ID is set to 85!(jetcobot_aa85)\""

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
alias ssh_jetcobot="ssh jetcobot@192.168.5.1"
alias ssh_pinky="ssh pinky@192.168.4.1"

alias cw="cd ~/ros2_ws/"
alias cs="cd ~/ros2_ws/src/"
alias jet="source ~/venv/jetcobot_venv/bin/activate"