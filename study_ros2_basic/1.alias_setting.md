**source로 ROS Humble의 setup.bash를 불러오기 (환경불러오기)**

```jsx
source /opt/ros/humble/setup.bash
```

**ALIAS SETTINGS**

```bash
# ALIAS SETTINGS
# 1.1 중요한 역할을 하는 .bashrc 파일을 직접 수정하는 것은 위험할 수 있으므로,
#     별도의 설정 파일 alias_setting.sh에 설정을 분리하여 관리합니다.
# 1.2 .bashrc 하단에 다음과 같은 한 줄을 추가하여 설정을 적용합니다:
#     source ~/alias_setting.sh

# 2. ROS2 및 Colcon의 자동 완성을 활성화하기 위한 alias
# eval : 문자열로 된 명령어를 실행시키는 역할
alias tmp1="eval \"\$(register-python-argcomplete3 ros2)\""
alias tmp2="tmp1; eval \"\$(register-python-argcomplete3 colcon)\""

# 3. ROS 환경 변수 설정
ID=25
export ROS_LOCALHOST_ONLY=1 # ROS 환경에서 로컬 네트워크 내에서만 메시지를 사용하도록 설정

# 4. 자주 사용하는 명령어 단축 (alias 설정)
alias sb="source ~/.bashrc; echo \"Bashrc is reloaded!\"" # source bashrc 줄여서 sb
alias ros_domain="export ROS_DOMAIN_ID=\$ID; echo \"ROS_DOMAINID is set to \$ID!\"" # 변수 지정은 $로
alias humble="source /opt/ros/humble/setup.bash; ros_domain; echo \"ROS2 Humble is activated!\""
alias gazebo="source /usr/share/gazebo/setup.sh; echo \"Gazebo is ready!\""

# 5. Bash 함수 선언
# (1) 워크스페이스 설정 함수
ws_setting()
{
	humble
	gazebo
	source ~/$1/install/local_setup.bash  # 입력 인자는 ${num}으로 받는다
	echo "$1 workspace is activated!"
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
alias ros2study="ws_setting \"ros2_study\""
alias urdfstudy="ws_setting \"urdf_study\""
alias my_mobile="ws_setting \"my_mobile\""
alias bt_tutorials="ws_setting \"bt_tutorials\""

```
