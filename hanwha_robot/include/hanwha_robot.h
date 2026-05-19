#pragma once
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>

#include "clink_api_rpc.h"
#include "clink_api_rpc_system.h"


using namespace std;

class HanwhaRobot {
private:
	const CLINK_CBOX_MODEL	CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN;				// 펜턴트 버전
	const CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_HCR14; 			// 로봇 종류
	const std::string		CLINK_CONFIG_FILE =  std::string(HANWHA_ROOT_PATH) + "/config/config_rpc.ini"; // 한화 로봇 설정 파일

	// SMC 파라미터
	const double LAMBDA = 7.0;  													// 슬라이딩 평면 기울기 (오차 수렴 속도)
	const double K_GAIN = 3.0;   													// 제어 강도 (외란 억제력)
	const double PHI = 1.2;		 													// 진동 방지

	// PD 제어 파라미터
	const double KP = 7.0;
	const double KD = 0.00001;
	const double target_time = 0.033;

	// 제어 변수
	double cmd_joint[6] = {0.0, }; 													// 계산된 조인트 값
	double last_vel[6] = {0.0, };  													// 이전 속도 값


	std::string ip_;																// 로봇 컨트롤 박스 ip
	uint32_t cbox_id;																// 로봇 컨트롤 박스 id;
	uint32_t robot_id;																// 로봇 id
	bool rt_mode;																	// 실시간 제어 여부

public:
	HanwhaRobot(const std::string& ip) : ip_(ip)
	{
		cbox_id = 0;
		robot_id = 0;
		rt_mode = false;
	}
	~HanwhaRobot() {
		disconnect();
		rt_mode = false;
	}

	// ----------------------------------------------------------------------------
	// action
	// ----------------------------------------------------------------------------
	bool connect();																	// 로봇 연결
	void disconnect();																// 연결 끊기
	bool servo_on();																// 서보 온
	bool servo_off();																// 서보 오프

	// 제어 함수
	bool movej(const std::array<double, 6>& q, double time);						// movej [deg]

	// 실시간 제어 함수
	void start_rt(std::array<double, 6>& q);										// 실시간 제어 모드 시작
	void stop_rt();																	// 실시간 제어 모드 종료
	void movej_rt(const std::array<double, 6>& target_q, double dt);				// 실시간 제어용 movej [deg]
	bool is_rt_mode() { return rt_mode; }											// 실시간 제어 모드 여부


	// ----------------------------------------------------------------------------
	// Getter
	// ----------------------------------------------------------------------------
	std::array<double, 6> get_curr_joint_deg();										// 실제 조인트 값 [deg]
	std::array<double, 6> get_curr_joint_vel_deg();									// 실제 각속도 값 [deg/s]
	std::array<double, 6> get_curr_joint_acc_deg();									// 실제 각가속도 값 [deg/s^2]
	CLINK_MOVING_STATE    get_curr_state();											// 로봇 동작 상태

	// ----------------------------------------------------------------------------
	// 유틸리티
	// ----------------------------------------------------------------------------
	std::string moving_state_str(CLINK_MOVING_STATE state);
};