#include "hanwha_robot.h"

bool HanwhaRobot::connect()
{
    CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;

    // 컨트롤 박스 연결
    err_ret_val = clink_rpc_system_cbox_connect(CLINK_CONFIG_FILE.c_str(), ip_.c_str(), &cbox_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        cout << "[Error] [HanwhaRobot] 컨트롤 박스 연결 실패: " << err_ret_val << endl;
        return false;
    }

    //  제어SW 초기화
    err_ret_val = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
    if (err_ret_val != CLINK_API_RESULT_OK && err_ret_val < CLINK_API_RESULT_WARNING_BEGIN) {
        cout << "[Error] [HanwhaRobot] 제어SW 초기화 실패: " << err_ret_val << endl;
        return false;
    }

    // 제어권 획득
    err_ret_val = clink_rpc_system_control_take(cbox_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        cout << "[Error] [HanwhaRobot] 제어권 획득 실패" << endl;
        return false;
    }

    // 로봇 생성
    err_ret_val = clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, "", 0U, &robot_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        cout << "[Error] [HanwhaRobot] 로봇 생성 실패" << endl;
        return false;
    }

    // EtherCAT 상태 확인 및 대기
    CLINK_ECAT_CONN_STATE ecat_stat = CLINK_ECAT_CONN_STATE_DISCONNECTED;
    clink_rpc_cbox_ecat_connection_state_get(cbox_id, &ecat_stat);
    if (CLINK_ECAT_CONN_STATE_CONNECTED != ecat_stat) {
        char_t valid_event = -1;
        clink_rpc_system_wait_event_group_subgroup(
            cbox_id,
            CLINK_EVENT_GRP_NOTIFICATION,
            CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,
            1000000,
            1,
            &valid_event);
    }

    // 6. 자동 속도 조절 기능 ON
    clink_rpc_robot_motion_auto_adjust_swith_set(cbox_id, robot_id, CLINK_SWITCH_ON);

    // --- 설정된 각속도 및 각가속도 값 출력 ---
    // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 0, 200);
    // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 1, 200);
    // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 2, 200);
    // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 3, 270);
    // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 4, 270);
    // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 5, 270);


    cout << "\n[Info] [HanwhaRobot] --- 설정된 조인트 한계값 (Safety Limit) ---" << endl;

    for (uint32_t i = 0; i < 6; i++) {
        clink_float_t set_speed_max = 0.0f;
        clink_float_t set_acc_max = 0.0f;

        // 설정된 최대 속도 가져오기 (degree/s)
        clink_rpc_robot_safety_limit_joint_speed_max_get(cbox_id, robot_id, i, &set_speed_max);
        // 설정된 최대 가속도 가져오기 (degree/s^2)
        clink_rpc_robot_safety_limit_joint_acc_max_get(cbox_id, robot_id, i, &set_acc_max);

        cout << "[Info] Joint " << i << " | 설정 각속도: " << set_speed_max
             << " deg/s | 설정 각가속도: " << set_acc_max << " deg/s^2" << endl;
    }

    // 글로벌 스피드 팩터(전체 속도 비율) 출력
    clink_float_t speed_factor = 0.0f;
    clink_rpc_robot_speed_factor_get(cbox_id, robot_id, &speed_factor);
    cout << "[Info] 현재 시스템 Speed Factor: " << speed_factor * 100.0f << " %" << endl;
    cout << "---------------------------------------------------------\n" << endl;

    cout << "[Info] [HanwhaRobot] 로봇 연결 및 초기화 완료: " << ip_ << endl;
    return true;
}
void HanwhaRobot::disconnect()
{
    // 안전 상 로봇 stop 및 servo off 실행 후 종료
    clink_rpc_robot_stop(cbox_id, robot_id, 0.5);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
    clink_rpc_system_control_release(cbox_id);

    cout << "[Info] [HanwhaRobot] 로봇 연결 해제: " << ip_ << endl;
}

bool HanwhaRobot::servo_on()
{

    CLINK_SWITCH result;
    clink_rpc_robot_servo_switch_get(cbox_id, robot_id, &result);
    if (result == CLINK_SWITCH_ON)
        return true;

    CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;

    err_ret_val = clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        cout << "[Error] [HanwhaRobot] 서보 온 실패: "  << endl;
        return false;
    }

    cout << "[Info] [HanwhaRobot] 서보 온"  << endl;
    return true;
}

bool HanwhaRobot::servo_off()
{
    CLINK_SWITCH result;
    clink_rpc_robot_servo_switch_get(cbox_id, robot_id, &result);
    if (result == CLINK_SWITCH_OFF)
        return true;


    CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;

    err_ret_val = clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        cout << "[Error] [HanwhaRobot] 서보 오프 실패: "  << endl;
        return false;
    }

    cout << "[Info] [HanwhaRobot] 서보 오프"  << endl;
    return true;
}

bool HanwhaRobot::movej(const std::array<double, 6> &q, double time)
{
    if (time <= 0) return false;

    // 실제 관절 각도 [deg]
    clink_float_t current_q[6];
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 0, &current_q[0]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 1, &current_q[1]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 2, &current_q[2]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 3, &current_q[3]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 4, &current_q[4]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 5, &current_q[5]);

    // 가장 많이 움직이는 축의 거리 계산
    double max_diff = 0.0;
    for (int i = 0; i < 6; i++) {
        double diff = std::abs(q[i] - current_q[i]);
        if (diff > max_diff) max_diff = diff;
    }

    // 속도 계산 (Speed = Distance / Time)
    clink_float_t spd = static_cast<clink_float_t>(max_diff / time);

    // 최소 속도
    if (spd < 0.1) spd = 0.1;

    // 가속도 설정 (속도의 4배)
    clink_float_t acc = spd * 4.0;
    clink_float_t jerk = 0.5;

    // 4. 모션 명령 생성 및 실행
    clink_rpc_system_event_queue_clear(cbox_id);

    uint32_t cmd_id = 0;
    clink_rpc_motion_command_robot_joint_create(cbox_id, &cmd_id);
    clink_rpc_motion_command_robot_robot_id_set(cbox_id, cmd_id, robot_id);

    for (uint32_t j_idx = 0; j_idx < 6; j_idx++)
    {
        clink_rpc_motion_command_robot_joint_angle_end_set(
            cbox_id,
            cmd_id,
            j_idx,
            static_cast<clink_float_t>(q[j_idx]));
    }

    clink_rpc_motion_command_velocity_profiler_set(cbox_id, cmd_id, CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE);
    clink_rpc_motion_command_acc_max_set(cbox_id, cmd_id, acc);
    clink_rpc_motion_command_dec_max_set(cbox_id, cmd_id, acc);
    clink_rpc_motion_command_speed_max_set(cbox_id, cmd_id, spd);
    clink_rpc_motion_command_jerk_percentage_set(cbox_id, cmd_id, jerk);
    clink_rpc_motion_command_blending_motion_mode_set(cbox_id, cmd_id, CLINK_SWITCH_OFF);

    clink_rpc_motion_command_queue_add(cbox_id, cmd_id);
    clink_rpc_motion_command_queue_flush(cbox_id);
    clink_rpc_motion_command_destroy(cbox_id, cmd_id);

    return true;
}

void HanwhaRobot::start_rt(std::array<double, 6>& q)
{
    // 초기 각도로 이동 [3 sec]
    movej(q, 3);

    char_t valid_event = -1;
    clink_rpc_system_wait_event_group_subgroup(
        cbox_id,
        CLINK_EVENT_GRP_MOTION_COMMAND,
        CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
        10000,
        1,
        &valid_event);

    clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_ON);

    // 제어 변수에 현제 각도 설정
    cmd_joint[0] = q[0];
    cmd_joint[1] = q[1];
    cmd_joint[2] = q[2];
    cmd_joint[3] = q[3];
    cmd_joint[4] = q[4];
    cmd_joint[5] = q[5];

    rt_mode = true;
}

void HanwhaRobot::stop_rt()
{
    clink_rpc_robot_joint_chaser_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
    rt_mode = false;
}

void HanwhaRobot::movej_rt(const std::array<double, 6>& target_q, double dt)
{
    if (!rt_mode)   return;
    if (dt <= 1e-6) return;
    clink_float_t next_target[6];

    // SMC 제어기
    for (int i = 0; i < 6; i++) {
        double error = target_q[i] - cmd_joint[i];

        // (s = de/dt + lambda * e)
        double s = LAMBDA * error - last_vel[i];

        // 제어 입력
        double reach_vel = K_GAIN * std::tanh(s / PHI);
        double current_vel = reach_vel + (LAMBDA * error);

        // 업데이트
        cmd_joint[i] += current_vel * dt;
        last_vel[i] = current_vel;

        next_target[i] = static_cast<clink_float_t>(cmd_joint[i]);
    }

    // 조인트 체이서 명령
    clink_rpc_robot_joint_chaser_move(cbox_id, robot_id, 6, next_target);

    // // PD 제어기
    // if (dt <= 1e-6) return;
    //
    // clink_float_t next_target[6];
    // double T = target_time;
    //
    // for (int i = 0; i < 6; i++) {
    // 	// 오차 계산
    // 	double error = target_q[i] - cmd_joint[i];
    //
    // 	// 속도 생성
    // 	double ref_vel = (error / T) * 1.875;
    //
    // 	//  PD 제어
    // 	double error_dot = ref_vel - last_vel[i];
    // 	double current_vel = (error * KP) + (error_dot * KD);
    //
    // 	// 상태 업데이트
    // 	cmd_joint[i] += current_vel * dt;
    // 	last_vel[i] = current_vel;
    //
    // 	next_target[i] = static_cast<clink_float_t>(cmd_joint[i]);
    // }
    //
    //
    // // 조인트 체이서 명령 전송
    // clink_rpc_robot_joint_chaser_move(cbox_id, robot_id, 6, next_target);
}

std::array<double, 6> HanwhaRobot::get_curr_joint_deg()
{
    clink_float_t angle[6];
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 0, &angle[0]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 1, &angle[1]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 2, &angle[2]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 3, &angle[3]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 4, &angle[4]);
    clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, 5, &angle[5]);

    std::array<double, 6> joint_deg;
    joint_deg[0] = angle[0];
    joint_deg[1] = angle[1];
    joint_deg[2] = angle[2];
    joint_deg[3] = angle[3];
    joint_deg[4] = angle[4];
    joint_deg[5] = angle[5];

    return joint_deg;
}

std::array<double, 6> HanwhaRobot::get_curr_joint_vel_deg()
{
    clink_float_t vel[6];
    clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, 0, &vel[0]);
    clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, 1, &vel[1]);
    clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, 2, &vel[2]);
    clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, 3, &vel[3]);
    clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, 4, &vel[4]);
    clink_rpc_robot_joint_speed_actual_get(cbox_id, robot_id, 5, &vel[5]);

    std::array<double, 6> joint_vel;
    joint_vel[0] = vel[0];
    joint_vel[1] = vel[1];
    joint_vel[2] = vel[2];
    joint_vel[3] = vel[3];
    joint_vel[4] = vel[4];
    joint_vel[5] = vel[5];

    return joint_vel;
}

std::array<double, 6> HanwhaRobot::get_curr_joint_acc_deg()
{
    clink_float_t acc[6];
    clink_rpc_robot_joint_acc_actual_get(cbox_id, robot_id, 0, &acc[0]);
    clink_rpc_robot_joint_acc_actual_get(cbox_id, robot_id, 1, &acc[1]);
    clink_rpc_robot_joint_acc_actual_get(cbox_id, robot_id, 2, &acc[2]);
    clink_rpc_robot_joint_acc_actual_get(cbox_id, robot_id, 3, &acc[3]);
    clink_rpc_robot_joint_acc_actual_get(cbox_id, robot_id, 4, &acc[4]);
    clink_rpc_robot_joint_acc_actual_get(cbox_id, robot_id, 5, &acc[5]);

    std::array<double, 6> joint_acc;
    joint_acc[0] = acc[0];
    joint_acc[1] = acc[1];
    joint_acc[2] = acc[2];
    joint_acc[3] = acc[3];
    joint_acc[4] = acc[4];
    joint_acc[5] = acc[5];

    return joint_acc;
}

CLINK_MOVING_STATE HanwhaRobot::get_curr_state()
{
    CLINK_MOVING_STATE res;
    clink_rpc_robot_state_moving_get(cbox_id, robot_id, & res);
    return res;
}

std::string HanwhaRobot::moving_state_str(CLINK_MOVING_STATE state)
{
    switch (state) {
        case CLINK_MOVING_STATE_IDLE:
            return "CLINK_MOVING_STATE_IDLE";

        case CLINK_MOVING_STATE_MOVING:
            return "CLINK_MOVING_STATE_MOVING";

        case CLINK_MOVING_STATE_PAUSING:
            return "CLINK_MOVING_STATE_PAUSING";

        case CLINK_MOVING_STATE_PAUSED:
            return "CLINK_MOVING_STATE_PAUSED";

        case CLINK_MOVING_STATE_STOPPING:
            return "CLINK_MOVING_STATE_STOPPING";

        case CLINK_MOVING_STATE_STOPPED:
            return "CLINK_MOVING_STATE_STOPPED";

        case CLINK_MOVING_STATE_EMG_STOPPING:
            return "CLINK_MOVING_STATE_EMG_STOPPING";

        case CLINK_MOVING_STATE_EMG_STOPPED:
            return "CLINK_MOVING_STATE_EMG_STOPPED";

        case CLINK_MOVING_STATE_INVALID:
            return "CLINK_MOVING_STATE_INVALID";

        default:
            return "UNKNOWN_STATE";
    }
}
