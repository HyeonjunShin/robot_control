#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h> 

// --- 전역 변수 및 MuJoCo 객체 ---
mjModel* m = nullptr;                  
mjData* d = nullptr;                   
mjvScene scn;                          
mjrContext con;                        
mjvOption opt;                         
mjvCamera cam;                         

// 스레드 간 데이터 동기화를 위한 뮤텍스
std::mutex mtx;

// 시뮬레이션 종료 플래그
bool is_running = true;

const std::string urdf_path = "../gui/m1013/m1013.urdf"; 

// --- GLFW 콜백 함수들 ---
bool button_left = false;
double lastx = 0; double lasty = 0;

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE) 
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) 
        button_left = (act == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (button_left) {
        double dx = xpos - lastx; 
        double dy = ypos - lasty;
        lastx = xpos; 
        lasty = ypos;
        std::lock_guard<std::mutex> lock(mtx); // 카메라 조작 시 데이터 보호
        mjv_moveCamera(m, mjMOUSE_ROTATE_V, dx / 200.0, dy / 200.0, &scn, &cam);
    }
}

// =================================================================
// [1] 물리 시뮬레이션 & 로봇 제어 전용 스레드 함수
// =================================================================
void controlLoop() {
    const double dt = 0.002; 
    
    // 이전에 맞춘 안정적인 게인 값 사용
    double Kp = 1500.0;  
    double Kd = 1000.0;   
    
    int num_dof = m->nv;
    std::vector<double> target_positions(num_dof, 0.0);

    std::cout << "[Control Thread] 제어 스레드가 시작되었습니다." << std::endl;

    while (is_running) {
        auto start_time = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mtx);

            // =================================================================
            // 💡 [핵심] 시간에 따라 변하는 흔들기 모션 (Sine Wave Trajectory)
            // =================================================================
            // d->time은 MuJoCo 시뮬레이션의 가상 시간(초)입니다.
            double t = d->time;

            // 예시: 1번 관절(보통 어깨나 베이스)을 중심축으로 두고 양옆으로 흔들기
            // 수식 설명: 중심 각도 + (흔들기 반경 * sin(속도 * 시간))
            // 아래 코드는 0도를 기준으로 ±0.5 라디안(약 ±28도) 범위를 왕복합니다.
            target_positions[0] = 0.0 + (0.5 * sin(2.0 * t)); 

            // 만약 2번 관절(팔꿈치 등)도 같이 위아래로 흔들고 싶다면 엇박자로 섞어줄 수 있습니다.
            target_positions[1] = -0.5 + (0.3 * cos(2.0 * t));
            target_positions[2] = 0.5; // 나머지 관절은 고정
            // =================================================================

            // --- PD 제어 토크 계산 및 인가 (기존 코드 유지) ---
            for (int i = 0; i < num_dof; ++i) {
                double current_pos = d->qpos[i];
                double current_vel = d->qvel[i];

                double pos_error = target_positions[i] - current_pos;
                double vel_error = 0.0 - current_vel;

                double torque = (Kp * pos_error) + (Kd * vel_error);

                const double max_torque = 300.0; 
                if (torque > max_torque)  torque = max_torque;
                if (torque < -max_torque) torque = -max_torque;

                d->qfrc_applied[i] = torque;
            }

            mj_step(m, d);
        }
        
        // 정주기 타이밍 연산 (기존 코드 유지)
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        auto sleep_time = std::chrono::microseconds(static_cast<int>(dt * 1e6)) - elapsed;
        if (sleep_time > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
}

// =================================================================
// [2] 메인 스레드 (오직 GUI 렌더링만 담당)
// =================================================================
int main() {
    std::cout << "MuJoCo 로딩 중... 버전: " << mj_versionString() << std::endl;
    if (!glfwInit()) return 1;

    char error[1000] = "Could not program application";
    m = mj_loadXML(urdf_path.c_str(), nullptr, error, 1000);
    if (!m) {
        std::cerr << "URDF 로드 실패: " << error << std::endl;
        glfwTerminate();
        return 1;
    }
    std::cout << "URDF '" << urdf_path << "' 로드 성공!" << std::endl;

    // [핵심 수정] mj_makeData가 호출되기 전에 물리 엔진 내부의 timestep 설정을 먼저 일치시켜야 안전합니다.
    m->opt.timestep = 0.002; 

    d = mj_makeData(m);
    mjv_makeScene(m, &scn, 1000);
    mjv_defaultOption(&opt);
    mjv_defaultCamera(&cam);
    
    // 카메라 기본 뷰 설정 (로봇의 크기에 맞춰 정렬)
    cam.distance = m->stat.extent * 1.5;
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];

    // GLFW 윈도우 창 생성
    GLFWwindow* window = glfwCreateWindow(1200, 900, "URDF Robot Multi-Thread Control", nullptr, nullptr);
    if (!window) {
        mj_deleteData(d); mj_deleteModel(m); mjv_freeScene(&scn);
        glfwTerminate(); return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 🚀 [제어 스레드 시작] 물리 및 로봇 제어를 독립된 스레드로 가동합니다.
    std::thread control_thread(controlLoop);

    // --- GUI 렌더링 루프 (모니터 주사율에 맞춰 유연하게 화면을 갱신) ---
    while (!glfwWindowShouldClose(window)) {
        {
            // 제어 스레드가 물리 데이터를 변경하는 도중 화면을 그리면 깨질 수 있으므로 동기화 잠금
            std::lock_guard<std::mutex> lock(mtx);
            mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        }

        mjrRect viewport = {0, 0, 1200, 900};
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // --- 프로그램 종료 처리 ---
    is_running = false;          // 제어 스레드 무한루프 종료 신호
    if (control_thread.joinable()) {
        control_thread.join();   // 제어 스레드가 안전하게 멈출 때까지 대기
    }

    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}