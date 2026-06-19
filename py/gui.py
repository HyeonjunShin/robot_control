import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import queue
import sys

# ==========================================
# 1. 환경 설정 및 모델 로드
# ==========================================
xml_path = '/home/uon/code/robot_control/gui/m1013/m1013_mujoco.xml'

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# ==========================================
# 2. 제어 변수 및 안정된 PD 게인 설정
# ==========================================
# 최종 목적지 각도 (라디안)
target_qpos = np.zeros(6)

# 매 스텝마다 부드럽게 증가하며 target_qpos를 추종할 필터링 변수
filtered_target = np.zeros(6)

# [안정화 튜닝된 PD 게인] 
# 수치 발산(NaN)을 방지하기 위해 과도했던 Kp를 낮추고 Damping(Kd) 비율을 높였습니다.
kp = np.array([2000.0, 2000.0, 1500.0, 800.0, 800.0, 500.0])
kd = np.array([ 250.0,  250.0,  180.0,  80.0,  80.0,  40.0])

# 관절 인덱스 주소 추출
joint_names = [f"joint_{i+1}" for i in range(6)]
joint_qpos_adr = [model.joint(name).qposadr[0] for name in joint_names]
joint_qvel_adr = [model.joint(name).dofadr[0] for name in joint_names]

# 안전한 스레드 통신 큐
input_queue = queue.Queue()

# 처음 구동 시 현재 실제 로봇 위치로 타겟 초기화
initial_pos = np.array(data.qpos[joint_qpos_adr])
target_qpos = np.copy(initial_pos)
filtered_target = np.copy(initial_pos)

# ==========================================
# 3. 터미널 입력 백그라운드 스레드
# ==========================================
def input_worker():
    while True:
        try:
            print("\n[입력] 6개 관절 각도(도, Degree)를 공백으로 구분해 입력하세요.")
            print("👉 예시 입력: 20 0 -30 0 45 10")
            print("👉 ", end="")
            sys.stdout.flush()
            
            line = sys.stdin.readline().strip()
            if not line:
                continue
                
            parts = line.split()
            if len(parts) != 6:
                print(f"❌ 에러: 6개의 숫자가 필요합니다. (현재 입력: {len(parts)}개)")
                continue
                
            raw_degrees = np.array([float(p) for p in parts])
            input_queue.put(raw_degrees)
            
        except ValueError:
            print("❌ 에러: 올바른 숫자 형식이 아닙니다.")
        except Exception as e:
            print(f"❌ 스레드 에러: {e}")
            break

threading.Thread(target=input_worker, daemon=True).start()

print("====================================================")
print("  두산 M1013 안전 급발진 방지 제어기 구동 ")
print("  - 입력 시 궤적 필터가 적용되어 부드럽게 이동합니다.")
print("====================================================")

# ==========================================
# 4. 메인 시뮬레이션 및 실시간 제어 루프
# ==========================================
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()
        
        # 새로운 입력이 큐에 들어왔는지 감지
        if not input_queue.empty():
            try:
                new_degrees = input_queue.get_nowait()
                target_qpos = np.radians(new_degrees)
                print(f"📌 [목표 접수] {new_degrees} 도 성분으로 부드럽게 궤적 이동을 시작합니다.")
            except queue.Empty:
                pass
        
        # 매 스텝 외력 초기화
        data.qfrc_applied[:] = 0.0
        
        # -----------------------------------------------------------------
        # [핵심] 1차 저역통과필터(Low-pass Filter) 기반 급발진 억제
        # alpha 수치가 작을수록(예: 0.01) 로봇이 더 묵직하고 느리게 이동합니다.
        # -----------------------------------------------------------------
        alpha = 0.02  
        filtered_target = (1 - alpha) * filtered_target + alpha * target_qpos
        
        # 현재 로봇 물리 상태 계측
        current_qpos = data.qpos[joint_qpos_adr]
        current_qvel = data.qvel[joint_qvel_adr]
        
        # 필터링된 타겟과의 오차를 계산하므로 초기 순간 오차가 극도로 작아집니다.
        error = filtered_target - current_qpos
        ctrl_torque = kp * error - kd * current_qvel
        
        # 토크 인가
        data.qfrc_applied[joint_qvel_adr] = ctrl_torque
        
        # 물리 스텝 진행 및 화면 동기화
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 2ms 정밀 주기 제어
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)