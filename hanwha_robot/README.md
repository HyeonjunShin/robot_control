# 한화 로봇 제어 라이브러리

## 라이브러리 추가 방법

프로젝트 메인 CMakeLists.txt
```cmake

add_subdirectory(hanwha_robot) # 추가 하세요

target_link_libraries(main
        hanwha_robot   # 추가 하세요
)
```

<br>

## 사용 예제

main.cpp
```c++
#include <iostream>
#include <thread>
#include "hanwha_robot.h"
#define SLEEP(x) std::this_thread::sleep_for( std::chrono::milliseconds(x))

int main() {
    
    // 생성
    HanwhaRobot robot("192.168.100.200");
    
    // 연결
    robot.connect();
    SLEEP(1000);
    
    // 서보 온
    robot.servo_on();
    SLEEP(1000);
    
    // movej
    std::array<double, 6> target;
    target = {0, -90, -90, -90, 90, 0};
    double time = 5;
    robot.movej(target, time); // 5초에 걸쳐서 목표 자세로 이동함
    
    SLEEP(1000);
    
    // 연결 해제
    robot.disconnect();
    
    return 0;
}
```
<br>

## 실시간 제어


> [!NOTE]
> 실시간 제어를 할 경우 제어 PC에서 아래 명령을 입력하세요.
>
> 
> ```shell
>  sudo ip route add 192.168.222.0/24 via 192.168.100.200
> ```
> 그리고 ping 192.168.222.222 명령이 된다면 실시간 제어가 가능합니다.



> [!NOTE]
> ping 192.168.222.222 명령이 안될 경우 한화 로봇 펜던트에서 직접 ip를 세팅해야 합니다.\n
> 1. 작업 관리자 실행:<kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>Esc</kbd> 
> 2. 새 작업 실행: Run new task 클릭
> 3. 네트워크 설정 진입: 명령 입력창에 ncpa.cpl 입력
> 4. IP 수정: TenAsys Virtual Ethernet 항목을 찾아 아래 IP로 수정합니다. (IP 주소: 192.168.222.222)


![IMG_20260415_151745.jpg](contents/screenshot/IMG_20260415_151745.jpg)