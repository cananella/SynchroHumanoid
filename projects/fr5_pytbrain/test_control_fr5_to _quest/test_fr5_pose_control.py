#!/usr/bin/env python3
"""
FR5 로봇 Pose 제어 테스트 스크립트

이 스크립트는 FR5 로봇의 다양한 pose 제어 방법을 테스트합니다:
1. 현재 pose 읽기
2. ServoCart를 이용한 증분 제어
3. MoveCart를 이용한 절대 위치 제어
"""

import sys
import time

from sdk_paths import setup_paths
setup_paths()

from third_party.fairino_python_sdk.linux.fairino import Robot

# ==================== 설정 ====================
ROBOT_IP = "192.168.58.2"  # FR5 로봇 IP 주소 (필요시 수정)

# ==================== 로봇 상태 초기화 함수 ====================
def reset_robot_state(robot):
    """로봇 상태를 완전히 초기화"""
    print("[INFO] 로봇 상태 완전 초기화 중...")

    # 1. 서보 모드 강제 종료
    robot.ServoMoveEnd()
    time.sleep(0.1)

    # 2. 모든 운동 정지
    robot.StopMotion()
    time.sleep(0.1)

    # 3. 모든 에러 초기화
    error = robot.ResetAllError()
    print(f"  [INFO] ResetAllError() 결과: {error}")
    time.sleep(0.2)

    # 4. 자동 모드 재설정
    robot.Mode(0)
    time.sleep(0.1)

    print("[SUCCESS] 로봇 상태 초기화 완료")

# ==================== 로봇 연결 ====================
def connect_robot(ip):
    """로봇에 연결"""
    print(f"[INFO] 로봇 연결 중... IP: {ip}")
    try:
        robot = Robot.RPC(ip)
        print("[SUCCESS] 로봇 연결 성공!")

        # 자동 모드로 전환
        print("[INFO] 로봇을 자동 모드로 전환 중...")
        error = robot.Mode(0)  # 0 = 자동 모드, 1 = 수동 모드
        if error == 0:
            print("[SUCCESS] 자동 모드 전환 완료!")
        else:
            print(f"[WARNING] 모드 전환 실패. 에러 코드: {error}")
            print("[INFO] 티치 펜던트에서 수동으로 자동 모드로 전환해주세요.")

        return robot
    except Exception as e:
        print(f"[ERROR] 로봇 연결 실패: {e}")
        sys.exit(1)


# ==================== 테스트 1: 현재 Pose 읽기 ====================
def test_get_current_pose(robot):
    """현재 TCP pose와 플랜지 pose 읽기"""
    print("\n" + "="*60)
    print("테스트 1: 현재 Pose 읽기")
    print("="*60)

    # TCP pose 읽기
    error, tcp_pose = robot.GetActualTCPPose(flag=1)
    if error == 0:
        print(f"[TCP Pose] x={tcp_pose[0]:.2f}, y={tcp_pose[1]:.2f}, z={tcp_pose[2]:.2f}, "
              f"rx={tcp_pose[3]:.2f}, ry={tcp_pose[4]:.2f}, rz={tcp_pose[5]:.2f}")
    else:
        print(f"[ERROR] TCP pose 읽기 실패. 에러 코드: {error}")
        return None

    # 플랜지 pose 읽기
    error, flange_pose = robot.GetActualToolFlangePose(flag=1)
    if error == 0:
        print(f"[Flange Pose] x={flange_pose[0]:.2f}, y={flange_pose[1]:.2f}, z={flange_pose[2]:.2f}, "
              f"rx={flange_pose[3]:.2f}, ry={flange_pose[4]:.2f}, rz={flange_pose[5]:.2f}")
    else:
        print(f"[ERROR] 플랜지 pose 읽기 실패. 에러 코드: {error}")

    # 관절 각도도 읽기
    error, joint_pos = robot.GetActualJointPosDegree(flag=1)
    if error == 0:
        print(f"[Joint Pos] j1={joint_pos[0]:.2f}°, j2={joint_pos[1]:.2f}°, j3={joint_pos[2]:.2f}°, "
              f"j4={joint_pos[3]:.2f}°, j5={joint_pos[4]:.2f}°, j6={joint_pos[5]:.2f}°")

    # 로봇 상태 패키지 확인 (에러 14 원인 파악)
    print("\n[로봇 상태 진단]")
    try:
        print(f"  program_state: {robot.robot_state_pkg.program_state}")
        print(f"  robot_state: {robot.robot_state_pkg.robot_state}")
        print(f"  robot_mode: {robot.robot_state_pkg.robot_mode}")
        print(f"  main_code: {robot.robot_state_pkg.main_code}")
        print(f"  sub_code: {robot.robot_state_pkg.sub_code}")

        # 추가 에러 진단
        error_code = robot.GetRobotErrorCode()
        print(f"  robot_error_code: {error_code}")

        emergency_state = robot.GetRobotEmergencyStopState()
        print(f"  emergency_stop_state: {emergency_state}")

        error_safety, safety_state = robot.GetSafetyStopState()
        print(f"  safety_stop_state: {safety_state} (error: {error_safety})")

        safety_code = robot.GetSafetyCode()
        print(f"  safety_code: {safety_code}")

    except Exception as e:
        print(f"  [WARNING] 로봇 상태 패키지 읽기 실패: {e}")

    return tcp_pose


# ==================== 테스트 2: ServoCart 증분 제어 ====================
def test_servo_cart_incremental(robot):
    """ServoCart를 이용한 증분 제어 테스트"""
    print("\n" + "="*60)
    print("테스트 2: ServoCart 증분 제어")
    print("="*60)

    # 혹시 이전에 서보 모드가 남아있을 수 있으니 먼저 종료
    print("[INFO] 이전 서보 모드 정리 중...")
    robot.ServoMoveEnd()
    time.sleep(0.2)

    print("[INFO] 서보 모드 시작...")
    error = robot.ServoMoveStart()
    if error != 0:
        print(f"[ERROR] 서보 모드 시작 실패. 에러 코드: {error}")
        return

    print("[INFO] Z축 상승 테스트 (10mm 위로 이동 - 기준 좌표계)")
    # 증분: [dx, dy, dz, drx, dry, drz]
    incremental_pose = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]  # Z축으로 0.05mm씩 (속도 제한 준수)

    try:
        for i in range(100):  # 200번 반복 = 10mm 이동
            error = robot.ServoCart(
                mode=2,                 # mode=1 (기준 좌표계 - 세계 좌표 기준)
                desc_pos=incremental_pose,
                vel=5,                  # vel 더 낮춤
                cmdT=0.003              # 제어 주기를 16ms로 늘림 (속도 절반)
            )

            if error != 0:
                print(f"[ERROR] ServoCart 실패. 에러 코드: {error}")
                break

            if (i + 1) % 20 == 0:
                print(f"  진행: {i+1}/100 단계")

            time.sleep(0.003)  # cmdT와 동일하게  # 8ms 대기

        print("[SUCCESS] Z축 상승 완료!")
        time.sleep(0.5)

        # 다시 내려오기
        print("[INFO] Z축 하강 테스트 (10mm 아래로 이동 - 기준 좌표계)")
        incremental_pose = [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]  # 상승과 동일한 크기, 반대 방향

        for i in range(100):  # 상승과 동일하게 200번 반복 = 10mm 이동
            error = robot.ServoCart(
                mode=2,                 # 상승과 동일한 mode (기준 좌표계)
                desc_pos=incremental_pose,
                vel=5,                  # 상승과 동일한 vel
                cmdT=0.003              # 상승과 동일한 cmdT
            )

            if error != 0:
                print(f"[ERROR] ServoCart 실패. 에러 코드: {error}")
                break

            if (i + 1) % 20 == 0:
                print(f"  진행: {i+1}/100 단계")

            time.sleep(0.003)  # 상승과 동일한 주기

        print("[SUCCESS] Z축 하강 완료!")

    finally:
        print("[INFO] 서보 모드 종료...")
        robot.ServoMoveEnd()

        # 에러가 발생했다면 완전 초기화
        if error != 0:
            print("[WARNING] 에러가 발생했습니다. 로봇 상태를 완전히 초기화합니다.")
            reset_robot_state(robot)


# ==================== 테스트 3: ServoCart 회전 제어 ====================
def test_servo_cart_rotation(robot):
    """ServoCart를 이용한 회전 제어 테스트"""
    print("\n" + "="*60)
    print("테스트 3: ServoCart 회전 제어")
    print("="*60)

    # 혹시 이전에 서보 모드가 남아있을 수 있으니 먼저 종료
    print("[INFO] 이전 서보 모드 정리 중...")
    robot.ServoMoveEnd()
    time.sleep(0.2)

    print("[INFO] 서보 모드 시작...")
    error = robot.ServoMoveStart()
    if error != 0:
        print(f"[ERROR] 서보 모드 시작 실패. 에러 코드: {error}")
        return

    print("[INFO] RZ축 회전 테스트 (0.5도 회전)")
    # 증분: [dx, dy, dz, drx, dry, drz]
    incremental_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.05]  # RZ축으로 0.05도씩 10번 = 0.5도

    try:
        for i in range(10):  # 10번 반복 = 0.5도 회전
            error = robot.ServoCart(
                mode=2,                 # mode=2로 변경 (예제와 동일)
                desc_pos=incremental_pose,
                vel=5,                  # vel 더 낮춤
                cmdT=0.016              # 제어 주기를 16ms로 늘림
            )

            if error != 0:
                print(f"[ERROR] ServoCart 실패. 에러 코드: {error}")
                print(f"[INFO] 에러 발생 시점: {i+1}/20 단계")
                print("[INFO] 테스트를 중단합니다. 로봇 상태를 확인하세요.")
                break

            time.sleep(0.016)  # cmdT와 동일하게

        print("[SUCCESS] RZ축 회전 완료!")
        time.sleep(0.5)

        # 다시 되돌리기 (에러가 없었을 때만)
        if error == 0:
            print("[INFO] RZ축 역회전 테스트 (10도 역회전)")
            incremental_pose = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]

            for i in range(10):  # 20번 → 10번으로 변경
                error = robot.ServoCart(
                    mode=1,
                    desc_pos=incremental_pose,
                    cmdT=0.008
                )

                if error != 0:
                    print(f"[ERROR] ServoCart 실패. 에러 코드: {error}")
                    print(f"[INFO] 에러 발생 시점: {i+1}/20 단계")
                    break

                time.sleep(0.008)

        print("[SUCCESS] RZ축 역회전 완료!")

    finally:
        print("[INFO] 서보 모드 종료...")
        robot.ServoMoveEnd()

        # 에러가 발생했다면 완전 초기화
        if error != 0:
            print("[WARNING] 에러가 발생했습니다. 로봇 상태를 완전히 초기화합니다.")
            reset_robot_state(robot)


# ==================== 테스트 4: MoveCart 절대 위치 제어 ====================
def test_move_cart_absolute(robot, current_pose):
    """MoveCart를 이용한 절대 위치 제어 테스트"""
    print("\n" + "="*60)
    print("테스트 4: MoveCart 절대 위치 제어")
    print("="*60)

    if current_pose is None:
        print("[ERROR] 현재 pose 정보가 없습니다. 테스트를 건너뜁니다.")
        return

    # 현재 위치에서 Z축 20mm 위로 이동
    target_pose = current_pose.copy()
    target_pose[2] += 20.0  # Z축 +20mm

    print(f"[INFO] 목표 pose로 이동 중...")
    print(f"  현재 Z: {current_pose[2]:.2f}mm")
    print(f"  목표 Z: {target_pose[2]:.2f}mm")

    error = robot.MoveCart(
        desc_pos=target_pose,
        tool=0,
        user=0,
        vel=30.0,       # 속도 30%
        blendT=-1.0     # 블로킹 모드 (도착까지 대기)
    )

    if error == 0:
        print("[SUCCESS] 목표 위치 도착!")
    else:
        print(f"[ERROR] MoveCart 실패. 에러 코드: {error}")
        return

    time.sleep(0.5)

    # 다시 원래 위치로 복귀
    print(f"[INFO] 원래 위치로 복귀 중...")
    error = robot.MoveCart(
        desc_pos=current_pose,
        tool=0,
        user=0,
        vel=30.0,
        blendT=-1.0
    )

    if error == 0:
        print("[SUCCESS] 원래 위치 복귀 완료!")
    else:
        print(f"[ERROR] MoveCart 실패. 에러 코드: {error}")


# ==================== 테스트 5: 사각형 그리기 ====================
def test_draw_square(robot):
    """ServoCart를 이용해 XY 평면에 사각형 그리기"""
    print("\n" + "="*60)
    print("테스트 5: 사각형 그리기 (10mm x 10mm)")
    print("="*60)

    # 혹시 이전에 서보 모드가 남아있을 수 있으니 먼저 종료
    print("[INFO] 이전 서보 모드 정리 중...")
    robot.ServoMoveEnd()
    time.sleep(0.2)

    print("[INFO] 서보 모드 시작...")
    error = robot.ServoMoveStart()
    if error != 0:
        print(f"[ERROR] 서보 모드 시작 실패. 에러 코드: {error}")
        return

    # 사각형 경로: 오른쪽 → 앞 → 왼쪽 → 뒤
    square_path = [
        ([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "오른쪽", 20),  # X+ 10mm
        ([0.0, 0.5, 0.0, 0.0, 0.0, 0.0], "앞", 20),      # Y+ 10mm
        ([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "왼쪽", 20),  # X- 10mm
        ([0.0, -0.5, 0.0, 0.0, 0.0, 0.0], "뒤", 20),    # Y- 10mm
    ]

    try:
        for pose_delta, direction, steps in square_path:
            print(f"[INFO] {direction} 방향으로 이동 중...")

            for i in range(steps):
                error = robot.ServoCart(
                    mode=1,
                    desc_pos=pose_delta,
                    cmdT=0.008
                )

                if error != 0:
                    print(f"[ERROR] ServoCart 실패. 에러 코드: {error}")
                    return

                time.sleep(0.008)

            print(f"  {direction} 이동 완료!")
            time.sleep(0.2)

        print("[SUCCESS] 사각형 그리기 완료!")

    finally:
        print("[INFO] 서보 모드 종료...")
        robot.ServoMoveEnd()

        # 에러가 발생했다면 완전 초기화
        if error != 0:
            print("[WARNING] 에러가 발생했습니다. 로봇 상태를 완전히 초기화합니다.")
            reset_robot_state(robot)


# ==================== 테스트 6: 연속 Pose 명령 테스트 ====================
def test_continuous_pose_commands(robot):
    """연속적인 임의 pose 명령 전송 테스트 (VLA 시뮬레이션)"""
    print("\n" + "="*60)
    print("테스트 6: 연속 Pose 명령 (VLA 시뮬레이션)")
    print("="*60)

    # 혹시 이전에 서보 모드가 남아있을 수 있으니 먼저 종료
    print("[INFO] 이전 서보 모드 정리 중...")
    robot.ServoMoveEnd()
    time.sleep(0.2)

    print("[INFO] 서보 모드 시작...")
    error = robot.ServoMoveStart()
    if error != 0:
        print(f"[ERROR] 서보 모드 시작 실패. 에러 코드: {error}")
        return

    import math

    print("[INFO] 사인파 움직임 테스트 (Z축, 5초간)")

    try:
        start_time = time.time()
        step = 0

        while time.time() - start_time < 5.0:  # 5초간 실행
            # 사인파 패턴 생성 (Z축)
            dz = 0.3 * math.sin(step * 0.1)  # 진폭 0.3mm

            # 임의의 작은 회전도 추가
            drz = 0.1 * math.cos(step * 0.05)

            pose_delta = [0.0, 0.0, dz, 0.0, 0.0, drz]

            error = robot.ServoCart(
                mode=1,
                desc_pos=pose_delta,
                cmdT=0.008
            )

            if error != 0:
                print(f"[ERROR] ServoCart 실패. 에러 코드: {error}")
                break

            step += 1

            if step % 50 == 0:
                elapsed = time.time() - start_time
                print(f"  진행: {elapsed:.1f}초 / 5.0초")

            time.sleep(0.008)

        print("[SUCCESS] 연속 pose 명령 테스트 완료!")
        print(f"  총 {step}개의 명령 전송됨 (평균 {step/5.0:.1f} Hz)")

    finally:
        print("[INFO] 서보 모드 종료...")
        robot.ServoMoveEnd()

        # 에러가 발생했다면 완전 초기화
        if error != 0:
            print("[WARNING] 에러가 발생했습니다. 로봇 상태를 완전히 초기화합니다.")
            reset_robot_state(robot)


# ==================== 메인 함수 ====================
def main():
    print("="*60)
    print("FR5 로봇 Pose 제어 테스트")
    print("="*60)

    # 로봇 연결
    robot = connect_robot(ROBOT_IP)

    # 사용자 선택
    print("\n테스트 메뉴:")
    print("  1. 현재 Pose 읽기")
    print("  2. ServoCart 증분 제어 (Z축 상하)")
    print("  3. ServoCart 회전 제어 (RZ축)")
    print("  4. MoveCart 절대 위치 제어")
    print("  5. 사각형 그리기")
    print("  6. 연속 Pose 명령 (VLA 시뮬레이션)")
    print("  7. 모든 테스트 실행")
    print("  8. 🔧 로봇 상태 초기화")
    print("  0. 종료")

    while True:
        try:
            choice = input("\n테스트 선택 (0-8): ").strip()

            if choice == "0":
                print("[INFO] 프로그램 종료")
                break

            elif choice == "1":
                test_get_current_pose(robot)

            elif choice == "2":
                test_servo_cart_incremental(robot)

            elif choice == "3":
                test_servo_cart_rotation(robot)

            elif choice == "4":
                current_pose = test_get_current_pose(robot)
                if current_pose:
                    test_move_cart_absolute(robot, current_pose)

            elif choice == "5":
                test_draw_square(robot)

            elif choice == "6":
                test_continuous_pose_commands(robot)

            elif choice == "7":
                current_pose = test_get_current_pose(robot)

                input("\n[Enter]를 눌러 테스트 2 시작...")
                test_servo_cart_incremental(robot)

                input("\n[Enter]를 눌러 테스트 3 시작...")
                test_servo_cart_rotation(robot)

                input("\n[Enter]를 눌러 테스트 4 시작...")
                if current_pose:
                    test_move_cart_absolute(robot, current_pose)

                input("\n[Enter]를 눌러 테스트 5 시작...")
                test_draw_square(robot)

                input("\n[Enter]를 눌러 테스트 6 시작...")
                test_continuous_pose_commands(robot)

                print("\n[SUCCESS] 모든 테스트 완료!")

            elif choice == "8":
                print("\n" + "="*60)
                print("🔧 로봇 상태 초기화")
                print("="*60)
                reset_robot_state(robot)
                print("\n[INFO] 이제 다시 테스트를 실행할 수 있습니다.")

            else:
                print("[ERROR] 잘못된 선택입니다. 0-8 사이의 숫자를 입력하세요.")

        except KeyboardInterrupt:
            print("\n[INFO] 사용자에 의해 중단됨")
            break
        except Exception as e:
            print(f"[ERROR] 예외 발생: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
