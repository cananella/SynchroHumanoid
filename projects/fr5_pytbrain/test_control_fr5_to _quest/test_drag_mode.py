#!/usr/bin/env python3
"""
FR5 Drag Teaching Mode Test
손으로 로봇을 조작할 수 있는 모드 테스트
"""

import sys
import time

from sdk_paths import setup_paths
setup_paths()

from third_party.fairino_python_sdk.linux.fairino import Robot

ROBOT_IP = "192.168.58.2"

def main():
    print("=" * 70)
    print("🤖 FR5 드래그 티칭 모드 테스트")
    print("=" * 70)

    # 로봇 연결
    print("\n1. 로봇 연결 중...")
    robot = Robot.RPC(ROBOT_IP)

    error, _ = robot.GetActualJointPosDegree()
    if error != 0:
        print(f"❌ 연결 실패 (error: {error})")
        return

    print("✅ 연결 성공!")

    # 자동 모드
    print("\n2. 자동 모드 설정...")
    error = robot.Mode(0)
    if error == 0:
        print("✅ 자동 모드")

    # 로봇 활성화
    print("\n3. 로봇 활성화...")
    error = robot.RobotEnable(1)
    if error == 0:
        print("✅ 로봇 활성화")

    # 현재 상태 확인
    print("\n4. 현재 상태 확인...")
    error, state = robot.IsInDragTeach()
    if error == 0:
        mode_str = "드래그 모드" if state == 1 else "일반 모드"
        print(f"✅ 현재: {mode_str}")

    # 대화형 모드
    print("\n" + "=" * 70)
    print("🎮 드래그 모드 컨트롤")
    print("=" * 70)
    print("\n명령:")
    print("  [1] - 드래그 모드 활성화 (손으로 조작 가능)")
    print("  [2] - 드래그 모드 비활성화 (로봇 고정)")
    print("  [p] - 현재 위치 출력")
    print("  [q] - 종료")
    print("=" * 70)

    drag_active = False

    try:
        while True:
            cmd = input("\n명령 입력: ").strip()

            if cmd == '1':
                print("\n🎯 드래그 모드 활성화 중...")
                error = robot.DragTeachSwitch(1)
                if error == 0:
                    drag_active = True
                    print("✅ 드래그 모드 활성화!")
                    print("   📌 로봇을 손으로 움직일 수 있습니다.")
                    print("   - 로봇이 현재 자세를 유지합니다")
                    print("   - 손으로 밀면 부드럽게 움직입니다")

                    # 현재 위치 출력
                    error, tcp = robot.GetActualTCPPose()
                    if error == 0:
                        print(f"\n   현재 TCP: [{tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}] mm")
                        print(f"   현재 Rot: [{tcp[3]:.1f}, {tcp[4]:.1f}, {tcp[5]:.1f}]°")
                else:
                    print(f"❌ 활성화 실패 (error: {error})")

            elif cmd == '2':
                print("\n🔒 드래그 모드 비활성화 중...")
                error = robot.DragTeachSwitch(0)
                if error == 0:
                    drag_active = False
                    print("✅ 드래그 모드 비활성화!")
                    print("   🔒 로봇이 고정되었습니다.")

                    # 최종 위치 출력
                    error, tcp = robot.GetActualTCPPose()
                    if error == 0:
                        print(f"\n   최종 TCP: [{tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}] mm")
                        print(f"   최종 Rot: [{tcp[3]:.1f}, {tcp[4]:.1f}, {tcp[5]:.1f}]°")
                else:
                    print(f"❌ 비활성화 실패 (error: {error})")

            elif cmd == 'p':
                error, tcp = robot.GetActualTCPPose()
                if error == 0:
                    print(f"\n📍 현재 위치:")
                    print(f"   TCP: [{tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}] mm")
                    print(f"   Rot: [{tcp[3]:.1f}, {tcp[4]:.1f}, {tcp[5]:.1f}]°")

                    error, joints = robot.GetActualJointPosDegree()
                    if error == 0:
                        print(f"   Joints: {[f'{j:.2f}' for j in joints]}")
                else:
                    print(f"❌ 위치 읽기 실패 (error: {error})")

            elif cmd == 'q':
                print("\n⚠️  종료 중...")
                # 드래그 모드가 활성화되어 있으면 해제
                if drag_active:
                    print("   드래그 모드 해제...")
                    robot.DragTeachSwitch(0)
                print("✅ 종료 완료")
                break

            else:
                print("⚠️  잘못된 명령입니다. [1/2/p/q] 중 하나를 입력하세요.")

    except KeyboardInterrupt:
        print("\n\n⚠️  Ctrl+C 감지")
        if drag_active:
            print("   드래그 모드 해제...")
            robot.DragTeachSwitch(0)
        print("✅ 종료 완료")


if __name__ == '__main__':
    main()
