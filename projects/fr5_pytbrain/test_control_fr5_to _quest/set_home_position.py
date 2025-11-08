#!/usr/bin/env python3
"""
FR5 로봇 홈 포지션 설정 스크립트
- 조인트 각도로 기본 자세 설정
- 현재 위치 저장
"""

import sys
import time
from third_party.fairino_python_sdk.linux.fairino import Robot

ROBOT_IP = "192.168.58.2"

print("=" * 70)
print("🏠 FR5 로봇 홈 포지션 설정")
print("=" * 70)

# 로봇 연결
print("\n🤖 로봇 연결 중...")
robot = Robot.RPC(ROBOT_IP)

error, joints = robot.GetActualJointPosDegree()
if error != 0:
    print(f"❌ 로봇 연결 실패! (error: {error})")
    sys.exit(1)

print("✅ 로봇 연결 성공!")

# 자동 모드로 전환
print("\n⚙️  자동 모드로 전환 중...")
error = robot.Mode(0)
if error != 0:
    print(f"⚠️  모드 전환 실패 (error: {error})")
    print("   티치 펜던트에서 수동으로 자동 모드로 전환해주세요.")

# 로봇 활성화
print("⚙️  로봇 활성화 중...")
error = robot.RobotEnable(1)
if error != 0:
    print(f"⚠️  로봇 활성화 실패 (error: {error})")

# 현재 위치 읽기
print("\n" + "=" * 70)
print("📍 현재 위치")
print("=" * 70)

error, current_joints = robot.GetActualJointPosDegree()
if error == 0:
    print(f"\n[관절 각도 (degree)]")
    for i, angle in enumerate(current_joints[:6]):
        print(f"  J{i+1}: {angle:7.2f}°")
else:
    print(f"❌ 관절 각도 읽기 실패 (error: {error})")
    current_joints = None

error, current_tcp = robot.GetActualTCPPose()
if error == 0:
    print(f"\n[TCP 위치]")
    print(f"  X: {current_tcp[0]:7.2f} mm")
    print(f"  Y: {current_tcp[1]:7.2f} mm")
    print(f"  Z: {current_tcp[2]:7.2f} mm")
    print(f"  RX: {current_tcp[3]:7.2f}°")
    print(f"  RY: {current_tcp[4]:7.2f}°")
    print(f"  RZ: {current_tcp[5]:7.2f}°")
else:
    print(f"❌ TCP 위치 읽기 실패 (error: {error})")

# 미리 정의된 홈 포지션들
home_positions = {
    "1": {
        "name": "Zero Position (모든 관절 0도)",
        "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    "2": {
        "name": "Ready Position (준비 자세)",
        "joints": [0.0, -30.0, 90.0, 0.0, 60.0, 0.0]
    },
    "3": {
        "name": "Home Position (현재 저장된 자세)",
        "joints": [80.45, -61.80, -121.02, -173.51, -6.54, 85.96]
    },
    "4": {
        "name": "Custom (직접 입력)"
    }
}

# 메뉴 출력
print("\n" + "=" * 70)
print("📋 홈 포지션 선택")
print("=" * 70)

for key, pos in home_positions.items():
    if "joints" in pos:
        print(f"\n{key}. {pos['name']}")
        joints_str = ", ".join([f"J{i+1}:{j:.1f}°" for i, j in enumerate(pos['joints'])])
        print(f"   [{joints_str}]")
    else:
        print(f"\n{key}. {pos['name']}")

print("\n0. 현재 위치를 홈 포지션으로 저장")

# 사용자 입력
while True:
    choice = input("\n선택 (0-4): ").strip()

    if choice == "0":
        # 현재 위치 저장
        if current_joints is None:
            print("❌ 현재 관절 각도를 읽을 수 없습니다.")
            continue

        print("\n💾 현재 위치를 홈 포지션으로 저장합니다:")
        print(f"   Joints: {[round(j, 2) for j in current_joints[:6]]}")

        # quest_fr5_teleop_v2.py 파일 업데이트 안내
        print("\n📝 다음 파일의 home_position을 수동으로 업데이트하세요:")
        print("   - /home/stream/vla/quest_fr5_teleop_v2.py")
        print(f"\n   self.home_position = {[round(j, 2) for j in current_joints[:6]]}")

        # TCP 위치도 출력
        if current_tcp:
            print(f"\n   또는 TCP 좌표로:")
            print(f"   self.home_position = {[round(t, 2) for t in current_tcp[:6]]}")

        break

    elif choice in home_positions and "joints" in home_positions[choice]:
        target_joints = home_positions[choice]["joints"]
        name = home_positions[choice]["name"]

        print(f"\n🎯 '{name}' 위치로 이동합니다...")
        print(f"   목표 각도: {[round(j, 2) for j in target_joints]}")

        confirm = input("   계속하시겠습니까? (y/n): ").strip().lower()
        if confirm != 'y':
            print("   취소되었습니다.")
            continue

        # 관절 모드로 이동
        print("\n   이동 중...")
        error = robot.MoveJ(
            joint_pos=target_joints,
            desc_pos=[0, 0, 0, 0, 0, 0],  # 더미
            tool=0,
            user=0,
            vel=20.0,  # 속도 20%
            acc=20.0,  # 가속도 20%
            ovl=100.0,
            blendT=-1.0,  # 블로킹 모드
            offset_flag=0,
            offset_pos=[0, 0, 0, 0, 0, 0]
        )

        if error == 0:
            print("   ✅ 이동 완료!")
            time.sleep(0.5)

            # 실제 도착 위치 확인
            error, actual_joints = robot.GetActualJointPosDegree()
            if error == 0:
                print(f"\n   실제 위치: {[round(j, 2) for j in actual_joints[:6]]}")

            error, actual_tcp = robot.GetActualTCPPose()
            if error == 0:
                print(f"\n   TCP 위치:")
                print(f"   X:{actual_tcp[0]:.2f}, Y:{actual_tcp[1]:.2f}, Z:{actual_tcp[2]:.2f}")
                print(f"   RX:{actual_tcp[3]:.2f}, RY:{actual_tcp[4]:.2f}, RZ:{actual_tcp[5]:.2f}")

            # 코드에 반영할 정보 출력
            print(f"\n   📝 이 위치를 홈으로 사용하려면:")
            print(f"   self.home_position = {[round(j, 2) for j in actual_joints[:6]]}  # Joints")
            print(f"   또는")
            print(f"   self.home_position = {[round(t, 2) for t in actual_tcp[:6]]}  # TCP")

        else:
            print(f"   ❌ 이동 실패 (error: {error})")

        break

    elif choice == "4":
        # 직접 입력
        print("\n✏️  관절 각도를 직접 입력하세요 (6개, 쉼표로 구분):")
        print("   예: 0, -30, 90, 0, 60, 0")

        try:
            input_str = input("   입력: ").strip()
            custom_joints = [float(x.strip()) for x in input_str.split(",")]

            if len(custom_joints) != 6:
                print("   ❌ 6개의 값을 입력해야 합니다.")
                continue

            print(f"\n   목표 각도: {[round(j, 2) for j in custom_joints]}")
            confirm = input("   이 위치로 이동하시겠습니까? (y/n): ").strip().lower()

            if confirm != 'y':
                print("   취소되었습니다.")
                continue

            print("\n   이동 중...")
            error = robot.MoveJ(
                joint_pos=custom_joints,
                desc_pos=[0, 0, 0, 0, 0, 0],
                tool=0,
                user=0,
                vel=20.0,
                acc=20.0,
                ovl=100.0,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0, 0, 0, 0, 0, 0]
            )

            if error == 0:
                print("   ✅ 이동 완료!")
                time.sleep(0.5)

                error, actual_joints = robot.GetActualJointPosDegree()
                error2, actual_tcp = robot.GetActualTCPPose()

                if error == 0 and error2 == 0:
                    print(f"\n   실제 관절: {[round(j, 2) for j in actual_joints[:6]]}")
                    print(f"   TCP 위치:")
                    print(f"   X:{actual_tcp[0]:.2f}, Y:{actual_tcp[1]:.2f}, Z:{actual_tcp[2]:.2f}")
                    print(f"   RX:{actual_tcp[3]:.2f}, RY:{actual_tcp[4]:.2f}, RZ:{actual_tcp[5]:.2f}")

                    print(f"\n   📝 코드에 반영:")
                    print(f"   self.home_position = {[round(j, 2) for j in actual_joints[:6]]}")
            else:
                print(f"   ❌ 이동 실패 (error: {error})")

        except ValueError:
            print("   ❌ 잘못된 입력 형식입니다.")
            continue

        break

    else:
        print("❌ 잘못된 선택입니다. 0-4를 선택하세요.")

print("\n" + "=" * 70)
print("✅ 완료")
print("=" * 70)
