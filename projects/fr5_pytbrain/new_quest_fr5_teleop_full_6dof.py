#!/usr/bin/env python3
"""
Quest-FR5 Teleoperation - FULL 6-DOF Control

특징:
- Position: Full 3-DOF (X, Y, Z) ✅
- Rotation: Full 3-DOF (RX, RY, RZ) ✅
- 개별 축 테스트 결과를 종합한 완전체
"""

import socket
import threading
import numpy as np
import time
import sys
import json

from sdk_paths import setup_paths
setup_paths()
from third_party.fairino_python_sdk.linux.fairino import Robot

# ==================== 설정 ====================
ROBOT_IP = "192.168.57.2"
QUEST_PORT = 5454
ROBOT_HZ = 72  # FR5 제어 주파수 (Quest tracking HZ와 동기화)

# 홈 포지션
HOME_JOINTS = [-136.0, -128.0, -90.0, 40.0, 90.00, 0.0]

# 좌표 변환 행렬: Quest → FR5 (Position)
QUEST_TO_FR5_MATRIX = np.array([
    [1,  0,  0],  # FR5_X = Quest_X
    [0, -1,  0],  # FR5_Y = -Quest_Y (상하 반전)
    [0,  0,  1]   # FR5_Z = Quest_Z
])

# 스케일
SCALE_POSITION = 1000.0   # Quest m → FR5 mm
SCALE_ROTATION = 1.0      # 1:1 매칭

# 안전 제한
MAX_DELTA_POS = 100.0     # 최대 100mm
MAX_DELTA_ROT = 5.0       # 최대 5도

# 데드존
MIN_DELTA_POS = 0.2       # 0.1mm 미만 무시
MIN_DELTA_ROT = 0.2       # 0.1도 미만 무시


class QuestFR5TeleopAsync:
    def __init__(self):
        self.running = False

        # 최신 Quest 데이터 (스레드 안전)
        self.quest_data_lock = threading.Lock()
        self.quest_position = None  # [x, y, z] in meters
        self.quest_rotation = None  # [rx, ry, rz] in degrees
        self.quest_timestamp = None  # Quest timestamp (중복 제거용)
        self.quest_connected = False

        # 동기화 상태
        self.calibrated = False
        self.quest_offset_pos = None
        self.quest_offset_rot = None
        self.robot_sync_tcp = None

        # 이전 프레임 (증분 계산용)
        self.previous_quest_pos = None
        self.previous_quest_rot = None

        # 로봇 연결
        self.connect_robot()

    def connect_robot(self):
        """로봇 연결 및 초기화"""
        print("=" * 70)
        print("🤖 FR5 로봇 연결 중...")
        print("=" * 70)

        self.robot = Robot.RPC(ROBOT_IP)
        error, _ = self.robot.GetActualJointPosDegree()
        if error != 0:
            print(f"❌ 로봇 연결 실패 (error: {error})")
            sys.exit(1)

        print("✅ 로봇 연결 성공!")

        # 자동 모드
        error = self.robot.Mode(0)
        if error == 0:
            print("✅ 자동 모드")
        else:
            print(f"⚠️  자동 모드 실패 (error: {error}) - 티치 펜던트에서 수동 전환 필요")

        # 로봇 활성화
        error = self.robot.RobotEnable(1)
        if error == 0:
            print("✅ 로봇 활성화")

    def quest_receiver_thread(self):
        """Quest 데이터 수신 스레드 (비동기) - JSON 형식"""
        print("\n🥽 Quest 수신 서버 시작...")
        print(f"   포트: {QUEST_PORT}")

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', QUEST_PORT))
        server.listen(1)

        print("   대기 중...")

        conn, addr = server.accept()
        print(f"✅ Quest 연결됨: {addr}\n")

        self.quest_connected = True

        try:
            while self.running:
                data = conn.recv(4096)
                if not data:
                    print("\n⚠️  Quest 연결 끊김")
                    self.quest_connected = False
                    break

                # JSON 파싱 (오른쪽 컨트롤러만 추출)
                lines = data.decode().strip().split('\n')
                for line in lines:
                    if line:
                        try:
                            json_data = json.loads(line)

                            # Timestamp 추출
                            timestamp = json_data.get('timestamp', None)

                            if 'right' in json_data and json_data['right']['enabled']:
                                right = json_data['right']

                                # Position (x, y, z)
                                pos = right['position']
                                quest_pos = np.array([pos['x'], pos['y'], pos['z']])

                                # Euler angles (rx, ry, rz)
                                euler = right['euler']
                                quest_rot = np.array([euler['x'], euler['y'], euler['z']])

                                # 최신 데이터 업데이트 (timestamp 중복 체크)
                                with self.quest_data_lock:
                                    # 중복 제거: timestamp가 변경되었을 때만 업데이트
                                    if timestamp is None or timestamp != self.quest_timestamp:
                                        self.quest_position = quest_pos
                                        self.quest_rotation = quest_rot
                                        self.quest_timestamp = timestamp

                        except:
                            pass

        except Exception as e:
            print(f"⚠️  Quest 수신 오류: {e}")
        finally:
            conn.close()
            server.close()
            self.quest_connected = False

    def normalize_angle_delta(self, angle_delta):
        """각도 증분을 -180~180 범위로 정규화"""
        return np.array([(delta + 180) % 360 - 180 for delta in angle_delta])

    def transform_quest_to_fr5(self, quest_pos_delta, quest_rot_delta):
        """Quest 좌표를 FR5 좌표로 변환 + 데드존 + 안전 제한"""

        # Position 변환 (행렬 변환)
        fr5_pos = (QUEST_TO_FR5_MATRIX @ quest_pos_delta) * SCALE_POSITION

        # Position 데드존 + 안전 제한
        pos_magnitude = np.linalg.norm(fr5_pos)
        if pos_magnitude < MIN_DELTA_POS:
            fr5_pos = np.zeros(3)
        elif pos_magnitude > MAX_DELTA_POS:
            print(f"   [위치 제한] {pos_magnitude:.2f}mm → {MAX_DELTA_POS}mm")
            fr5_pos = fr5_pos * (MAX_DELTA_POS / pos_magnitude)

        # Rotation 변환 (개별 축 테스트 결과 적용)
        # RX: 반전, RY: 그대로, RZ: 반전
        fr5_rot = np.array([
            -quest_rot_delta[0],  # RX 반전 (테스트 확인)
            quest_rot_delta[1],   # RY 그대로 (테스트 확인)
            -quest_rot_delta[2]   # RZ 반전 (테스트 확인)
        ]) * SCALE_ROTATION

        # Rotation 데드존 체크 (전체 magnitude)
        rot_magnitude = np.linalg.norm(fr5_rot)
        if rot_magnitude < MIN_DELTA_ROT:
            fr5_rot = np.zeros(3)
        else:
            # 각 축 개별 제한 (가장 중요!)
            fr5_rot[0] = np.clip(fr5_rot[0], -MAX_DELTA_ROT, MAX_DELTA_ROT)  # RX
            fr5_rot[1] = np.clip(fr5_rot[1], -MAX_DELTA_ROT, MAX_DELTA_ROT)  # RY
            fr5_rot[2] = np.clip(fr5_rot[2], -MAX_DELTA_ROT, MAX_DELTA_ROT)  # RZ

            # 전체 magnitude 최종 안전 체크 (여유 있게)
            rot_magnitude = np.linalg.norm(fr5_rot)
            max_combined = MAX_DELTA_ROT * 1.5  # 5° * 1.5 = 7.5°
            if rot_magnitude > max_combined:
                print(f"   [회전 제한] {rot_magnitude:.2f}° → {max_combined:.2f}°")
                fr5_rot = fr5_rot * (max_combined / rot_magnitude)

        return np.concatenate([fr5_pos, fr5_rot])

    def calibrate(self, calibration_time=5.0):
        """Quest와 동기화"""
        print(f"\n📍 동기화 시작...")
        print(f"   {calibration_time}초 동안 컨트롤러를 가만히 두세요!")

        quest_positions = []
        quest_rotations = []

        start_time = time.time()
        last_print = start_time

        while time.time() - start_time < calibration_time:
            with self.quest_data_lock:
                if self.quest_position is not None:
                    quest_positions.append(self.quest_position.copy())
                    quest_rotations.append(self.quest_rotation.copy())

            elapsed = time.time() - start_time
            if time.time() - last_print >= 1.0:
                remaining = calibration_time - elapsed
                print(f"   {remaining:.0f}초 남음... (수집된 샘플: {len(quest_positions)})")
                last_print = time.time()

            time.sleep(0.01)

        if len(quest_positions) < 10:
            print(f"❌ 데이터 부족! ({len(quest_positions)} 샘플)")
            return False

        quest_positions = np.array(quest_positions)
        quest_rotations = np.array(quest_rotations)

        n = len(quest_positions)
        start_idx = int(n * 0.1)
        end_idx = int(n * 0.9)

        quest_pos_median = np.median(quest_positions[start_idx:end_idx], axis=0)
        quest_rot_median = np.median(quest_rotations[start_idx:end_idx], axis=0)

        error, robot_tcp = self.robot.GetActualTCPPose()
        if error != 0:
            print(f"❌ 로봇 위치 읽기 실패 (error: {error})")
            return False

        self.quest_offset_pos = quest_pos_median
        self.quest_offset_rot = quest_rot_median
        self.robot_sync_tcp = np.array(robot_tcp)

        print(f"\n✅ 동기화 완료!")
        print(f"   수집된 샘플: {len(quest_positions)}")
        print(f"   Quest 위치: [{quest_pos_median[0]:.4f}, {quest_pos_median[1]:.4f}, {quest_pos_median[2]:.4f}]")
        print(f"   Quest 회전: [{quest_rot_median[0]:.1f}, {quest_rot_median[1]:.1f}, {quest_rot_median[2]:.1f}]°")
        print(f"   Robot TCP:  [{robot_tcp[0]:.1f}, {robot_tcp[1]:.1f}, {robot_tcp[2]:.1f}]")
        print(f"   Robot Rot:  [{robot_tcp[3]:.1f}, {robot_tcp[4]:.1f}, {robot_tcp[5]:.1f}]°")

        pos_std = np.std(quest_positions[start_idx:end_idx], axis=0)
        rot_std = np.std(quest_rotations[start_idx:end_idx], axis=0)
        print(f"   위치 변동성: [{pos_std[0]*1000:.2f}, {pos_std[1]*1000:.2f}, {pos_std[2]*1000:.2f}] mm")
        print(f"   회전 변동성: [{rot_std[0]:.2f}, {rot_std[1]:.2f}, {rot_std[2]:.2f}]°")

        if np.max(pos_std) > 0.01:
            print(f"   ⚠️  컨트롤러가 많이 흔들렸습니다.")

        self.calibrated = True

        with self.quest_data_lock:
            if self.quest_position is not None:
                self.previous_quest_pos = self.quest_position.copy()
                self.previous_quest_rot = self.quest_rotation.copy()

        return True

    def robot_control_thread(self):
        """로봇 제어 스레드"""
        print(f"\n🔄 로봇 제어 시작 ({ROBOT_HZ} Hz)")
        print("   ✅ FULL 6-DOF 제어 활성화!\n")

        dt = 1.0 / ROBOT_HZ
        error_count = 0

        error = self.robot.ServoMoveStart()
        if error != 0:
            print(f"❌ 서보 모드 시작 실패 (error: {error})")
            return

        print("✅ 서보 모드 시작\n")

        try:
            while self.running:
                loop_start = time.time()

                if not self.calibrated:
                    time.sleep(dt)
                    continue

                with self.quest_data_lock:
                    if self.quest_position is None:
                        time.sleep(dt)
                        continue

                    quest_pos = self.quest_position.copy()
                    quest_rot = self.quest_rotation.copy()

                if self.previous_quest_pos is not None:
                    quest_delta_pos = quest_pos - self.previous_quest_pos
                    quest_delta_rot = self.normalize_angle_delta(quest_rot - self.previous_quest_rot)
                else:
                    quest_delta_pos = np.zeros(3)
                    quest_delta_rot = np.zeros(3)

                self.previous_quest_pos = quest_pos.copy()
                self.previous_quest_rot = quest_rot.copy()

                fr5_delta = self.transform_quest_to_fr5(quest_delta_pos, quest_delta_rot)

                error = self.robot.ServoCart(
                    mode=2,  # TCP 좌표계
                    desc_pos=fr5_delta.tolist(),
                    vel=15,
                    cmdT=dt
                )

                if error != 0 and error != 10009:
                    error_count += 1
                    if error_count % 10 == 0:
                        print(f"⚠️  ServoCart 에러 {error} (누적: {error_count})")

                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            print(f"❌ 로봇 제어 오류: {e}")
        finally:
            self.robot.ServoMoveEnd()
            print("\n✅ 서보 모드 종료")

    def move_to_home(self):
        """홈 포지션으로 이동"""
        print("\n🏠 홈 포지션으로 이동 중...")

        error = self.robot.MoveJ(
            joint_pos=HOME_JOINTS,
            desc_pos=[0, 0, 0, 0, 0, 0],
            tool=0,
            user=0,
            vel=30.0,
            acc=30.0,
            ovl=100.0,
            blendT=-1.0,
            offset_flag=0,
            offset_pos=[0, 0, 0, 0, 0, 0]
        )

        if error == 0:
            print("✅ 홈 포지션 도착!")
            time.sleep(0.5)
        else:
            print(f"❌ 이동 실패 (error: {error})")

    def run(self):
        """메인 루프"""
        print("\n" + "=" * 70)
        print("🎮 Quest-FR5 Teleoperation (FULL 6-DOF)")
        print("=" * 70)
        print("\n📖 사용법:")
        print("   1. Quest 앱 실행 → 자동 연결 대기")
        print("   2. [h] - 홈 포지션으로 이동")
        print("   3. [c] - 현재 위치에서 동기화")
        print("   4. 동기화 후 텔레오퍼레이션 시작")
        print("   5. [q] - 종료")
        print("\n✅ Position: Full 3-DOF (X, Y, Z)")
        print("✅ Rotation: Full 3-DOF (RX, RY, RZ)")
        print("   컨트롤러의 모든 움직임이 로봇에 반영됩니다.")
        print("\n📊 변환 규칙:")
        print("   Position: [X, -Y, Z] * 1000 (m→mm)")
        print("   Rotation: [-RX, RY, -RZ] * 1.0 (degree)")
        print("=" * 70 + "\n")

        self.running = True

        quest_thread = threading.Thread(target=self.quest_receiver_thread, daemon=True)
        quest_thread.start()

        robot_thread = threading.Thread(target=self.robot_control_thread, daemon=True)
        robot_thread.start()

        try:
            while self.running:
                cmd = input("명령 [h:홈, c:동기화, q:종료]: ").strip().lower()

                if cmd == 'q':
                    print("\n⚠️  종료 중...")
                    self.running = False
                    break

                elif cmd == 'h':
                    if not self.calibrated:
                        self.move_to_home()
                    else:
                        print("⚠️  이미 동기화됨. 종료 후 다시 시작하세요.")

                elif cmd == 'c':
                    if not self.quest_connected:
                        print("⚠️  Quest가 연결되지 않았습니다.")
                    elif self.calibrated:
                        print("⚠️  이미 동기화됨.")
                    else:
                        if self.calibrate():
                            print("🎬 텔레오퍼레이션 시작! (FULL 6-DOF)")
                        else:
                            print("❌ 동기화 실패")

                else:
                    print("⚠️  잘못된 명령")

        except KeyboardInterrupt:
            print("\n⚠️  Ctrl+C 감지")
            self.running = False

        print("\n🧹 정리 중...")
        quest_thread.join(timeout=2.0)
        robot_thread.join(timeout=2.0)

        print("✅ 종료 완료")


def main():
    teleop = QuestFR5TeleopAsync()
    teleop.run()


if __name__ == '__main__':
    main()
