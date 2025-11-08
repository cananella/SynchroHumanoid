#!/usr/bin/env python3
"""
Quest-FR5 Teleoperation - JSON Format (Right Controller Only)

특징:
- JSON 형식에서 오른쪽 컨트롤러만 파싱
- 비동기 처리: Quest와 FR5가 독립적으로 작동
- 병목 없음: Quest Hz와 무관하게 안정적 동작
- 최신 데이터 사용: 항상 최신 Quest 포즈를 로봇에 전송
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
ROBOT_IP = "192.168.58.2"
QUEST_PORT = 5454
ROBOT_HZ = 125  # FR5 제어 주파수 (높일수록 부드러움)

# 홈 포지션 (조인트 또는 TCP)
HOME_JOINTS = [90.0, -120.0, -85.0, 25.0, 90.0, -0.01]

# 좌표 변환 행렬: Quest → FR5
# Quest: 앞(+Z), 위(+Y), 오른쪽(+X)
# FR5:   앞(+Z), 아래(+Y), 오른쪽(+X)
QUEST_TO_FR5_MATRIX = np.array([
    [1,  0,  0],  # FR5_X = Quest_X
    [0, -1,  0],  # FR5_Y = -Quest_Y (상하 반전)
    [0,  0,  1]   # FR5_Z = Quest_Z
])

# 스케일
SCALE_POSITION = 1000.0   # Quest m → FR5 mm (Quest 10cm = 로봇 40mm)
SCALE_ROTATION = 1.0     # 둘 다 degree

# 안전 제한 (한 프레임당 최대 이동량)
MAX_DELTA_POS = 100.0     # 최대 100mm
MAX_DELTA_ROT = 2.0      # 최대 2도

# 데드존 (Quest 센서 노이즈 제거 - 매우 작게!)
MIN_DELTA_POS = 0.1      # 0.1mm 미만은 무시 (센서 노이즈만 제거)


class QuestFR5TeleopAsync:
    def __init__(self):
        self.running = False

        # 최신 Quest 데이터 (스레드 안전)
        self.quest_data_lock = threading.Lock()
        self.quest_position = None  # [x, y, z] in meters
        self.quest_rotation = None  # [rx, ry, rz] in degrees
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
                data = conn.recv(4096)  # JSON은 더 큰 버퍼 필요
                if not data:
                    print("\n⚠️  Quest 연결 끊김")
                    self.quest_connected = False
                    break

                # JSON 파싱 (오른쪽 컨트롤러만 추출)
                lines = data.decode().strip().split('\n')
                for line in lines:
                    if line:
                        try:
                            # JSON 파싱
                            json_data = json.loads(line)

                            # 오른쪽 컨트롤러 데이터 추출
                            if 'right' in json_data and json_data['right']['enabled']:
                                right = json_data['right']

                                # Position (x, y, z)
                                pos = right['position']
                                quest_pos = np.array([pos['x'], pos['y'], pos['z']])

                                # Euler angles (rx, ry, rz)
                                euler = right['euler']
                                quest_rot = np.array([euler['x'], euler['y'], euler['z']])

                                # 최신 데이터 업데이트 (스레드 안전)
                                with self.quest_data_lock:
                                    self.quest_position = quest_pos  # m
                                    self.quest_rotation = quest_rot  # deg

                        except json.JSONDecodeError:
                            pass  # JSON 파싱 실패 시 무시
                        except KeyError:
                            pass  # right 키가 없거나 구조가 다를 때 무시
                        except:
                            pass

        except Exception as e:
            print(f"⚠️  Quest 수신 오류: {e}")
        finally:
            conn.close()
            server.close()
            self.quest_connected = False

    def transform_quest_to_fr5(self, quest_pos_delta, quest_rot_delta):
        """Quest 좌표를 FR5 좌표로 변환 + 데드존 + 안전 제한"""
        # Position 변환
        fr5_pos = (QUEST_TO_FR5_MATRIX @ quest_pos_delta) * SCALE_POSITION  # m → mm

        # 데드존 + 안전 제한
        pos_magnitude = np.linalg.norm(fr5_pos)
        if pos_magnitude < MIN_DELTA_POS:
            # 센서 노이즈 제거 (0.1mm 미만)
            fr5_pos = np.zeros(3)
        elif pos_magnitude > MAX_DELTA_POS:
            # 안전 제한 (80mm 초과)
            print(f"   [제한] {pos_magnitude:.2f}mm → {MAX_DELTA_POS}mm")
            fr5_pos = fr5_pos * (MAX_DELTA_POS / pos_magnitude)

        # Rotation 변환
        # TODO: Rotation은 나중에 추가 (복잡함)
        fr5_rot = quest_rot_delta * 0.0  # 일단 비활성화

        return np.concatenate([fr5_pos, fr5_rot])

    def calibrate(self, calibration_time=5.0):
        """
        Quest와 동기화 (안정적인 기준점 설정)

        Args:
            calibration_time: 기준점 측정 시간 (초)
        """
        print(f"\n📍 동기화 시작...")
        print(f"   {calibration_time}초 동안 컨트롤러를 가만히 두세요!")

        # Quest 데이터 수집
        quest_positions = []
        quest_rotations = []

        start_time = time.time()
        last_print = start_time

        while time.time() - start_time < calibration_time:
            with self.quest_data_lock:
                if self.quest_position is not None:
                    quest_positions.append(self.quest_position.copy())
                    quest_rotations.append(self.quest_rotation.copy())

            # 진행 상황 표시
            elapsed = time.time() - start_time
            if time.time() - last_print >= 1.0:
                remaining = calibration_time - elapsed
                print(f"   {remaining:.0f}초 남음... (수집된 샘플: {len(quest_positions)})")
                last_print = time.time()

            time.sleep(0.01)

        if len(quest_positions) < 10:
            print(f"❌ 데이터 부족! ({len(quest_positions)} 샘플)")
            return False

        # 평균값 계산 (outlier 제거를 위해 중앙 80% 사용)
        quest_positions = np.array(quest_positions)
        quest_rotations = np.array(quest_rotations)

        # 중앙 80% 선택
        n = len(quest_positions)
        start_idx = int(n * 0.1)
        end_idx = int(n * 0.9)

        quest_pos_median = np.median(quest_positions[start_idx:end_idx], axis=0)
        quest_rot_median = np.median(quest_rotations[start_idx:end_idx], axis=0)

        # 로봇 현재 위치
        error, robot_tcp = self.robot.GetActualTCPPose()
        if error != 0:
            print(f"❌ 로봇 위치 읽기 실패 (error: {error})")
            return False

        # 오프셋 저장
        self.quest_offset_pos = quest_pos_median
        self.quest_offset_rot = quest_rot_median
        self.robot_sync_tcp = np.array(robot_tcp)

        print(f"\n✅ 동기화 완료!")
        print(f"   수집된 샘플: {len(quest_positions)}")
        print(f"   Quest 기준점: [{quest_pos_median[0]:.4f}, {quest_pos_median[1]:.4f}, {quest_pos_median[2]:.4f}]")
        print(f"   Robot TCP:    [{robot_tcp[0]:.1f}, {robot_tcp[1]:.1f}, {robot_tcp[2]:.1f}]")

        # 변동성 체크
        pos_std = np.std(quest_positions[start_idx:end_idx], axis=0)
        print(f"   Quest 변동성: [{pos_std[0]*1000:.2f}, {pos_std[1]*1000:.2f}, {pos_std[2]*1000:.2f}] mm")

        if np.max(pos_std) > 0.01:  # 10mm 이상 흔들림
            print(f"   ⚠️  컨트롤러가 많이 흔들렸습니다. 다시 시도하세요.")

        self.calibrated = True

        # 증분 계산을 위한 초기화
        with self.quest_data_lock:
            if self.quest_position is not None:
                self.previous_quest_pos = self.quest_position.copy()
                self.previous_quest_rot = self.quest_rotation.copy()

        return True

    def robot_control_thread(self):
        """로봇 제어 스레드 (일정 주기)"""
        print(f"\n🔄 로봇 제어 시작 ({ROBOT_HZ} Hz)")

        dt = 1.0 / ROBOT_HZ
        error_count = 0

        # 서보 모드 시작
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

                # 최신 Quest 데이터 가져오기 (스레드 안전)
                with self.quest_data_lock:
                    if self.quest_position is None:
                        time.sleep(dt)
                        continue

                    quest_pos = self.quest_position.copy()
                    quest_rot = self.quest_rotation.copy()

                # 이전 프레임 대비 증분 계산 (프레임간 차이!)
                if self.previous_quest_pos is not None:
                    quest_delta_pos = quest_pos - self.previous_quest_pos
                    quest_delta_rot = quest_rot - self.previous_quest_rot
                else:
                    # 첫 프레임: 움직임 없음
                    quest_delta_pos = np.zeros(3)
                    quest_delta_rot = np.zeros(3)

                # 현재 프레임 저장 (다음 프레임을 위해)
                self.previous_quest_pos = quest_pos.copy()
                self.previous_quest_rot = quest_rot.copy()

                # FR5 좌표로 변환
                fr5_delta = self.transform_quest_to_fr5(quest_delta_pos, quest_delta_rot)

                # 로봇으로 전송 (증분 모드)
                # vel 계산: cmdT가 0.008 (125Hz)이므로 vel=10~20 정도가 적당
                error = self.robot.ServoCart(
                    mode=2,  # TCP 좌표계
                    desc_pos=fr5_delta.tolist(),
                    vel=15,  # vel=15 (125Hz 기준 최적값, 부드럽게!)
                    cmdT=dt
                )

                if error != 0 and error != 10009:
                    error_count += 1
                    if error_count % 10 == 0:
                        print(f"⚠️  ServoCart 에러 {error} (누적: {error_count})")

                # 주기 유지
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
        print("🎮 Quest-FR5 Teleoperation (JSON - Right Controller)")
        print("=" * 70)
        print("\n📖 사용법:")
        print("   1. Quest 앱 실행 → 자동 연결 대기")
        print("   2. [h] - 홈 포지션으로 이동")
        print("   3. [c] - 현재 위치에서 동기화 (Quest ↔ FR5)")
        print("   4. 동기화 후 자동으로 텔레오퍼레이션 시작")
        print("   5. [q] - 종료")
        print("=" * 70 + "\n")

        self.running = True

        # 스레드 시작
        quest_thread = threading.Thread(target=self.quest_receiver_thread, daemon=True)
        quest_thread.start()

        robot_thread = threading.Thread(target=self.robot_control_thread, daemon=True)
        robot_thread.start()

        # 메인 루프 (사용자 입력)
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
                            print("🎬 텔레오퍼레이션 시작!")
                        else:
                            print("❌ 동기화 실패")

                else:
                    print("⚠️  잘못된 명령")

        except KeyboardInterrupt:
            print("\n⚠️  Ctrl+C 감지")
            self.running = False

        # 스레드 종료 대기
        print("\n🧹 정리 중...")
        quest_thread.join(timeout=2.0)
        robot_thread.join(timeout=2.0)

        print("✅ 종료 완료")


def main():
    teleop = QuestFR5TeleopAsync()
    teleop.run()


if __name__ == '__main__':
    main()
