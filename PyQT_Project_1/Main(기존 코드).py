# ==================================================
# Project : Robot Sensor & Motion Control Dashboard
# 작업자 : 김민정
# 작업 기간 : 2026-10-04 ~ 10-08
#
# 사용 OS : Windows
# IDE : VS Code
# 사용 툴 : Qt Designer
# 사용 언어 : Python
# Framework : PyQt5
#
# 목적 :
# 센서 데이터를 기반으로 로봇 상태를 판단하고
# AUTO / MANUAL 모드에 따라 로봇의 이동을 제어하는
# 로봇 제어 대시보드 구현
#
# 주요 기능 :
# 1. 실시간 센서 데이터 표시
# 2. QTimer 기반 센서 데이터 갱신
# 3. 센서 기반 안전 상태 판단
# 4. 안전거리 유지 (AI활용 알고리즘 활용)
# 5. AUTO / MANUAL 모드 (AI활용 알고리즘 활용)
# 6. AUTO 모드 장애물 자동 회피 (AI활용 알고리즘 활용)
# 7. MANUAL 모드 로봇 이동 명령 제어
# 8. 위험 상황 자동 Emergency STOP
# 9. 로봇 상태 및 이벤트 로그 표시
# 10. 로그 파일 저장
# ==================================================

import sys
import subprocess
import random
from datetime import datetime
from pathlib import Path

from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QGraphicsScene, QFileDialog
from PyQt5.QtCore import QTimer, QTime, Qt
from PyQt5.QtGui import QBrush, QPen, QColor, QFont


GUI_FILE_NAME = 'gui'

# Qt Designer에서 만든 최신 gui.ui를 gui.py로 변환
# gui.py를 직접 수정하지 않고 항상 ui 파일을 기준으로 사용한다.
subprocess.run([
    sys.executable,
    '-m', 'PyQt5.uic.pyuic',
    '-x', f'{GUI_FILE_NAME}.ui',
    '-o', f'{GUI_FILE_NAME}.py'
])

from gui import Ui_MainWindow


class Form(QMainWindow, Ui_MainWindow):
    # 로봇 제어 기준값
    SAFE_DISTANCE = 0.3       # 장애물과 유지할 최소 안전거리(m)
    PIXELS_PER_METER = 100    # 1m = 100px
    MOVE_STEP = 15            # 로봇 한 번 이동 거리(px)

    MAP_WIDTH = 550
    MAP_HEIGHT = 250

    BATTERY_WARNING = 25      # 25% 미만 배터리 경고
    BATTERY_RETURN = 20       # 20% 미만 충전소 복귀

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Map 초기화 및 세팅
        self.scene = QGraphicsScene(self)
        self.graphicsViewMap.setScene(self.scene)
        self.scene.setSceneRect(0, 0, self.MAP_WIDTH, self.MAP_HEIGHT)

        # 로봇 생성
        self.robot_item = self.scene.addText('🤖',QFont('Segoe UI Emoji', 20))
        # 로봇 초기 위치 세팅
        self.robot_item.setPos(200, 200)

        # 충전소 생성
        self.charger_item = self.scene.addText('🏠',QFont('Segoe UI Emoji', 20))
        # 충전소 초기 위치 세팅
        self.charger_item.setPos(10, 10)

        # 장애물 생성
        self.obstacle_items = []
        # 장애물 위치 세팅
        obstacle_positions = [
            (200, 50),
            (300, 10),
            (500, 100)
        ]
        # 장애물 세팅
        for x, y in obstacle_positions:
            obstacle = self.scene.addRect(
                0, 0, 30, 30,
                QPen(Qt.red),
                QBrush(QColor('red'))
            )

            obstacle.setPos(x, y)
            self.obstacle_items.append(obstacle)

        # 센서 데이터
        self.sensor = {
            'battery': 82.0,
            'distance': 0.0,
            'motor_temp': 38
        }

        # 로봇 상태
        self.robot_state = {
            'mode': self.comboMode.currentText(),
            'state': 'STOPPED',
            'command': 'STOP',
            'safety': 'NORMAL',
            'reason': 'NORMAL'
        }

        self.battery_warning_shown = False # 배터리 경고창이 여러 번 뜨지 않도록 
        self.auto_danger_shown = False # AUTO 위험 경고창이 계속 반복되지 않도록
        self.charging = False # 충전 여부
        self.returning_to_charger = False # 로봇 충전소로 복귀 여부

        # Signal 연결
        self.btnForward.clicked.connect(lambda: self.manual_move('FORWARD'))
        self.btnBackward.clicked.connect(lambda: self.manual_move('BACKWARD'))
        self.btnLeft.clicked.connect(lambda: self.manual_move('LEFT'))
        self.btnRight.clicked.connect(lambda: self.manual_move('RIGHT'))
        # self.btnStop.clicked.connect(lambda: self.manual_move('STOP'))
        self.comboMode.currentTextChanged.connect(self.change_mode)
        self.btnSaveLogFile.clicked.connect(self.save_log_file)

        # 현재 모드에 맞게 방향 버튼 활성화 / 비활성화
        self.update_manual_buttons()
        # 초기 거리 계산
        self.sensor['distance'] = self.calculate_distance()
        # 화면 표시
        self.update_sensor_display()
        self.update_status()

        # QTimer
        # 0.5초마다 센서와 AUTO 동작 갱신
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_system)
        self.timer.start(500)

        # 프로그램 실행 후 Map 전체가 보이도록 조정
        QTimer.singleShot(0, self.fit_map_view)
        self.add_log('[INFO] Robot Dashboard started')


    # 전체 시스템 반복 실행
    def update_system(self):
        # 충전 중이면 충전 처리만 한다.
        if self.charging:
            self.charge_battery()
            return
        
        if self.returning_to_charger:
            self.return_to_charger_step()
            return

        # 배터리 감소
        self.sensor['battery'] -= 8
        if self.sensor['battery'] < 0:
            self.sensor['battery'] = 0

        # 모터 온도 센서 시뮬레이션
        self.sensor['motor_temp'] = random.randint(30,75)

        # 배터리 상태 확인
        if self.check_battery(): return

        # AUTO 모드에서는 로봇만 자동 이동
        # 장애물은 움직이지 않는다.
        if self.robot_state['mode'] == 'AUTO':
            self.auto_move()

        # 현재 장애물 거리 계산
        self.sensor['distance'] = self.calculate_distance()
        # 안전거리 밖이면 정상 상태
        if self.sensor['distance'] >= self.SAFE_DISTANCE:
            self.robot_state['safety'] = 'NORMAL'
            if self.robot_state['reason'] == 'SAFE DISTANCE':
                self.robot_state['reason'] = 'NORMAL'

        # 화면 갱신
        self.update_sensor_display()
        self.update_status()

    # MANUAL 모드 로봇 이동
    def manual_move(self, command):
        # MANUAL 모드가 아니면 수동 조작하지 않는다.
        if self.robot_state['mode'] != 'MANUAL':
            return
        # 배터리 20% 미만이면 수동 조작 금지
        if self.sensor['battery'] < self.BATTERY_RETURN:
            return
        # STOP 버튼
        if command == 'STOP':
            self.robot_state['command'] = 'STOP'
            self.robot_state['state'] = 'STOPPED'
            self.update_status()
            self.add_log('[CMD] STOP')
            return

        # 현재 로봇 위치
        x = self.robot_item.x()
        y = self.robot_item.y()

        new_x = x
        new_y = y

        # 버튼에 따라 이동 예정 위치 계산
        if command == 'FORWARD':
            new_y -= self.MOVE_STEP
        elif command == 'BACKWARD':
            new_y += self.MOVE_STEP
        elif command == 'LEFT':
            new_x -= self.MOVE_STEP
        elif command == 'RIGHT':
            new_x += self.MOVE_STEP

        # Map 밖으로 나가지 않도록 제한
        new_x, new_y = self.limit_robot_position(new_x,new_y)

        # 이동 예정 위치에서 장애물 거리 계산
        next_distance = self.calculate_distance_at(new_x,new_y)

        # 안전거리 안으로 들어가는 이동은 차단
        if next_distance < self.SAFE_DISTANCE:
            self.robot_state['command'] = 'STOP'
            self.robot_state['state'] = 'EMERGENCY STOP'
            self.robot_state['safety'] = 'DANGER'
            self.robot_state['reason'] = 'SAFE DISTANCE'

            QMessageBox.critical(self, '위험 거리', f'장애물과 {self.SAFE_DISTANCE:.1f}m 이상의 안전거리를 유지해야 합니다.')
            self.add_log(f'[DANGER] {next_distance:.2f}m → movement blocked')
            self.update_status()
            return

        # 안전한 경우에만 실제 로봇 이동
        self.robot_item.setPos(new_x, new_y)

        # 이동 후 실제 거리 다시 계산
        self.sensor['distance'] = self.calculate_distance()
        self.robot_state['command'] = command
        self.robot_state['state'] = 'MOVING'
        self.robot_state['safety'] = 'NORMAL'
        self.robot_state['reason'] = 'NORMAL'

        self.update_sensor_display()
        self.update_status()

        self.add_log(f'[CMD] {command}')


    # AUTO 모드 로봇 이동
    def auto_move(self):
        # 현재 로봇 위치
        x = self.robot_item.x()
        y = self.robot_item.y()

        # 이동 가능한 네 방향
        directions = [
            ('FORWARD', 0, -self.MOVE_STEP),
            ('BACKWARD', 0, self.MOVE_STEP),
            ('LEFT', -self.MOVE_STEP, 0),
            ('RIGHT', self.MOVE_STEP, 0)
        ]

        # 매번 랜덤한 순서로 방향 검사
        random.shuffle(directions)

        # 네 방향을 하나씩 확인
        for command, dx, dy in directions:
            new_x = x + dx
            new_y = y + dy

            # Map 밖으로 나가지 않도록 제한
            new_x, new_y = self.limit_robot_position(new_x, new_y)

            # Map 끝이라 좌표가 바뀌지 않았다면 다음 방향 확인
            if new_x == x and new_y == y:
                continue

            # 이동 예정 위치의 장애물 거리
            next_distance = self.calculate_distance_at(new_x, new_y)

            # 안전거리 안이면 해당 방향 이동 금지
            if next_distance < self.SAFE_DISTANCE:
                self.show_auto_danger()
                continue

            # 안전한 방향이면 실제 이동
            self.robot_item.setPos(new_x, new_y)

            # 이동 후 거리 갱신
            self.sensor['distance'] = self.calculate_distance()

            self.robot_state['command'] = command
            self.robot_state['state'] = 'MOVING'
            self.robot_state['safety'] = 'NORMAL'
            self.robot_state['reason'] = 'NORMAL'

            # 새로운 위험 상황에서 다시 알림을 띄울 수 있도록 초기화
            self.auto_danger_shown = False
            self.add_log(f'[AUTO] {command}')
            return

        # 네 방향 모두 갈 수 없는 경우
        self.robot_state['command'] = 'STOP'
        self.robot_state['state'] = 'EMERGENCY STOP'
        self.robot_state['safety'] = 'DANGER'
        self.robot_state['reason'] = 'NO SAFE DIRECTION'

        self.show_auto_danger()

    
    # AUTO 위험 경고창
    def show_auto_danger(self):
        # 같은 위험 상황에서 경고창이 계속 반복되는 것을 방지
        if self.auto_danger_shown:
            return

        self.auto_danger_shown = True
        self.robot_state['safety'] = 'DANGER'
        self.robot_state['reason'] = 'SAFE DISTANCE'

        QMessageBox.critical(self, '위험 거리', f'장애물이 {self.SAFE_DISTANCE:.1f}m 안전거리 안으로 접근했습니다.\n다른 방향으로 이동합니다.')
        self.add_log('[DANGER] Safe distance detected → change direction')

    # Map 범위 제한
    def limit_robot_position(self, x, y):
        robot_width = self.robot_item.boundingRect().width()
        robot_height = self.robot_item.boundingRect().height()

        x = max(0, min(x, self.MAP_WIDTH - robot_width))
        y = max(0, min(y, self.MAP_HEIGHT - robot_height))

        return x, y


    # 현재 로봇과 가장 가까운 장애물 거리
    def calculate_distance(self):
        robot_rect = self.robot_item.sceneBoundingRect()
        min_distance = float('inf')

        for obstacle in self.obstacle_items:
            obstacle_rect = obstacle.sceneBoundingRect()
            distance = self.rect_distance(robot_rect, obstacle_rect)

            if distance < min_distance:
                min_distance = distance

        return round(min_distance, 2)


    # 이동 예정 위치의 장애물 거리
    def calculate_distance_at(self, x, y):
        # 현재 로봇 사각형
        robot_rect = self.robot_item.sceneBoundingRect()

        # 현재 위치와 이동 예정 위치의 차이
        dx = x - self.robot_item.x()
        dy = y - self.robot_item.y()

        # 실제 로봇을 움직이지 않고
        # 이동 예정 위치의 사각형만 생성
        next_robot_rect = robot_rect.translated(dx, dy)
        min_distance = float('inf')
        
        for obstacle in self.obstacle_items:
            distance = self.rect_distance(next_robot_rect, obstacle.sceneBoundingRect())

            if distance < min_distance:
                min_distance = distance

        return round(min_distance, 2)


    # 두 사각형 사이의 거리 계산
    def rect_distance(self, rect1, rect2):
        # X축 거리
        if rect1.right() < rect2.left():
            dx = rect2.left() - rect1.right()
        elif rect2.right() < rect1.left():
            dx = rect1.left() - rect2.right()
        else:
            dx = 0

        # Y축 거리
        if rect1.bottom() < rect2.top():
            dy = rect2.top() - rect1.bottom()
        elif rect2.bottom() < rect1.top():
            dy = rect1.top() - rect2.bottom()
        else:
            dy = 0

        # 픽셀 거리 계산
        distance_pixels = (dx ** 2 + dy ** 2) ** 0.5
        # pixel → meter
        return distance_pixels / self.PIXELS_PER_METER


    # 배터리 상태 확인
    def check_battery(self):
        battery = self.sensor['battery']

        # 25% 미만이 되면 경고창 한 번 표시
        if battery < self.BATTERY_WARNING:
            if not self.battery_warning_shown:
                self.battery_warning_shown = True
                QMessageBox.warning( self,'배터리 경고','배터리가 25% 미만입니다.\n''20% 미만이 되면 충전소로 이동합니다.')
                self.add_log('[WARN] Battery below 25%')

        # 20% 미만이면 충전소로 이동
        if battery < self.BATTERY_RETURN:
            self.move_to_charger()
            return True
        
        return False


    # 충전소 이동
    def move_to_charger(self):
        # 수동 조작 차단
        self.set_direction_buttons(False)

        self.robot_state['command'] = 'RETURN TO CHARGER'
        self.robot_state['state'] = 'RETURNING'
        self.robot_state['safety'] = 'WARNING'
        self.robot_state['reason'] = 'LOW BATTERY'

        # 충전소 이동 상태 시작(AI 활용 코드 작성)
        self.returning_to_charger = True
        self.charger_detour_direction = None
        self.sensor['distance'] = self.calculate_distance()
        self.add_log('[WARN] Battery below 20% → move to charger')
        self.update_status()


    # 충전
    def charge_battery(self):
        # 충전 중에는 배터리를 조금씩 증가
        self.sensor['battery'] += 2

        if self.sensor['battery'] >= 100:
            self.sensor['battery'] = 100
            self.charging = False
            self.battery_warning_shown = False

            self.robot_state['command'] = 'STOP'
            self.robot_state['state'] = 'STOPPED'
            self.robot_state['safety'] = 'NORMAL'
            self.robot_state['reason'] = 'CHARGE COMPLETE'

            # 충전 완료 알림
            QMessageBox.information(self, '충전 완료', '로봇이 정상적으로 충전되었습니다.')

            # MANUAL 모드라면 충전 완료 후 다시 버튼 활성화
            self.update_manual_buttons()
            self.add_log('[INFO] Battery charging complete')

        self.update_sensor_display()
        self.update_status()
        self.charger_detour_direction = None


    # 충전소로 한 칸씩 이동하는 함수 (AI 활용)
    def return_to_charger_step(self):

        # 현재 로봇 위치
        x = self.robot_item.x()
        y = self.robot_item.y()

        # 충전소 위치
        charger_x = self.charger_item.x()
        charger_y = self.charger_item.y()

        # --------------------------------------------------
        # 1. 충전소 도착
        # --------------------------------------------------
        if x == charger_x and y == charger_y:

            self.returning_to_charger = False
            self.charging = True

            self.robot_state['command'] = 'STOP'
            self.robot_state['state'] = 'CHARGING'
            self.robot_state['safety'] = 'NORMAL'
            self.robot_state['reason'] = 'AT CHARGER'

            self.add_log('[INFO] Arrived at charger')

            self.update_sensor_display()
            self.update_status()
            return

        # --------------------------------------------------
        # 2. 먼저 X축을 충전소와 맞춘다.
        # --------------------------------------------------
        if x != charger_x:

            # 남은 거리와 MOVE_STEP 중 작은 값 사용
            step = min(
                self.MOVE_STEP,
                abs(charger_x - x)
            )

            if x > charger_x:
                new_x = x - step
                command = 'LEFT'

            else:
                new_x = x + step
                command = 'RIGHT'

            new_y = y

        # --------------------------------------------------
        # 3. X축이 맞으면 Y축으로 충전소까지 이동
        # --------------------------------------------------
        else:

            step = min(
                self.MOVE_STEP,
                abs(charger_y - y)
            )

            new_x = x

            if y > charger_y:
                new_y = y - step
                command = 'FORWARD'

            else:
                new_y = y + step
                command = 'BACKWARD'

        # --------------------------------------------------
        # 4. 이동 예정 위치 안전거리 확인
        # --------------------------------------------------
        next_distance = self.calculate_distance_at(
            new_x,
            new_y
        )

        if next_distance < self.SAFE_DISTANCE:

            self.robot_state['command'] = 'STOP'
            self.robot_state['state'] = 'EMERGENCY STOP'
            self.robot_state['safety'] = 'DANGER'
            self.robot_state['reason'] = 'SAFE DISTANCE'

            self.add_log(
                '[DANGER] Charger path blocked'
            )

            self.update_status()
            return

        # --------------------------------------------------
        # 5. 실제 이동
        # --------------------------------------------------
        self.robot_item.setPos(
            new_x,
            new_y
        )

        self.sensor['distance'] = self.calculate_distance()

        self.robot_state['command'] = command
        self.robot_state['state'] = 'RETURNING'
        self.robot_state['safety'] = 'WARNING'
        self.robot_state['reason'] = 'LOW BATTERY'

        self.update_sensor_display()
        self.update_status()


    # AUTO / MANUAL 모드 변경
    def change_mode(self, mode):
        self.robot_state['mode'] = mode
        self.robot_state['command'] = 'STOP'
        self.robot_state['state'] = 'STOPPED'
        self.auto_danger_shown = False

        # AUTO → 방향 버튼 Disabled
        # MANUAL → 방향 버튼 Enabled
        self.update_manual_buttons()
        self.update_status()
        self.add_log(f'[MODE] Changed to {mode}')


    # 방향 버튼 활성화 / 비활성화
    def update_manual_buttons(self):
        # MANUAL이고 배터리가 20% 이상일 때만 조작 가능
        enabled = (self.robot_state['mode'] == 'MANUAL' and self.sensor['battery'] >= self.BATTERY_RETURN and not self.charging)
        self.set_direction_buttons(enabled)
        # STOP 버튼 비활성화
        self.btnStop.setEnabled(False)


    def set_direction_buttons(self, enabled):
        self.btnForward.setEnabled(enabled)
        self.btnBackward.setEnabled(enabled)
        self.btnLeft.setEnabled(enabled)
        self.btnRight.setEnabled(enabled)
        self.btnStop.setEnabled(enabled)


    # 센서 화면 업데이트
    def update_sensor_display(self):
        battery = self.sensor['battery']
        distance = self.sensor['distance']
        temp = self.sensor['motor_temp']

        self.lblBattery.setText(f'{battery:.0f}%')
        self.progressBattery.setValue(int(battery))

        # 배터리가 25% 미만이면 빨간색
        if battery < self.BATTERY_WARNING:
            self.progressBattery.setStyleSheet('QProgressBar::chunk {''background-color: red;''}')
        else:
            self.progressBattery.setStyleSheet('QProgressBar::chunk {''background-color: green;''}')

        self.lblDistance.setText(f'{distance:.2f} m')
        self.lblMotorTemp.setText(f'{temp} °C')


    # 로봇 상태 화면 업데이트
    def update_status(self):
        self.lblState.setText(f"State : {self.robot_state['state']}")
        self.lblCommand.setText(f"Command : {self.robot_state['command']}")
        self.lblSafety.setText(f"Safety : {self.robot_state['safety']}")
        self.lblReason.setText(f"Reason : {self.robot_state['reason']}")
        self.lblObstacleDistance.setText(f"Distance : {self.sensor['distance']:.2f} m")
        self.lblSafeDistance.setText(f"Safe Margin : {self.SAFE_DISTANCE:.2f} m")


    # 로그 기록
    def add_log(self, message):
        time = QTime.currentTime().toString('hh:mm:ss')
        self.textLog.append(f'[{time}] {message}')


    # 로그 파일 저장
    def save_log_file(self):
        # 오늘 날짜를 기본 파일명으로 설정
        today = datetime.now().strftime('%Y-%m-%d')

        # 파일 다이얼로그 -> 파일 경로 str 가지고 옴
        file_path, _ = QFileDialog.getSaveFileName(self, '로그 파일 저장', f'{today}_log.txt', 'Text Files (*.txt);;All Files (*)')

        # 저장 창에서 취소한 경우
        if not file_path:
            return

        # 로그창의 전체 내용 가져오기
        log_text = self.textLog.toPlainText()
        try:
            # 선택한 경로에 UTF-8 텍스트 파일 저장
            with open(file_path,'w', encoding='utf-8') as file:
                file.write(log_text)
            QMessageBox.information(self, '로그 저장 완료', '로그 파일이 정상적으로 저장되었습니다.')
            self.add_log(f'[INFO] Log file saved → {file_path}')
        except Exception as error:
            QMessageBox.warning(self, '로그 저장 실패', f'로그 파일을 저장하지 못했습니다.\n\n{error}')

    # Map 화면 크기 조절
    def fit_map_view(self):
        self.graphicsViewMap.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)


    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'scene'):
            self.fit_map_view()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Form()
    window.show()
    sys.exit(app.exec_())