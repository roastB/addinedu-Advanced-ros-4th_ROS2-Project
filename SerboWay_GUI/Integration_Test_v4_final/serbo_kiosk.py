# ===================== 표준 라이브러리 (Python Built-in Modules) =====================
import sys # 시스템 관련 기능(프로그램 종료, 경로 조작 등)
import os # 운영체제 인터페이스(파일 경로, 환경변수 등)
import json # JSON 데이터 처리
import random # 난수 생성(영수증 번호 생성용)
import string # 문자열 유틸리티(영수증 번호 생성용)
import subprocess # 외부 프로세스 실행(Streamlit 서버 실행)
from datetime import datetime # 시간 관련 기능(주문 타임스탬프)
from typing import Optional, Dict, Any, List # 타입 힌트
import time # 시간 지연 처리
import struct

# ============= 통신 모듈 ============
import requests # HTTP 요청 처리(메인 서버 API 호출)
import webbrowser # [Streamlit 연동 추가] 웹브라우저 띄우기
import threading # 멀티스레딩 처리
import socket # 네트워크 통신(음성 주문-키오스크 간 TCP 통신)
import serial # 시리얼 통신 (RFID 아두이노와 통신용)
import signal # 신호 처리(프로세스 제어)

# ============== PyQt 모듈 ==================
from PyQt5.QtWidgets import (
    QApplication, # PyQt 애플리케이션 메인 클래스
    QMainWindow, # 메인 윈도우 클래스
    QStackedWidget, # 여러 페이지를 스택으로 관리하는 위젯
    QPushButton, # 버튼 위젯
    QListWidget, # 리스트 위젯
    QMessageBox, # 메시지 박스 (경고, 정보 표시용)
    QLabel, # 텍스트/이미지 표시용 라벨
    QTextEdit, # 텍스트 편집 위젯
    QLineEdit, # 한 줄 텍스트 입력 위젯
)
from PyQt5 import uic # UI 파일 로드용
from PyQt5.QtGui import QIcon # 아이콘 처리
from PyQt5.QtCore import Qt, QSize # Qt 코어 기능
from PyQt5.QtCore import QLoggingCategory # 로깅 카테고리
from PyQt5.QtCore import QEvent # 이벤트 처리

# 음성 주문 이벤트 클래스 정의
class VoiceOrderEvent(QEvent):
    def __init__(self):
        super().__init__(QEvent.User) # 사용자 정의 이벤트 타입

# ============== 메인 서버 설정 ================
# ORDER_SERVER_URL = "http://192.168.0.6:5003/" # 주문 전송 API 주소
# ORDER_SERVER_URL = "http://192.168.55.177:5003" # 대체 서버 주소
# ORDER_SERVER_URL = " http://192.168.0.136:5003" # 대체 서버 주소
ORDER_SERVER_URL = "http://192.168.0.145:5004"    # SR


# ============ Streamlit 설정 =============
STREAMLIT_PORT = 8501 # Streamlit 서버 포트
STREAMLIT_SCRIPT = "Serboway_voice_order.py" # 음성 에이전트 스크립트 경로

# Table Number 고정
TABLE_NUM = 1 # 테이블 번호 (고정값)

# ========= 메인 서버와 통신 ============
def send_order_to_server(order_data):
    """주문 정보를 메인 서버로 전송하는 함수"""
    try:
        response = requests.post(ORDER_SERVER_URL, json=order_data) # POST 요청으로 주문 데이터 전송
        response.raise_for_status() # HTTP 오류 발생 시 예외 발생
        return response.json() # JSON 응답 반환
    except Exception as e:
        print(f"주문 서버 연결 실패: {e}") # 오류 로그 출력
        return {"status": "fail", "message": str(e)} # 실패 응답 반환

def get_menu_json(server_url=ORDER_SERVER_URL, local_file="menu.json"):
    """메뉴 JSON을 가져오는 함수 (메인 서버→로컬 파일→기본값 순서로 시도)"""
    # 1. 메인 서버에서 가져오기 시도
    try:
        print(f"메인 서버({server_url})에서 메뉴 데이터 가져오기 시도...")
        response = requests.get(server_url, timeout=5) # 5초 타임아웃으로 GET 요청
        if response.status_code == 200: # 성공 응답인 경우
            menu_data = response.json() # JSON 데이터 파싱
            if menu_data and "menu" in menu_data: # 유효한 메뉴 데이터인지 확인
                print("메인 서버에서 메뉴 데이터를 성공적으로 불러왔습니다.")
                # 성공 시 로컬에도 저장해둠 (백업)
                try:
                    with open(local_file, "w", encoding="utf-8") as f:
                        json.dump(menu_data, f, ensure_ascii=False, indent=2) # 로컬 파일에 백업 저장
                except Exception as e:
                    print(f"로컬 저장 실패: {e}")
                return menu_data # 메뉴 데이터 반환
    except Exception as e:
        print(f"서버 연결 실패: {e}")

    # 2. 로컬 파일에서 가져오기 시도
    try:
        print(f"로컬 파일({local_file})에서 메뉴 데이터 가져오기 시도...")
        with open(local_file, "r", encoding="utf-8") as f:
            menu_data = json.load(f) # 로컬 JSON 파일 로드
        if menu_data and "menu" in menu_data: # 유효한 데이터인지 확인
            print("로컬 파일에서 메뉴 데이터를 성공적으로 불러왔습니다.")
            return menu_data
    except Exception as e:
        print(f"로컬 파일 불러오기 실패: {e}")

    # 3. 기본값 반환
    print("메뉴 데이터를 불러오지 못했습니다. 기본 구조를 사용합니다.")
    return {"menu": {}, "sauce": {}, "vegetable": {}, "cheese": {}} # 빈 메뉴 구조 반환

# RFID 컨트롤러 클래스 정의
# test_kiosk.py의 RFIDController 클래스 수정
import struct

class RFIDController:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        """RFID 컨트롤러 초기화"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            self.uid = bytes(4)  # UID 저장용
            print("RFID 컨트롤러 연결 성공")
        except Exception as e:
            print(f"RFID 컨트롤러 연결 실패: {e}")
            self.ser = None

    def send_command(self, command, data=0):
        """rfid_test.py와 동일한 명령 전송 방식"""
        if not self.ser:
            return None
        
        req_data = struct.pack('<2s4sic', command, self.uid, data, b'\n')
        self.ser.write(req_data)
        return self.ser.read_until(b'\n')

    def detect_card(self):
        """카드 감지"""
        response = self.send_command(b'GS')
        if response and len(response) > 6:
            response = response[:-2]  # 마지막 \r\n 제거
            cmd = response[:2].decode()
            if cmd == 'GS' and response[2] == 0:
                self.uid = response[3:]
                return True
        return False

    def get_card_balance(self):
        """카드 잔액 조회 (UID 파라미터 제거)"""
        if not self.detect_card():
            return None
        
        response = self.send_command(b'GT')
        if response and len(response) > 6:
            response = response[:-2]
            cmd = response[:2].decode()
            if cmd == 'GT' and response[2] == 0:
                balance = int.from_bytes(response[3:7], 'little')
                return balance
        return None

    def set_balance(self, amount):
        """카드 잔액 설정 (charge_card 대신)"""
        response = self.send_command(b'ST', amount)
        if response and len(response) >= 3:
            response = response[:-2]
            return len(response) >= 3 and response[2] == 0
        return False

    def deduct_amount(self, amount):
        """카드에서 금액 차감 (UID 파라미터 제거)"""
        current_balance = self.get_card_balance()
        if current_balance is None:
            return False, "카드 읽기 실패"
        
        if current_balance < amount:
            return False, "잔액 부족"
        
        new_balance = current_balance - amount
        success = self.set_balance(new_balance)
        
        if success:
            return True, f"결제 완료. 잔액: {new_balance}원"
        else:
            return False, "결제 처리 실패"

# 메인 키오스크 애플리케이션 클래스
class SerbowayApp(QMainWindow):
    """메인 키오스크 애플리케이션"""
    
    def __init__(self):
        super().__init__() # 부모 클래스 초기화
        self.setWindowTitle("Serboway Kiosk") # 윈도우 제목 설정
        self.setGeometry(200, 200, 600, 500) # 윈도우 위치와 크기 설정
        
        # 음성 주문 관련 변수 초기화
        self.voice_order_data = None # 음성 주문 데이터 저장용
        
        # TCP 서버 스레드 시작 (음성 주문 수신용)
        self.tcp_server_thread = threading.Thread(
            target=self.run_tcp_server, daemon=True
        )
        self.tcp_server_thread.start() # 백그라운드에서 TCP 서버 실행

        # JSON 메뉴 로드
        self.menu_json = get_menu_json() # 메뉴 데이터 로드
        self.order_data = {"menu": []} # 주문 데이터 초기화
        
        # 현재 선택사항 초기화
        self.current_sandwich = None # 선택된 샌드위치
        self.selected_sauce = None # 선택된 소스
        self.selected_vegetable = None # 선택된 야채
        self.selected_cheese = None # 선택된 치즈

        # RFID 컨트롤러 초기화
        self.rfid_controller = RFIDController() # RFID 컨트롤러 객체 생성
        self.total_order_amount = 0 # 총 주문 금액
        self.CHARGE_AMOUNT = 500000 # 초기 충전 금액 3만원으로 설정

        # [Streamlit 연동 추가] Streamlit 프로세스 핸들 저장
        self.streamlit_proc = None # Streamlit 프로세스 객체

        # 스택 위젯 설정 (여러 페이지 관리)
        self.stack = QStackedWidget() # 스택 위젯 생성
        self.setCentralWidget(self.stack) # 중앙 위젯으로 설정

        # UI 페이지 로드 (각 UI 파일을 로드하여 페이지 생성)
        self.page0 = uic.loadUi("kiosk_UI/1_choose_ordermethod.ui") # 주문 방식 선택
        self.page1 = uic.loadUi("kiosk_UI/2_choose_sandwich.ui") # 샌드위치 선택
        self.page2 = uic.loadUi("kiosk_UI/3_choose_sauce.ui") # 소스 선택
        self.page3 = uic.loadUi("kiosk_UI/4_choose_vegetables.ui") # 야채 선택
        self.page4 = uic.loadUi("kiosk_UI/5_choose_cheese.ui") # 치즈 선택
        self.page5 = uic.loadUi("kiosk_UI/6_confirm_order.ui") # 주문 확인
        self.page6 = uic.loadUi("kiosk_UI/7_choose_paymentmethod.ui") # 결제 방식 선택
        self.page7 = uic.loadUi("kiosk_UI/8_order_complete.ui") # 주문 완료
        self.page8 = uic.loadUi("kiosk_UI/9_pick_up.ui") # 픽업 페이지
        self.page9 = uic.loadUi("kiosk_UI/10_request_collect.ui") # 수거 요청 페이지
        self.page10 = uic.loadUi("kiosk_UI/11_collect_done.ui") # 수거 완료 페이지
        self.page11 = uic.loadUi("kiosk_UI/12_rfid_charge_x.ui") # RFID 페이지

        # 모든 페이지를 스택에 추가
        for page in [
            self.page0, self.page1, self.page2, self.page3, self.page4,
            self.page5, self.page6, self.page7, self.page8, self.page9,
            self.page10, self.page11,
        ]:
            self.stack.addWidget(page) # 스택 위젯에 페이지 추가

        # 버튼 연결 및 동적 매핑 설정
        self.connect_buttons() # 버튼 이벤트 연결
        self.populate_dynamic_buttons() # 동적 버튼 설정

    def connect_buttons(self):
        """버튼 이벤트 연결 함수"""
        def btn(page, name):
            """페이지에서 버튼을 찾는 헬퍼 함수"""
            button = page.findChild(
                QPushButton, name, Qt.FindChildrenRecursively
            ) # 재귀적으로 버튼 찾기
            if not button: # 버튼을 찾지 못했으면
                print(f"⚠️ 버튼 누락: {name} 버튼을 찾을 수 없습니다.") # 경고 메시지 출력
            return button # 버튼 객체 반환 (없으면 None)

        # ========== 첫 페이지: 주문 방식 선택 버튼 ==========
        voice_btn = btn(self.page0, "voiceBtn") # 음성 주문 버튼 찾기
        touch_btn = btn(self.page0, "touchBtn") # 터치 주문 버튼 찾기
        
        if voice_btn: # 음성 주문 버튼이 존재하면
            voice_btn.clicked.connect(
                self.launch_streamlit_voice_order
            ) # Streamlit 음성 주문 실행 함수와 연결
        
        if touch_btn: # 터치 주문 버튼이 존재하면
            touch_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(1)
            ) # 샌드위치 선택 페이지로 이동

        # ========== 주문 확인 페이지 버튼들 ==========
        add_btn = btn(self.page5, "addorderBtn") # 추가 주문 버튼
        restart_btn = btn(self.page5, "restartBtn") # 주문 재시작 버튼
        pay_btn = btn(self.page5, "payBtn") # 결제 버튼
        
        if add_btn: # 추가 주문 버튼이 존재하면
            add_btn.clicked.connect(self.add_order) # 추가 주문 함수와 연결
        
        if restart_btn: # 재시작 버튼이 존재하면
            restart_btn.clicked.connect(self.restart_order) # 주문 재시작 함수와 연결
        
        if pay_btn: # 결제 버튼이 존재하면
            pay_btn.clicked.connect(self.go_to_payment) # 결제 페이지 이동 함수와 연결

        # ========== 결제 페이지 버튼 ==========
        rfid_btn = btn(self.page6, "rfidBtn") # RFID 결제 버튼
        if rfid_btn: # RFID 버튼이 존재하면
            rfid_btn.clicked.connect(self.show_rfid_page) # RFID 페이지 표시 함수와 연결

        # ========== RFID 페이지 버튼들 연결 ==========
        reset_btn = self.page11.findChild(QPushButton, "resetButton", Qt.FindChildrenRecursively) # 충전 버튼
        payment_btn = self.page11.findChild(QPushButton, "paymentButton", Qt.FindChildrenRecursively) # 결제 버튼
        
        if reset_btn: # 충전 버튼이 존재하면
            reset_btn.clicked.connect(self.reset_rfid_card) # 카드 충전 함수와 연결
        
        if payment_btn: # 결제 버튼이 존재하면
            payment_btn.clicked.connect(self.process_rfid_payment) # RFID 결제 처리 함수와 연결

        # ========== 나머지 페이지 버튼들 ==========
        # 8_order_complete.ui - OK 버튼
        ok_btn = btn(self.page7, "okBtn")
        if ok_btn:
            ok_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(8)
            ) # 픽업 페이지로 이동

        # 9_pick_up.ui - Pick Up Done 버튼
        pickup_btn = btn(self.page8, "pickupBtn")
        if pickup_btn:
            pickup_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(9)
            ) # 수거 요청 페이지로 이동

        # 10_request_collect.ui - Request Collect 버튼
        reqcol_btn = btn(self.page9, "reqcolBtn")
        if reqcol_btn:
            reqcol_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(10)
            ) # 수거 완료 페이지로 이동

        # 11_collect_done.ui - Collect Done 버튼
        coldone_btn = btn(self.page10, "colDone")
        if coldone_btn:
            coldone_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(0)
            ) # 메인 페이지로 돌아가기

    def calculate_total_amount(self):
        """현재 주문의 총 금액을 계산하는 함수"""
        total = 0 # 총 금액 초기화
        for item in self.order_data["menu"]: # 주문 데이터의 각 메뉴 아이템에 대해
            total += item["price"] * item["qty"] # (가격 × 수량)을 총 금액에 추가
        return total # 총 금액 반환

    def show_rfid_page(self):
        """RFID 페이지로 이동하면서 주문 금액 표시"""
        # 총 금액 계산
        total_amount = self.calculate_total_amount() # 현재 주문의 총 금액 계산
        
        # totalLabel에 주문 금액 표시 (오타 수정: findChild)
        total_label = self.page11.findChild(QLabel, "totalLabel", Qt.FindChildrenRecursively)
        if total_label: # 라벨이 존재하면
            total_label.setText(f"주문 금액: {total_amount:,}원") # 주문 금액 텍스트 설정
        
        # paymentEdit에 주문 금액 미리 입력
        payment_edit = self.page11.findChild(QLineEdit, "paymentEdit", Qt.FindChildrenRecursively)
        if payment_edit: # 입력 필드가 존재하면
            payment_edit.setText(str(total_amount)) # 주문 금액을 입력 필드에 설정
        
        # RFID 페이지로 이동
        self.stack.setCurrentIndex(11) # 스택 위젯의 11번 페이지(RFID 페이지)로 이동

    def process_rfid_payment(self):
        """RFID 카드 결제 처리 함수"""
        if not self.rfid_controller.ser:
            QMessageBox.warning(self, "오류", "RFID 리더기가 연결되지 않았습니다.")
            return

        total_amount = self.calculate_total_amount()
        if total_amount <= 0:
            QMessageBox.warning(self, "오류", "주문 금액이 없습니다.")
            return

        QMessageBox.information(self, "카드 결제", "결제할 카드를 리더기에 올려주세요.")

        try:
            # 카드 감지 및 결제 처리
            success, message = self.rfid_controller.deduct_amount(total_amount)
            
            if success:
                QMessageBox.information(self, "결제 완료", message)
                self.complete_order()
            else:
                if "잔액 부족" in message:
                    reply = QMessageBox.question(
                        self, "잔액 부족",
                        f"{message}\n충전하시겠습니까?",
                        QMessageBox.Yes | QMessageBox.No
                    )
                    if reply == QMessageBox.Yes:
                        self.reset_rfid_card()
                else:
                    QMessageBox.warning(self, "결제 실패", message)

        except Exception as e:
            QMessageBox.warning(self, "오류", f"결제 처리 중 오류가 발생했습니다: {e}")

    def reset_rfid_card(self):
        """RFID 카드 충전 함수"""
        if not self.rfid_controller.ser:
            QMessageBox.warning(self, "오류", "RFID 리더기가 연결되지 않았습니다.")
            return

        QMessageBox.information(self, "카드 충전", "카드를 리더기에 올려주세요.")

        try:
            if self.rfid_controller.detect_card():
                if self.rfid_controller.set_balance(self.CHARGE_AMOUNT):
                    QMessageBox.information(self, "충전 완료", f"{self.CHARGE_AMOUNT:,}원이 충전되었습니다.")
                else:
                    QMessageBox.warning(self, "충전 실패", "카드 충전에 실패했습니다.")
            else:
                QMessageBox.warning(self, "오류", "카드를 인식할 수 없습니다.")

        except Exception as e:
            QMessageBox.warning(self, "오류", f"충전 처리 중 오류가 발생했습니다: {e}")

    # [Streamlit 연동 추가] 음성 주문 버튼 클릭 시 Streamlit 서버 실행 및 브라우저 접속
    def launch_streamlit_voice_order(self):
        """음성 주문 버튼 클릭 시 Streamlit 서버를 실행하고 브라우저로 접속"""
        # Streamlit 서버가 이미 실행 중인지 확인
        if self.streamlit_proc is None or self.streamlit_proc.poll() is not None:
            # Streamlit 서버 실행 (subprocess)
            self.streamlit_proc = subprocess.Popen(
                [
                    "streamlit", # Streamlit 명령어
                    "run", # run 서브명령어
                    STREAMLIT_SCRIPT, # 실행할 스크립트 파일
                    "--server.headless=true", # 헤드리스 모드 (GUI 없음)
                    f"--server.port={STREAMLIT_PORT}", # 포트 설정
                ],
                stdout=subprocess.DEVNULL, # 표준 출력 무시
                stderr=subprocess.DEVNULL, # 표준 오류 무시
            )
        
        # 웹브라우저로 Streamlit UI 오픈
        webbrowser.open(f"http://localhost:{STREAMLIT_PORT}") # 기본 브라우저에서 Streamlit 페이지 열기

    def populate_dynamic_buttons(self):
        """동적 버튼 설정 함수 (메뉴 데이터에 따라 버튼 텍스트와 이벤트 설정)"""
        # ========== 샌드위치 버튼 매핑 ==========
        sandwich_map = {
            # UI 파일의 실제 버튼 객체명과 메뉴 키, 이미지 경로 매핑
            "BulgogiBtn": ("불고기 샌드위치", "image/Bulgogi141.png"),
            "ShrimpBtn": ("새우 샌드위치", "image/Shrimp141.png"),
            "BaconBtn": ("베이컨 샌드위치", "image/Bacon141.png"),
        }

        for obj_name, (menu_key, img_path) in sandwich_map.items():
            btn = self.page1.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            ) # UI에서 버튼 찾기
            
            if not btn or menu_key not in self.menu_json.get("menu", {}): # 버튼이 없거나 메뉴가 JSON에 없으면 스킵
                continue

            # 배경 이미지 제거 + 텍스트 정렬
            btn.setStyleSheet(
                """
                QPushButton {
                    background-image: none; /* UI 파일의 기존 배경 이미지 제거 */
                    text-align: center; /* 텍스트 중앙 정렬 */
                }
                """
            )

            # 아이콘 설정 (이미지 파일 존재 여부 확인 후 설정)
            if os.path.exists(img_path): # 이미지 파일이 존재하면
                btn.setIcon(QIcon(img_path)) # 해당 이미지를 아이콘으로 설정
            else: # 이미지 파일이 없으면
                btn.setIcon(
                    QIcon(self.menu_json["menu"][menu_key].get("image", img_path))
                ) # JSON에서 이미지 경로 가져오기
            
            btn.setIconSize(QSize(128, 128)) # 아이콘 크기를 128x128로 설정

            # 클릭 이벤트 연결 (기존 연결 해제 후 새로 연결)
            try:
                btn.clicked.disconnect() # 기존에 연결된 시그널 해제
            except TypeError: # 연결된 시그널이 없으면 TypeError 발생하므로 무시
                pass
            
            btn.clicked.connect(
                lambda _, m=menu_key: self.select_sandwich(m)
            ) # 샌드위치 선택 함수와 연결

        # ========== 소스 버튼 매핑 ==========
        sauce_map = {
            "ItalianBtn": "이탈리안", # UI 버튼명과 소스명 매핑
            "ChillyBtn": "칠리",
        }

        for obj_name, sauce_key in sauce_map.items():
            btn = self.page2.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            ) # 소스 페이지에서 버튼 찾기
            
            if not btn or sauce_key not in self.menu_json.get("sauce", {}): # 버튼이 없거나 소스가 JSON에 없으면 스킵
                continue

            price = self.menu_json["sauce"][sauce_key]["price"] # JSON에서 소스 가격 가져오기
            btn.setText(f"{sauce_key}\n(+{price}원)") # 버튼 텍스트를 "소스명\n(+가격원)" 형태로 설정
            
            try:
                btn.clicked.disconnect() # 기존 연결 해제
            except TypeError:
                pass
            
            btn.clicked.connect(
                lambda _, s=sauce_key: self.select_sauce(s)
            ) # 소스 선택 함수와 연결

        # ========== 야채 버튼 매핑 ==========
        veg_map = {
            "LettuceBtn": "양상추", # UI 버튼명과 야채명 매핑
            "RomaineBtn": "로메인",
            "BazilBtn": "바질",
        }

        for obj_name, veg_key in veg_map.items():
            btn = self.page3.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            ) # 야채 페이지에서 버튼 찾기
            
            if btn and veg_key in self.menu_json.get("vegetable", {}): # 버튼이 있고 야채가 JSON에 있으면
                price = self.menu_json["vegetable"][veg_key].get("price", 0) # JSON에서 야채 가격 가져오기 (없으면 0)
                btn.setText(f"{veg_key}\n(+{price}원)") # 버튼 텍스트 설정
                
                try:
                    btn.clicked.disconnect() # 기존 연결 해제
                except TypeError:
                    pass
                
                btn.clicked.connect(
                    lambda _, v=veg_key: self.select_vegetable(v)
                ) # 야채 선택 함수와 연결

        # ========== 치즈 버튼 매핑 ==========
        cheese_map = {
            "SliceBtn": "슬라이스 치즈", # UI 버튼명과 치즈명 매핑
            "ShredBtn": "슈레드 치즈",
            "MozzarellaBtn": "모짜렐라 치즈",
        }

        for obj_name, cheese_key in cheese_map.items():
            btn = self.page4.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            ) # 치즈 페이지에서 버튼 찾기
            
            if btn and cheese_key in self.menu_json.get("cheese", {}): # 버튼이 있고 치즈가 JSON에 있으면
                price = self.menu_json["cheese"][cheese_key].get("price", 0) # JSON에서 치즈 가격 가져오기 (없으면 0)
                btn.setText(f"{cheese_key}\n(+{price}원)") # 버튼 텍스트 설정
                
                try:
                    btn.clicked.disconnect() # 기존 연결 해제
                except TypeError:
                    pass
                
                btn.clicked.connect(
                    lambda _, c=cheese_key: self.select_cheese(c)
                ) # 치즈 선택 함수와 연결

        # ========== 주문 리스트 위젯 참조 ==========
        # QTextEdit 위젯을 찾아서 주문 목록 표시용으로 사용
        self.order_list_widget = self.page5.findChild(
            QTextEdit, "orderList", Qt.FindChildrenRecursively
        )

    def select_sandwich(self, name):
        """샌드위치 선택 함수"""
        self.current_sandwich = name # 선택된 샌드위치 저장
        self.stack.setCurrentIndex(2) # 소스 선택 페이지로 이동

    def select_sauce(self, sauce):
        """소스 선택 함수"""
        self.selected_sauce = sauce # 선택된 소스 저장
        self.stack.setCurrentIndex(3) # 야채 선택 페이지로 이동

    def select_vegetable(self, veg):
        """야채 선택 함수"""
        self.selected_vegetable = veg # 선택된 야채 저장
        self.stack.setCurrentIndex(4) # 치즈 선택 페이지로 이동

    def select_cheese(self, cheese):
        """치즈 선택 함수"""
        self.selected_cheese = cheese # 선택된 치즈 저장
        self.save_order_item() # 주문 아이템 저장
        self.stack.setCurrentIndex(5) # 주문 확인 페이지로 이동
        self.update_order_list() # 주문 목록 업데이트

    def save_order_item(self): 
        """사용자가 선택한 샌드위치, 소스, 야채, 치즈를 주문 목록에 추가하는 함수"""
        base_price = self.menu_json["menu"][self.current_sandwich]["price"] # 기본 샌드위치 가격
        opt_price = (
            self.menu_json["sauce"][self.selected_sauce].get("price", 0) # 소스 가격
            + self.menu_json["vegetable"][self.selected_vegetable].get("price", 0) # 야채 가격
            + self.menu_json["cheese"][self.selected_cheese].get("price", 0) # 치즈 가격
        )
        unit_price = base_price + opt_price # 총 단가 계산

        # 주문 데이터에 아이템 추가
        self.order_data["menu"].append(
            {
                "name": self.current_sandwich, # 샌드위치 이름
                "price": unit_price, # 총 가격
                "qty": 1, # 수량 (기본 1개)
                "sauce": self.selected_sauce, # 선택된 소스
                "vegetable": self.selected_vegetable, # 선택된 야채
                "cheese": self.selected_cheese, # 선택된 치즈
            }
        )

        # 서버 전송용 주문 데이터 생성
        self.send_order_data = {
            "table_number": TABLE_NUM, # 테이블 번호
            "sandwich": self.current_sandwich, # 샌드위치
            "sauce": self.selected_sauce, # 소스
            "vegetable": self.selected_vegetable, # 야채
            "cheese": self.selected_cheese, # 치즈
            "price": unit_price, # 가격
        }

    def update_order_list(self):
        """주문 내역을 UI에 표시하고 총 금액을 계산해서 보여주는 함수"""
        if hasattr(self, "order_list_widget") and self.order_list_widget: # order_list_widget이 존재하면
            self.order_list_widget.clear() # 기존 목록 지우기
            total = 0 # 총 금액 초기화
            
            for item in self.order_data["menu"]: # 주문 데이터의 각 메뉴 아이템에 대해
                # 주문 아이템 텍스트 생성 (메뉴명, 옵션들, 수량, 가격)
                text = (
                    f"{item['name']} ({item['sauce']}/{item['vegetable']}/{item['cheese']}) "
                    f"x{item['qty']} - {item['price']}원"
                )
                self.order_list_widget.append(text) # QTextEdit에 텍스트 추가
                total += item["price"] * item["qty"] # 총 금액에 (가격 × 수량) 추가

            # 총 합계 라벨 찾아서 업데이트
            lbl = self.page5.findChild(
                QLabel, "summaryLabel", Qt.FindChildrenRecursively
            ) # 총합계 라벨 찾기
            if lbl: # 라벨이 존재하면
                lbl.setText(f"총 합계: {total}원") # 총 합계 텍스트 설정

    def go_to_payment(self):
        """결제 화면으로 이동하는 함수"""
        if not self.order_data["menu"]: # 주문 내역이 없으면
            QMessageBox.warning(self, "경고", "주문 내역이 없습니다.") # 경고 메시지 표시
            return
        self.stack.setCurrentIndex(6) # 결제 방식 선택 페이지로 이동

    def complete_order(self):
        """주문 완료 처리 함수"""
        print("최종 주문:", self.order_data) # 디버깅용 출력
        
        # 타임스탬프 추가
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S") # 현재 시간을 문자열로 변환
        
        # 주문 데이터에 타임스탬프 추가
        order_with_time = self.order_data.copy() # 주문 데이터 복사
        order_with_time["timestamp"] = timestamp # 타임스탬프 추가

        # JSON 파일로 저장
        order_filename = f"order_{timestamp}.json" # 파일명 생성
        with open(order_filename, "w", encoding="utf-8") as f:
            json.dump(order_with_time, f, ensure_ascii=False, indent=4) # JSON 파일로 저장
        print(f"주문 내역이 {order_filename}에 저장되었습니다.")

        # 서버로 주문 데이터 전송
        result = send_order_to_server(self.send_order_data) # 서버에 주문 전송
        if result.get("status") == "fail": # 서버 전송 실패
            QMessageBox.warning(self, "서버 오류", "주문 저장 중 오류가 발생했습니다.")
        
        self.stack.setCurrentIndex(7) # 주문 완료 페이지로 이동

    def restart_order(self):
        """주문 재시작 함수"""
        self.order_data = {"menu": []} # 주문 데이터 초기화
        self.stack.setCurrentIndex(1) # 샌드위치 선택 페이지로 이동

    def add_order(self):
        """추가 주문을 위해 샌드위치 선택 페이지로 이동하는 함수"""
        # 현재 선택사항 초기화 (새로운 주문을 위해)
        self.current_sandwich = None # 선택된 샌드위치 초기화
        self.selected_sauce = None # 선택된 소스 초기화
        self.selected_vegetable = None # 선택된 야채 초기화
        self.selected_cheese = None # 선택된 치즈 초기화
        
        # 샌드위치 선택 페이지(page1)로 이동
        self.stack.setCurrentIndex(1) # 스택 위젯의 인덱스 1번 페이지로 이동

    def show_webview(self):
        """웹뷰 표시 함수 (현재 미사용)"""
        self.stack.addWidget(self.webview) # 웹뷰를 스택에 추가
        self.stack.setCurrentWidget(self.webview) # 웹뷰를 현재 위젯으로 설정

    def handle_payment_result(self, result):
        """결제 결과 처리 함수"""
        if result.get("status") == "paid": # 결제 완료 상태인 경우
            self.save_order() # 주문 저장
            self.show_confirmation() # 확인 페이지 표시

    def save_order(self):
        """주문 데이터 저장 함수"""
        try:
            order_data = {
                "orders": self.server.current_order, # 현재 주문 데이터
                "timestamp": datetime.now().isoformat(), # ISO 형식 타임스탬프
                "receipt": "".join(random.choices(string.digits, k=8)), # 8자리 영수증 번호
            }

            filename = f"order_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json" # 파일명 생성
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(order_data, f, ensure_ascii=False, indent=2) # JSON 파일로 저장
            print(f"주문 데이터 로컬 저장 완료: {filename}")
        except Exception as e:
            print(f"주문 저장 오류: {e}")

    def show_confirmation(self):
        """확인 페이지 표시 함수"""
        QMessageBox.information(
            self, "주문 완료", "음성 주문이 정상적으로 처리되었습니다!"
        ) # 주문 완료 메시지 표시

    def closeEvent(self, event):
        """창 종료 시 리소스 정리 함수"""
        try:
            # [Streamlit 연동 추가] Streamlit 프로세스 종료
            if self.streamlit_proc: # Streamlit 프로세스가 실행 중이면
                self.streamlit_proc.terminate() # 프로세스 종료
            
            # TCP 서버 종료
            if hasattr(self, "server") and self.server.socket: # TCP 서버가 실행 중이면
                self.server.running = False # 실행 상태를 False로 설정
                self.server.socket.close() # 소켓 닫기
        except Exception as e:
            print(f"종료 처리 오류: {e}")
        
        super().closeEvent(event) # 부모 클래스의 closeEvent 호출

    def run_tcp_server(self):
        """TCP 서버 실행 함수 (음성 주문 수신용)"""
        # KIOSK_HOST = "192.168.0.55" # 키오스크 호스트 IP (주석 처리됨)
        # KIOSK_HOST = "192.168.55.177" # 현재 사용 중인 키오스크 호스트 IP
        KIOSK_HOST = "192.168.0.159"
        # KIOSK_PORT = 650 # 포트 (주석 처리됨)
        KIOSK_PORT = 5050 # 현재 사용 중인 포트

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: # TCP 소켓 생성
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 소켓 재사용 옵션 설정
            s.bind((KIOSK_HOST, KIOSK_PORT)) # 호스트와 포트에 바인딩
            s.listen(1) # 연결 대기 (최대 1개 연결)
            
            while True: # 무한 루프로 연결 대기
                conn, addr = s.accept() # 클라이언트 연결 수락
                with conn: # 연결 컨텍스트 관리
                    data = conn.recv(4096) # 4096바이트까지 데이터 수신
                    if data: # 데이터가 있으면
                        try:
                            order_json = json.loads(data.decode()) # JSON 데이터 파싱
                            self.voice_order_data = order_json # 음성 주문 데이터 저장
                            conn.sendall(b"received") # 수신 확인 응답
                            
                            # 주문 UI 반영을 위한 이벤트 발송
                            QApplication.postEvent(self, VoiceOrderEvent()) # 사용자 정의 이벤트 발송
                        except Exception: # JSON 파싱 오류 등
                            conn.sendall(b"fail") # 실패 응답

    def process_voice_order(self):
        """음성 주문 처리 함수"""
        # 디버깅 확인 용
        print("음성 주문 처리 시작")
        print("수신된 주문:", self.voice_order_data)

        if self.voice_order_data: # 음성 주문 데이터가 있으면
            order = self.voice_order_data # 주문 데이터 참조
            print(2) # 디버깅용 출력
            
            self.order_data = {"menu": []} # 주문 데이터 초기화
            
            # 음성 주문 데이터를 키오스크 주문 형식으로 변환
            for item in order.get("menu", []): # 주문의 각 메뉴 아이템에 대해
                self.order_data["menu"].append(
                    {
                        "name": item.get("name"), # 메뉴 이름
                        "price": item.get("price"), # 가격
                        "qty": item.get("qty", 1), # 수량 (기본 1)
                        "sauce": item.get("sauce"), # 소스
                        "vegetable": item.get("vegetable"), # 야채
                        "cheese": item.get("cheese"), # 치즈
                    }
                )
            
            self.update_order_list() # 주문 목록 UI 업데이트
            self.stack.setCurrentIndex(6) # 결제 페이지로 이동
            
            # 사용자에게 음성 주문 도착 알림
            QMessageBox.information(
                self, "음성 주문", "음성 주문이 도착했습니다. 결제를 진행해주세요."
            )

    def event(self, event):
        """이벤트 처리 함수 (사용자 정의 이벤트 처리)"""
        if event.type() == QEvent.User: # 사용자 정의 이벤트인 경우
            print("VoiceOrderEvent 수신됨") # 디버깅용 출력
            self.process_voice_order() # 음성 주문 처리
            return True # 이벤트 처리 완료
        return super().event(event) # 기본 이벤트 처리

# 메인 실행 부분
if __name__ == "__main__":
    app = QApplication(sys.argv) # PyQt 애플리케이션 생성
    window = SerbowayApp() # 키오스크 애플리케이션 인스턴스 생성
    window.show() # 윈도우 표시
    sys.exit(app.exec_()) # 애플리케이션 실행 및 종료 시 시스템 종료

# PyQt 로깅 설정 (디버그 메시지 숨김)
QLoggingCategory.setFilterRules(
    "*.debug=false\n*.info=false\n*.warning=false\n*.critical=true"
) # 내부 디버그 메시지 숨김
