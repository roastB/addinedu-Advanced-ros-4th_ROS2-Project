# 시스템 및 프로세스 관리
import sys
import os
import atexit
import subprocess
import time

# PyQt5 GUI 위젯
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QStackedWidget,
    QSpinBox, QListWidget, QMessageBox
)

# PyQt5 핵심 기능 (스레드/신호 처리 포함)
from PyQt5.QtCore import Qt, QThread, pyqtSignal

# PyQt5 웹 엔진 (Streamlit 연동)
from PyQt5 import QtWebEngineWidgets, QtCore

# PyQt5 이미지 처리
from PyQt5.QtGui import QImage, QPixmap

# 컴퓨터 비전 및 통신
import cv2
import websockets
from pyzbar.pyzbar import decode


# HTTP 요청을 위한 모듈을 가져옵니다
import requests
# JSON 처리를 위한 모듈을 가져옵니다
import json

# ========= 환영 페이지 ============
class WelcomePage(QWidget):
    def __init__(self, stack):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()
        
        # 화면 전환을 위한 스택 위젯 참조를 저장합니다
        self.stack = stack

        # 환영 메시지 레이블을 생성합니다
        label = QLabel("Welcome to SerboWay!")
        # 레이블을 중앙 정렬합니다
        label.setAlignment(Qt.AlignCenter)
        # 레이블의 글꼴 크기와 두께를 설정합니다
        label.setStyleSheet("font-size: 48px; font-weight: bold")

        # 음성 주문 버튼을 생성합니다
        voice_btn = QPushButton("🎙 Voice Order")
        # 버튼의 글꼴 크기와 두께를 설정합니다
        voice_btn.setStyleSheet("font-size: 24px; font-weight: bold")

        # 터치 주문 버튼을 생성합니다
        touch_btn = QPushButton("🖐 Touch Order")
        # 버튼의 글꼴 크기와 두께를 설정합니다
        touch_btn.setStyleSheet("font-size: 24px; font-weight: bold")

        # 음성 주문 버튼 클릭 시 스택 인덱스 1(음성 주문 페이지)로 이동하도록 설정합니다
        voice_btn.clicked.connect(lambda: stack.setCurrentIndex(1))
        # 터치 주문 버튼 클릭 시 스택 인덱스 2(터치 주문 페이지)로 이동하도록 설정합니다
        touch_btn.clicked.connect(lambda: stack.setCurrentIndex(2))

        # 수직 레이아웃을 생성합니다
        layout = QVBoxLayout()
        # 레이아웃에 환영 메시지 레이블을 추가합니다
        layout.addWidget(label)
        # 레이아웃에 음성 주문 버튼을 추가합니다
        layout.addWidget(voice_btn)
        # 레이아웃에 터치 주문 버튼을 추가합니다
        layout.addWidget(touch_btn)
        # 위젯에 레이아웃을 설정합니다
        self.setLayout(layout)

# ============== 샌드위치 선택 페이지 ==============
class SandwichPage(QWidget):
    def __init__(self, stack, order_data):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()
        
        # 화면 전환을 위한 스택 위젯 참조를 저장합니다
        self.stack = stack
        # 주문 데이터를 저장할 딕셔너리 참조를 저장합니다
        self.order_data = order_data

        # 페이지 제목 레이블을 생성합니다
        label = QLabel("Sandwiches")
        # 레이블의 글꼴 크기와 두께를 설정합니다
        label.setStyleSheet("font-size: 48px; font-weight: bold")

        # 메뉴 아이템 목록을 생성합니다 (이름, 가격)
        menu_items = [("Bulgogi", 6500), ("Shrimp", 6200), ("Bacon", 6000)]
        # 메뉴 버튼들의 정보를 저장할 리스트를 초기화합니다
        self.menu_buttons = [] # 메뉴 (이름, 가격, 수량스핀박스) 목록

        # 수직 레이아웃을 생성합니다
        layout = QVBoxLayout()
        # 레이아웃에 제목 레이블을 추가합니다
        layout.addWidget(label)

        # 각 메뉴 아이템에 대한 UI 요소를 생성합니다
        for name, price in menu_items:
            # 각 메뉴 아이템을 위한 수평 레이아웃을 생성합니다
            h_layout = QHBoxLayout()
            
            # 메뉴 이름과 가격이 표시된 버튼을 생성합니다
            btn = QPushButton(f"{name} ({price}원)")
            # 버튼의 글꼴 크기를 설정합니다
            btn.setStyleSheet("font-size: 24px")

            # 수량 선택용 스핀박스를 생성합니다
            spin = QSpinBox()
            # 스핀박스의 값 범위를 0~10으로 설정합니다
            spin.setRange(0, 10)
            # 스핀박스의 글꼴 크기를 설정합니다
            spin.setStyleSheet("font-size: 24px")

            # 수평 레이아웃에 버튼을 추가합니다
            h_layout.addWidget(btn)
            # 수평 레이아웃에 스핀박스를 추가합니다
            h_layout.addWidget(spin)
            # 메인 수직 레이아웃에 수평 레이아웃을 추가합니다
            layout.addLayout(h_layout)

            # 메뉴 버튼 정보(이름, 가격, 스핀박스)를 리스트에 저장합니다
            self.menu_buttons.append((name, price, spin))

        # '장바구니' 버튼을 생성합니다
        cart_btn = QPushButton("장바구니")
        # 버튼의 글꼴 크기를 설정합니다
        cart_btn.setStyleSheet("font-size: 24px")
        # 버튼 클릭 시 go_next 메서드를 호출하도록 연결합니다
        cart_btn.clicked.connect(self.go_next)
        # 버튼을 레이아웃에 추가합니다
        layout.addWidget(cart_btn)
        # 위젯에 레이아웃을 설정합니다
        self.setLayout(layout)

    ## 삭제 예정이라는 주석이 있지만 현재 사용 중인 메서드
    def go_next(self):
        """선택된 메뉴와 수량을 order_data에 저장하고 다음 페이지로 이동"""
        # 각 메뉴 버튼 정보를 확인합니다
        for name, price, spin in self.menu_buttons:
            # 메뉴의 선택 수량을 가져옵니다
            qty = spin.value()
            # 수량이 1개 이상 선택된 경우에만 주문 데이터에 저장합니다
            if qty > 0: # 수량이 1개 이상 선택된 경우만 저장
                # 주문 데이터에 선택된 메뉴 정보를 저장합니다
                self.order_data['menu'] = {'name': name, 'price': price, 'qty': qty}

        # 다음 페이지(소스 선택 페이지)로 이동합니다
        self.stack.setCurrentIndex(3)


# ============= 옵션 선택 페이지 ==============
class OptionPage(QWidget):
    def __init__(self, stack, order_data, title, options, next_index):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()
        
        # 화면 전환을 위한 스택 위젯 참조를 저장합니다
        self.stack = stack
        # 주문 데이터를 저장할 딕셔너리 참조를 저장합니다
        self.order_data = order_data
        # 페이지 제목(예: "Sauce", "Vegetables", "Cheese")을 저장합니다
        self.title = title
        # 옵션 목록(이름, 가격)을 저장합니다
        self.options = options
        # 다음 페이지 인덱스를 저장합니다
        self.next_index = next_index

        # 제목 레이블을 생성합니다
        label = QLabel(title)
        # 레이블을 중앙 정렬합니다
        label.setAlignment(Qt.AlignCenter)
        # 레이블의 글꼴 크기를 설정합니다
        label.setStyleSheet("font-size: 36px")

        # 선택된 옵션을 추적하는 변수를 초기화합니다
        self.selected_option = None

        # 수직 레이아웃을 생성합니다
        layout = QVBoxLayout()
        # 레이아웃에 제목 레이블을 추가합니다
        layout.addWidget(label)

        # 각 옵션에 대한 버튼을 생성합니다
        for name, price in options:
            # 옵션 이름과 가격이 표시된 버튼을 생성합니다
            btn = QPushButton(f"{name} ({price}원)")
            # 버튼의 글꼴 크기를 설정합니다
            btn.setStyleSheet("font-size: 24px")
            
            # 버튼 클릭 시 해당 옵션을 선택하는 함수를 연결합니다
            # 람다 함수에서 기본 인자를 사용하여 현재 루프의 값을 캡처합니다
            btn.clicked.connect(lambda _, n=name, p=price: self.select_option(n, p))
            
            # 버튼을 레이아웃에 추가합니다
            layout.addWidget(btn)

        # '장바구니' 버튼을 생성합니다
        cart_btn = QPushButton("장바구니")
        # 버튼의 글꼴 크기를 설정합니다
        cart_btn.setStyleSheet("font-size: 24px")
        # 버튼 클릭 시 go_next 메서드를 호출하도록 연결합니다
        cart_btn.clicked.connect(self.go_next)
        # 버튼을 레이아웃에 추가합니다
        layout.addWidget(cart_btn)
        # 위젯에 레이아웃을 설정합니다
        self.setLayout(layout)

    def select_option(self, name, price):
        """선택된 옵션을 order_data에 저장"""
        # 페이지 제목을 소문자로 변환하여 주문 데이터의 키로 사용합니다
        # (예: "Sauce" -> "sauce")
        self.order_data[self.title.lower()] = {'name': name, 'price': price}

    def go_next(self):
        """다음 페이지로 이동"""
        # 다음 페이지로 이동합니다
        self.stack.setCurrentIndex(self.next_index)


# ============= 주문 확인 페이지 ==============
class ConfirmPage(QWidget):
    def __init__(self, stack, order_data):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()
        
        # 화면 전환을 위한 스택 위젯 참조를 저장합니다
        self.stack = stack
        # 주문 데이터를 저장할 딕셔너리 참조를 저장합니다
        self.order_data = order_data

        # 수직 레이아웃을 생성합니다
        layout = QVBoxLayout()

        # 주문 요약 정보를 표시할 레이블을 생성합니다
        self.summary = QLabel()
        # 레이블의 글꼴 크기를 설정합니다
        self.summary.setStyleSheet("font-size: 20px")
        # 레이블을 레이아웃에 추가합니다
        layout.addWidget(self.summary)

        # '주문 완료' 버튼을 생성합니다
        confirm_btn = QPushButton("주문 완료")
        # 버튼의 글꼴 크기를 설정합니다
        confirm_btn.setStyleSheet("font-size: 24px")

        # '처음으로' 버튼을 생성합니다
        home_btn = QPushButton("처음으로")
        # 버튼의 글꼴 크기를 설정합니다
        home_btn.setStyleSheet("font-size: 24px")

        # 주문 완료 버튼 클릭 시 결제 페이지(index 7)로 이동하도록 설정합니다
        confirm_btn.clicked.connect(lambda: stack.setCurrentIndex(7))
        # 처음으로 버튼 클릭 시 시작 페이지(index 0)로 이동하도록 설정합니다
        home_btn.clicked.connect(lambda: stack.setCurrentIndex(0))

        # 버튼들을 레이아웃에 추가합니다
        layout.addWidget(confirm_btn)
        layout.addWidget(home_btn)
        # 위젯에 레이아웃을 설정합니다
        self.setLayout(layout)

    def showEvent(self, event):
        """페이지가 표시될 때 주문 내역 요약을 업데이트"""
        # 주문 내역 텍스트를 초기화합니다
        text = "[주문 내역]\n"
        # 총 금액 변수를 초기화합니다
        total = 0

        # 저장된 주문 데이터를 순회하며 내역을 생성합니다
        for key in ['menu', 'sauce', 'vegetables', 'cheese']:
            # 해당 키가 주문 데이터에 있는지 확인합니다
            if key in self.order_data:
                # 아이템 정보를 가져옵니다
                item = self.order_data[key]
                # 아이템 이름을 가져옵니다
                name = item['name']
                # 아이템 가격을 가져옵니다
                price = item['price']
                # 아이템 수량을 가져옵니다 (없으면 기본값 1)
                qty = item.get('qty', 1)
                # 항목별 소계를 계산합니다
                subtotal = price * qty
                
                # 항목 정보를 텍스트에 추가합니다
                text += f"- {name} x{qty}: {subtotal}원\n"
                # 총 금액에 소계를 추가합니다
                total += subtotal

        # 총 금액 정보를 텍스트에 추가합니다
        text += f"\n총 금액: {total}원"
        # 요약 레이블의 텍스트를 업데이트합니다
        self.summary.setText(text)


# ============= 결제 페이지 ==============class PaymentPage(QWidget):
    def __init__(self, stack, order_data=None):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()
        
        # 화면 전환을 위한 스택 위젯 참조를 저장합니다
        self.stack = stack
        # 주문 데이터를 저장할 딕셔너리 참조를 저장합니다
        self.order_data = order_data

        # 결제 방식 선택 레이블을 생성합니다
        label = QLabel("결제 방식 선택")
        # 레이블을 중앙 정렬합니다
        label.setAlignment(Qt.AlignCenter)
        # 레이블의 글꼴 크기를 설정합니다
        label.setStyleSheet("font-size: 36px")

        # 결제 방식 버튼들을 생성합니다
        card_btn = QPushButton("신용카드 결제")
        pay_btn = QPushButton("스마트페이")
        qr_btn = QPushButton("QR 코드 결제") # 추가된 버튼
        
        # 각 버튼의 글꼴 크기를 설정합니다
        card_btn.setStyleSheet("font-size: 24px")
        pay_btn.setStyleSheet("font-size: 24px")
        qr_btn.setStyleSheet("font-size: 24px")

        # 각 결제 방식 버튼 클릭 시 동작을 설정합니다
        card_btn.clicked.connect(lambda: stack.setCurrentIndex(8)) # 완료 페이지로 이동
        pay_btn.clicked.connect(lambda: stack.setCurrentIndex(8)) # 완료 페이지로 이동
        qr_btn.clicked.connect(lambda: stack.setCurrentIndex(9)) # QR 코드 결제 페이지로 이동

        # 수직 레이아웃을 생성합니다
        layout = QVBoxLayout()
        # 레이아웃에 위젯들을 추가합니다
        layout.addWidget(label)
        layout.addWidget(card_btn)
        layout.addWidget(pay_btn)
        layout.addWidget(qr_btn)
        # 위젯에 레이아웃을 설정합니다
        self.setLayout(layout)

# ============= 완료 페이지 ==============
class CompletePage(QWidget):
    def __init__(self):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()

        # 주문 완료 메시지 레이블을 생성합니다
        label = QLabel("주문 완료! 제조를 시작합니다!")
        # 레이블을 중앙 정렬합니다
        label.setAlignment(Qt.AlignCenter)
        # 레이블의 글꼴 크기와 색상을 설정합니다
        label.setStyleSheet("font-size: 48px; color: green")

        # 수직 레이아웃을 생성합니다
        layout = QVBoxLayout()
        # 레이아웃에 레이블을 추가합니다
        layout.addWidget(label)
        # 위젯에 레이아웃을 설정합니다
        self.setLayout(layout)

# ============ 주문 데이터 감시 기능 추가 =========
class OrderDataWatcher(QThread):
    # 주문 데이터 수신 신호를 정의합니다
    orderDataReceived = pyqtSignal(dict)

    def __init__(self):
        # 부모 클래스의 초기화 메서드를 호출합니다
        super().__init__()
        # 스레드 실행 상태를 초기화합니다
        self.running = True

    def run(self):
        # 필요한 모듈을 가져옵니다
        import os
        import json
        import time

        # 파일의 마지막 수정 시간을 추적하기 위한 변수를 초기화합니다
        last_modified = 0
        
        # 스레드가 실행 상태인 동안 반복합니다
        while self.running:
            try:
                # order_data.json 파일이 존재하는지 확인합니다
                if os.path.exists("order_data.json"):
                    # 파일의 현재 수정 시간을 가져옵니다
                    current_modified = os.path.getmtime("order_data.json")
                    
                    # 파일이 마지막 확인 이후 수정되었는지 확인합니다
                    if current_modified > last_modified:
                        # 파일을 열어 주문 데이터를 읽어옵니다
                        with open("order_data.json", "r", encoding="utf-8") as f:
                            order_data = json.load(f)
                            
                            # 주문 데이터 수신 신호를 발생시킵니다
                            self.orderDataReceived.emit(order_data)
                        
                        # 마지막 수정 시간을 업데이트합니다
                        last_modified = current_modified
            except Exception as e:
                # 주문 데이터 읽기 오류를 출력합니다
                print(f"주문 데이터 읽기 오류: {e}")
            
            # 1초 간격으로 파일 변경을 확인합니다
            time.sleep(1)

    def stop(self):
        # 스레드 실행 상태를 중지로 설정합니다
        self.running = False

class PaymentPage(QWidget):
    def __init__(self, stack, order_data=None):
        # 부모 클래스(QWidget)의 초기화 메서드를 호출합니다.
        super().__init__()

        # 페이지 전환을 위한 QStackedWidget 객체를 저장합니다.
        self.stack = stack
        # 주문 정보를 저장할 딕셔너리를 저장합니다.
        self.order_data = order_data

        # 결제 방식 선택 안내 레이블을 생성합니다.
        label = QLabel("결제 방식 선택")
        # 레이블을 중앙 정렬합니다.
        label.setAlignment(Qt.AlignCenter)
        # 레이블의 글꼴 크기를 키웁니다.
        label.setStyleSheet("font-size: 36px")

        # 신용카드 결제 버튼을 생성합니다.
        card_btn = QPushButton("신용카드 결제")
        # 버튼의 글꼴 크기를 키웁니다.
        card_btn.setStyleSheet("font-size: 24px")

        # 스마트페이 결제 버튼을 생성합니다.
        pay_btn = QPushButton("스마트페이")
        # 버튼의 글꼴 크기를 키웁니다.
        pay_btn.setStyleSheet("font-size: 24px")


        # 신용카드 결제 버튼 클릭 시 완료 페이지(예: 인덱스 8)로 이동합니다.
        card_btn.clicked.connect(lambda: stack.setCurrentIndex(8))
        # 스마트페이 결제 버튼 클릭 시 완료 페이지(예: 인덱스 8)로 이동합니다.
        pay_btn.clicked.connect(lambda: stack.setCurrentIndex(8))

        # 수직 레이아웃을 생성합니다.
        layout = QVBoxLayout()
        # 레이아웃에 안내 레이블을 추가합니다.
        layout.addWidget(label)
        # 레이아웃에 결제 버튼들을 추가합니다.
        layout.addWidget(card_btn)
        layout.addWidget(pay_btn)

        # 위젯에 레이아웃을 설정합니다.
        self.setLayout(layout)



# 1. Streamlit 서버 실행 관련 설정 -------------------------------
# ============ Streamlit 연동 및 메인 실행 =============
# Streamlit 서버 포트 번호를 설정합니다
STREAMLIT_PORT = 8501

def start_streamlit():
    # Streamlit 스크립트 파일의 절대 경로를 가져옵니다
    streamlit_script = os.path.abspath("Serboway_whisper_agent2.py")
    
    # Streamlit 실행 명령어를 설정합니다
    streamlit_cmd = [
        sys.executable, "-m", "streamlit", "run",
        streamlit_script,
        "--server.headless=True",
        "--server.port={}".format(STREAMLIT_PORT),
        "--browser.serverAddress=0.0.0.0"
    ]
    
    # 로그 파일을 생성합니다
    log_file = open("streamlit.log", "w")
    
    # Streamlit 프로세스를 시작합니다
    process = subprocess.Popen(
        streamlit_cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        # 운영체제가 Windows인 경우 shell=True로 설정합니다
        shell=True if os.name == 'nt' else False
    )
    
    # 생성된 프로세스를 반환합니다
    return process

def kill_streamlit(proc):
    try:
        # 운영체제에 따라 다른 방식으로 프로세스를 종료합니다
        if os.name == 'nt':  # Windows
            # taskkill 명령어로 프로세스와 하위 프로세스를 강제 종료합니다
            subprocess.call(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
        else:  # Unix/Linux/Mac
            # kill 메서드로 프로세스를 종료합니다
            proc.kill()
    except Exception as e:
        # 프로세스 종료 오류를 출력합니다
        print(f"Error killing process: {e}")

def handle_received_order(data, stack, order_data):
    # 기존 주문 데이터를 초기화합니다
    order_data.clear()
    
    # 새로운 주문 데이터로 업데이트합니다
    order_data.update(data)
    
    # 결제 방식 선택 페이지로 이동합니다
    stack.setCurrentIndex(7)
    
    # 처리 완료된 주문 데이터 파일을 삭제합니다
    if os.path.exists("order_data.json"):
        os.remove("order_data.json")


def main():
    app = QApplication(sys.argv)
    order_data = {}
    stack = QStackedWidget()

    # Streamlit 웹뷰
    webview = QtWebEngineWidgets.QWebEngineView()
    webview.load(QtCore.QUrl(f"http://localhost:{STREAMLIT_PORT}"))
    def handle_load_finished(ok):
        if ok:
            print("✅ Streamlit 앱 로드 완료")
        else:
            print("❌ Streamlit 앱 로드 실패")
            webview.load(QtCore.QUrl.fromLocalFile(os.path.abspath("error.html")))
    webview.loadFinished.connect(handle_load_finished)

    # 페이지 인스턴스 생성
    welcome_page = WelcomePage(stack)
    sandwich_page = SandwichPage(stack, order_data)
    sauce_page = OptionPage(stack, order_data, "Sauce", [("Italian", 0), ("Chilly", 0)], 4)
    veg_page = OptionPage(stack, order_data, "Vegetables", [("Lettuce", 0), ("Romaine", 700), ("Bazil", 800)], 5)
    cheese_page = OptionPage(stack, order_data, "Cheese", [("Slice", 0), ("Shred", 1000), ("Mozzarella", 1300)], 6)
    confirm_page = ConfirmPage(stack, order_data)
    payment_page = PaymentPage(stack, order_data)
    complete_page = CompletePage()

    # QStackedWidget에 페이지 추가 (인덱스 주석 참고)
    stack.addWidget(welcome_page)      # 0: 환영
    stack.addWidget(webview)           # 1: 음성 주문 (Streamlit)
    stack.addWidget(sandwich_page)     # 2: 샌드위치 선택
    stack.addWidget(sauce_page)        # 3: 소스 선택
    stack.addWidget(veg_page)          # 4: 야채 선택
    stack.addWidget(cheese_page)       # 5: 치즈 선택
    stack.addWidget(confirm_page)      # 6: 주문 확인
    stack.addWidget(payment_page)      # 7: 결제 방식 선택
    stack.addWidget(complete_page)     # 8: 주문 완료


    # 주문 데이터 감시 스레드
    order_watcher = OrderDataWatcher()
    order_watcher.orderDataReceived.connect(lambda data: handle_received_order(data, stack, order_data))
    order_watcher.start()

    # 메인 윈도우
    main_window = QWidget()
    main_layout = QVBoxLayout()
    # 레이아웃에 스택 위젯을 추가
    main_layout.addWidget(stack)
    main_window.setLayout(main_layout)
    # 윈도우 제목을 설정
    main_window.setWindowTitle("SerboWay Kiosk")
    # 윈도우 크기를 설정
    main_window.resize(500, 600)

    # 초기 페이지를 환영페이지(0번 인덱스로 설정
    stack.setCurrentIndex(0)  # 환영 페이지(WelcomePage)로 설정
    # 윈도우를 화면에 표시 합니다.
    main_window.show()

    # Streamlit 서버 실행
    streamlit_process = start_streamlit()
    atexit.register(kill_streamlit, streamlit_process)
     # 애플리케이션 종료 시 주문 데이터 감시 스레드를 중지하도록 연결합니다
    app.aboutToQuit.connect(order_watcher.stop)
    # 애플리케이션 실행 루프를 시작하고, 종료 시 시스템에 종료 코드를 반환합니다
    sys.exit(app.exec_())

# 스크립트가 직접 실행될 떄만 main 함수를 호출한다.
if __name__ == "__main__":
    main()
