import sys, os, atexit, subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QStackedWidget, QSpinBox, QListWidget, QMessageBox
)
from PyQt5.QtCore import Qt
from PyQt5 import QtWebEngineWidgets, QtCore

class WelcomePage(QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        # 환영 문구 레이블
        label = QLabel("Welcome to SerboWay!")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 48px; font-weight: bold")

        # 🎙 음성 주문 버튼 (Voice Order)
        voice_btn = QPushButton("🎙 Voice Order")
        voice_btn.setStyleSheet("font-size: 24px; font-weight: bold")
        # 🖐 터치 주문 버튼 (Touch Order)
        touch_btn = QPushButton("🖐 Touch Order")
        touch_btn.setStyleSheet("font-size: 24px; font-weight: bold")

        # 버튼 클릭 시 각 페이지로 이동 (Voice = index 1, Touch = index 2)
        voice_btn.clicked.connect(lambda: stack.setCurrentIndex(1))
        touch_btn.clicked.connect(lambda: stack.setCurrentIndex(2))

        # 세로 레이아웃에 위젯 추가
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(voice_btn)
        layout.addWidget(touch_btn)
        self.setLayout(layout)

# ============== 샌드위치 선택 페이지 ==============
class SandwichPage(QWidget):
    def __init__(self, stack, order_data):
        super().__init__()
        self.stack = stack
        self.order_data = order_data

        label = QLabel("Sandwiches")
        label.setStyleSheet("font-size: 48px; font-weight: bold")

        # 메뉴 아이템 및 수량 선택 위젯 생성
        menu_items = [("Bulgogi", 6500), ("Shrimp", 6200), ("Bacon", 6000)]
        self.menu_buttons = []  # 메뉴 (이름, 가격, 수량스핀박스) 목록

        layout = QVBoxLayout()
        layout.addWidget(label)
        for name, price in menu_items:
            h_layout = QHBoxLayout()
            btn = QPushButton(f"{name} ({price}원)")
            btn.setStyleSheet("font-size: 24px")
            spin = QSpinBox()
            spin.setRange(0, 10)
            spin.setStyleSheet("font-size: 24px")
            # 메뉴 이름 버튼과 수량 스핀박스를 한 행에 배치
            h_layout.addWidget(btn)
            h_layout.addWidget(spin)
            layout.addLayout(h_layout)
            # 추후 선택된 내역을 저장하기 위해 목록에 추가
            self.menu_buttons.append((name, price, spin))

        # '장바구니' 버튼 (다음 단계로 진행)
        cart_btn = QPushButton("장바구니")
        cart_btn.setStyleSheet("font-size: 24px")
        cart_btn.clicked.connect(self.go_next)
        layout.addWidget(cart_btn)
        self.setLayout(layout)

    def go_next(self):
        """선택된 메뉴와 수량을 order_data에 저장하고 다음 페이지로 이동"""
        for name, price, spin in self.menu_buttons:
            qty = spin.value()
            if qty > 0:  # 수량이 1개 이상 선택된 경우만 저장
                self.order_data['menu'] = {'name': name, 'price': price, 'qty': qty}
        # 다음 페이지(소스 선택 페이지)로 이동
        self.stack.setCurrentIndex(3)

# ============= 옵션 선택 페이지 ==============
class OptionPage(QWidget):
    def __init__(self, stack, order_data, title, options, next_index):
        super().__init__()
        self.stack = stack
        self.order_data = order_data
        self.title = title  # 예: "Sauce", "Vegetables", "Cheese"
        self.options = options  # 옵션 목록 (이름, 추가 가격)
        self.next_index = next_index  # 다음 페이지 인덱스

        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 36px")

        self.selected_option = None

        layout = QVBoxLayout()
        layout.addWidget(label)
        # 옵션 선택을 위한 버튼 생성
        for name, price in options:
            btn = QPushButton(f"{name} ({price}원)")
            btn.setStyleSheet("font-size: 24px")
            # 각 버튼 클릭 시 해당 옵션을 선택 상태로 설정
            btn.clicked.connect(lambda _, n=name, p=price: self.select_option(n, p))
            layout.addWidget(btn)

        # '장바구니' 버튼 (다음 단계로 진행)
        cart_btn = QPushButton("장바구니")
        cart_btn.setStyleSheet("font-size: 24px")
        cart_btn.clicked.connect(self.go_next)
        layout.addWidget(cart_btn)
        self.setLayout(layout)

    def select_option(self, name, price):
        """선택된 옵션을 order_data에 저장"""
        # title을 소문자로 변환해 키로 사용 (예: "Sauce" -> "sauce")
        self.order_data[self.title.lower()] = {'name': name, 'price': price}

    def go_next(self):
        """다음 페이지로 이동"""
        self.stack.setCurrentIndex(self.next_index)

# ============= 주문 확인 페이지 ==============
class ConfirmPage(QWidget):
    def __init__(self, stack, order_data):
        super().__init__()
        self.stack = stack
        self.order_data = order_data

        layout = QVBoxLayout()
        # 주문 요약 정보를 표시할 레이블
        self.summary = QLabel()
        self.summary.setStyleSheet("font-size: 20px")
        layout.addWidget(self.summary)

        # '주문 완료' 및 '처음으로' 버튼
        confirm_btn = QPushButton("주문 완료")
        confirm_btn.setStyleSheet("font-size: 24px")
        home_btn = QPushButton("처음으로")
        home_btn.setStyleSheet("font-size: 24px")

        # 주문 완료 클릭 -> 결제 페이지(index 7), 처음으로 클릭 -> 환영 페이지(index 0)
        confirm_btn.clicked.connect(lambda: stack.setCurrentIndex(7))
        home_btn.clicked.connect(lambda: stack.setCurrentIndex(0))

        layout.addWidget(confirm_btn)
        layout.addWidget(home_btn)
        self.setLayout(layout)

    def showEvent(self, event):
        """페이지가 표시될 때 주문 내역 요약을 업데이트"""
        text = "[주문 내역]\n"
        total = 0
        # 저장된 주문 데이터를 순회하며 내역 생성
        for key in ['menu', 'sauce', 'vegetables', 'cheese']:
            if key in self.order_data:
                item = self.order_data[key]
                name = item['name']
                price = item['price']
                qty = item.get('qty', 1)
                subtotal = price * qty
                text += f"- {name} x{qty}: {subtotal}원\n"
                total += subtotal
        text += f"\n총 금액: {total}원"
        self.summary.setText(text)

# ============= 결제 페이지 ==============
class PaymentPage(QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        label = QLabel("결제 방식 선택")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 36px")

        card_btn = QPushButton("신용카드 결제")
        pay_btn = QPushButton("스마트페이")
        card_btn.setStyleSheet("font-size: 24px")
        pay_btn.setStyleSheet("font-size: 24px")

        # 두 결제 버튼 모두 완료 페이지(index 8)로 이동
        card_btn.clicked.connect(lambda: stack.setCurrentIndex(8))
        pay_btn.clicked.connect(lambda: stack.setCurrentIndex(8))

        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(card_btn)
        layout.addWidget(pay_btn)
        self.setLayout(layout)

# ============= 완료 페이지 ==============
class CompletePage(QWidget):
    def __init__(self):
        super().__init__()
        label = QLabel("주문 완료! 제조를 시작합니다!")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 48px; color: green")
        layout = QVBoxLayout()
        layout.addWidget(label)
        self.setLayout(layout)

# ============ Streamlit 연동 =============
# 1. Streamlit 서버 백그라운드 실행 (headless 모드로 브라우저 자동 실행 안 되게)
streamlit_cmd = [sys.executable, "-m", "streamlit", "run", "Serboway_whisper_agent2.py", "--server.headless=true"]
process = subprocess.Popen(streamlit_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)  # 비동기 실행

# 앱 종료 시 Streamlit 프로세스도 함께 종료되도록 설정
def kill_streamlit(proc):
    if os.name == 'nt':
        # Windows: 스트림릿 프로세스 (자식 프로세스 포함) 강제 종료
        subprocess.call(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
    else:
        # Unix 계열: 프로세스 종료
        proc.kill()

atexit.register(kill_streamlit, process)  # 프로그램 종료 시 kill_streamlit 호출 등록

# 2. PyQt 애플리케이션 설정 및 QWebEngineView 통합
app = QApplication(sys.argv)
stack = QStackedWidget()
order_data = {}  # 주문 데이터를 저장할 딕셔너리

# 음성 주문 페이지용 QWebEngineView 생성 및 로드
webview = QtWebEngineWidgets.QWebEngineView()
webview.load(QtCore.QUrl("http://localhost:8502"))  # Streamlit 웹 앱 로드 (로컬호스트)
# (선택 사항) Streamlit 웹 페이지 로딩 완료 이벤트 처리 - 로드 성공/실패 콘솔 출력
def on_load_finished(ok):
    if ok:
        print("Streamlit app loaded successfully in WebView.")
    else:
        print("Failed to load Streamlit app in WebView.")
webview.loadFinished.connect(on_load_finished)

# QStackedWidget에 모든 페이지 추가 (인덱스 주석 참고)
stack.addWidget(WelcomePage(stack))                   # index 0: 환영 페이지
stack.addWidget(webview)                              # index 1: 음성 주문 페이지 (Streamlit 웹뷰)
stack.addWidget(SandwichPage(stack, order_data))      # index 2: 샌드위치 선택 페이지 (터치 주문 시작)
stack.addWidget(OptionPage(stack, order_data, "Sauce", [("Italian", 0), ("Chilly", 0)], 4))       # index 3: 소스 선택
stack.addWidget(OptionPage(stack, order_data, "Vegetables", [("Lettuce", 0), ("Romaine", 700), ("Bazil", 800)], 5))  # index 4: 야채 선택
stack.addWidget(OptionPage(stack, order_data, "Cheese", [("Slice", 0), ("Shred", 1000), ("Mozzarella", 1300)], 6))   # index 5: 치즈 선택
stack.addWidget(ConfirmPage(stack, order_data))       # index 6: 주문 확인 페이지
stack.addWidget(PaymentPage(stack))                   # index 7: 결제 방식 선택 페이지
stack.addWidget(CompletePage())                       # index 8: 주문 완료 페이지

# 메인 윈도우 설정: QStackedWidget을 중앙에 배치
main_window = QWidget()
main_layout = QVBoxLayout()
main_layout.addWidget(stack)
main_window.setLayout(main_layout)
main_window.setWindowTitle("SerboWay Kiosk")
main_window.resize(400, 500)
main_window.show()

# 3. 애플리케이션 이벤트 루프 실행 (단일 QApplication 사용)
sys.exit(app.exec_())
