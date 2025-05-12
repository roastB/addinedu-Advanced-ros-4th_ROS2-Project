import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QStackedWidget, QSpinBox, QListWidget, QMessageBox
)
from PyQt5.QtCore import Qt

import sys, os, atexit, subprocess
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore

class WelcomePage(QWidget):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        label = QLabel("Welcome to SerboWay!")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 48px; font-weight: bold")
        # 음성 주문 페이지
        voice_btn = QPushButton("🎙 Voice Order")
        voice_btn.setStyleSheet("font-size: 24px; font-weight: bold")
        # 터치 주문 버튼
        touch_btn = QPushButton("🖐 Touch Order")
        touch_btn.setStyleSheet("font-size: 24px; font-weight: bold")
        voice_btn.clicked.connect(lambda: stack.setCurrentIndex(1))
        touch_btn.clicked.connect(lambda: stack.setCurrentIndex(2))
        # 세로 레이아웃에 위젯 추가
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(voice_btn)
        layout.addWidget(touch_btn)
        self.setLayout(layout)


# ============== 샌드위치 선택 페이지================
class SandwichPage(QWidget):
    def __init__(self, stack, order_data):
        super().__init__()
        self.stack = stack
        self.order_data = order_data

        label = QLabel("Sandwiches")
        label.setStyleSheet(("font-size: 48px; font-weight: bold"))
        label.setAlignment(Qt.AlignCenter)

        self.menu_buttons = []
        self.spin_boxes = []

        menu_items = [
            ("Bulgogi", 6500),
            ("Shrimp", 6200),
            ("Bacon", 6000)
        ]

        layout = QVBoxLayout()
        layout.addWidget(label)

        for name, price in menu_items:
            h = QHBoxLayout()
            btn = QPushButton(f"{name} ({price}원)")
            btn.setStyleSheet("font-size: 24px")
            spin = QSpinBox()
            spin.setRange(0, 10)
            spin.setStyleSheet("font-size: 24px")
            h.addWidget(btn)
            h.addWidget(spin)
            layout.addLayout(h)
            self.menu_buttons.append((name, price, spin))

        cart_btn = QPushButton("장바구니")
        cart_btn.setStyleSheet("font-size: 24px")
        cart_btn.clicked.connect(self.go_next)
        layout.addWidget(cart_btn)

        self.setLayout(layout)

    def go_next(self):
        for name, price, spin in self.menu_buttons:
            qty = spin.value()
            if qty > 0:
                self.order_data['menu'] = {'name': name, 'price': price, 'qty': qty}
        self.stack.setCurrentIndex(3)


# ============= 옵션 선택 페이지 =====================
class OptionPage(QWidget):
    def __init__(self, stack, order_data, title, options, next_index):
        super().__init__()
        self.stack = stack
        self.order_data = order_data
        self.title = title
        self.options = options
        self.next_index = next_index

        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 36px")

        self.selected_option = None
        self.option_buttons = []

        layout = QVBoxLayout()
        layout.addWidget(label)

        for name, price in options:
            btn = QPushButton(f"{name} ({price}원)")
            btn.setStyleSheet("font-size: 24px")
            btn.clicked.connect(lambda _, n=name, p=price: self.select_option(n, p))
            layout.addWidget(btn)
            self.option_buttons.append(btn)

        cart_btn = QPushButton("장바구니")
        cart_btn.setStyleSheet("font-size: 24px")
        cart_btn.clicked.connect(self.go_next)
        layout.addWidget(cart_btn)

        self.setLayout(layout)

    def select_option(self, name, price):
        self.order_data[self.title.lower()] = {'name': name, 'price': price}

    def go_next(self):
        self.stack.setCurrentIndex(self.next_index)


# ============= 주문 확인 페이지 ===================
class ConfirmPage(QWidget):
    def __init__(self, stack, order_data):
        super().__init__()
        self.stack = stack
        self.order_data = order_data

        layout = QVBoxLayout()
        self.summary = QLabel()
        self.summary.setStyleSheet("font-size: 20px")
        layout.addWidget(self.summary)

        confirm_btn = QPushButton("주문 완료")
        confirm_btn.setStyleSheet("font-size: 24px")
        home_btn = QPushButton("처음으로")
        home_btn.setStyleSheet("font-size: 24px")
        confirm_btn.clicked.connect(lambda: stack.setCurrentIndex(7))
        home_btn.clicked.connect(lambda: stack.setCurrentIndex(0))

        layout.addWidget(confirm_btn)
        layout.addWidget(home_btn)
        self.setLayout(layout)

    def showEvent(self, event):
        text = "[주문 내역]\n"
        total = 0
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


# ============= 결제 페이지 ============
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

        card_btn.clicked.connect(lambda: stack.setCurrentIndex(8))
        pay_btn.clicked.connect(lambda: stack.setCurrentIndex(8))

        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(card_btn)
        layout.addWidget(pay_btn)
        self.setLayout(layout)


# ======== 완료 페이지 =============
class CompletePage(QWidget):
    def __init__(self):
        super().__init__()
        label = QLabel("주문 완료! 제조를  시작합니다!")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 48px; color: green")
        layout = QVBoxLayout()
        layout.addWidget(label)
        self.setLayout(layout)

# ============ Streamlit 연동 =================
# 1. Streamlit 서버 백그라운드 실행 (headless 모드로 브라우저 자동 안 뜨게)
streamlit_cmd = [sys.executable, "-m", "streamlit", "run", "Serboway_whisper_agent2.py", "--server.headless=True"]
process = subprocess.Popen(streamlit_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)  # 비동기 실행

# 앱 종료 시 Streamlit 프로세스도 종료하도록 설정
def kill_streamlit(proc):
    if os.name == 'nt':
        # Windows: 스트림릿 프로세스 (자식 포함) 강제 종료
        subprocess.call(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
    else:
        # Unix 계열: 프로세스.kill()로 종료
        proc.kill()

atexit.register(kill_streamlit, process)  # 프로그램 종료 시점에 호출 등록

# 2. PyQt 애플리케이션 설정 및 WebEngineView 생성
app = QtWidgets.QApplication(sys.argv)
main_win = QtWidgets.QMainWindow()
main_win.setWindowTitle("Voice Order System")

# QWebEngineView를 메인 윈도우 중심 위젯으로 사용 (필요하면 QStackedWidget 활용 가능)
webview = QtWebEngineWidgets.QWebEngineView()
webview.load(QtCore.QUrl("http://localhost:8501"))  # 스트림릿 서버 주소 로드:contentReference[oaicite:20]{index=20}
main_win.setCentralWidget(webview)
main_win.resize(1200, 800)  # 윈도우 크기 조정
main_win.show()

# 3. (선택) Streamlit 웹 페이지 로딩 완료 후 처리 - 예: 로딩 중 안내, 완료 시 콘솔 로그 출력
def on_load_finished(ok):
    if ok:
        print("Streamlit app loaded successfully in WebView.")
    else:
        print("Failed to load Streamlit app.")

webview.loadFinished.connect(on_load_finished)

# 4. 애플리케이션 실행
sys.exit(app.exec_())





app = QApplication(sys.argv)
stack = QStackedWidget()
order_data = {}

stack.addWidget(WelcomePage(stack))            # 0
stack.addWidget(QWidget())                     # 1 (Voice order - 생략 가능)
stack.addWidget(SandwichPage(stack, order_data))  # 2
stack.addWidget(OptionPage(stack, order_data, "Sauce", [("Italian", 0), ("Chilly", 0)], 4)) # 3
stack.addWidget(OptionPage(stack, order_data, "Vegetables", [("Lettuce", 0), ("Romaine", 700), ("Bazil", 800)], 5)) # 4
stack.addWidget(OptionPage(stack, order_data, "Cheese", [("Slice", 0), ("Shred", 1000), ("Mozzarella", 1300)], 6))  # 5
stack.addWidget(ConfirmPage(stack, order_data))  # 6
stack.addWidget(PaymentPage(stack))           # 7
stack.addWidget(CompletePage())               # 8

main_window = QWidget()
main_layout = QVBoxLayout()
main_layout.addWidget(stack)
main_window.setLayout(main_layout)
main_window.setWindowTitle("SerboWay Kiosk")
main_window.resize(400, 500)
main_window.show()

sys.exit(app.exec_())
