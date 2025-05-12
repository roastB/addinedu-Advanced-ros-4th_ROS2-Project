import sys, os, atexit, subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QStackedWidget, QSpinBox, QListWidget, QMessageBox
)
from PyQt5.QtCore import Qt
from PyQt5 import QtWebEngineWidgets, QtCore
import websockets
import sys, os, atexit, subprocess
import time
from PyQt5.QtWidgets import QApplication
from PyQt5 import QtWebEngineWidgets, QtCore
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
import cv2
from pyzbar.pyzbar import decode

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

    ## 삭제 예정
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
    def __init__(self, stack, order_data=None):
        super().__init__()
        self.stack = stack
        self.order_data = order_data
        
        label = QLabel("결제 방식 선택")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 36px")
        
        card_btn = QPushButton("신용카드 결제")
        pay_btn = QPushButton("스마트페이")
        qr_btn = QPushButton("QR 코드 결제")  # 추가된 버튼
        
        card_btn.setStyleSheet("font-size: 24px")
        pay_btn.setStyleSheet("font-size: 24px")
        qr_btn.setStyleSheet("font-size: 24px")
        
        # 각 결제 방식에 따른 동작 설정
        card_btn.clicked.connect(lambda: stack.setCurrentIndex(8))  # 완료 페이지
        pay_btn.clicked.connect(lambda: stack.setCurrentIndex(8))   # 완료 페이지
        qr_btn.clicked.connect(lambda: stack.setCurrentIndex(9))    # QR 코드 결제 페이지
        
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(card_btn)
        layout.addWidget(pay_btn)
        layout.addWidget(qr_btn)
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

# ========== QR Code ==================
# QR 코드 인식 스레드 클래스
class QRCodeReader(QThread):
    # 신호 정의
    imageUpdate = pyqtSignal(QImage)
    qrCodeDetected = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.threadActive = False
        self.cap = None

    def run(self):
        self.threadActive = True
        self.cap = cv2.VideoCapture(0)  # 카메라 연결
        
        while self.threadActive:
            ret, frame = self.cap.read()
            if not ret: continue
                
            # QR 코드 감지
            try:
                decoded_objects = decode(frame)
                for obj in decoded_objects:
                    qr_data = obj.data.decode('utf-8')
                    self.qrCodeDetected.emit(qr_data)
            except Exception as e:
                print(f"QR 코드 감지 오류: {e}")
                
            # 이미지를 PyQt에 표시할 형식으로 변환
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            scaled_image = qt_image.scaled(640, 480, Qt.KeepAspectRatio)
            self.imageUpdate.emit(scaled_image)
    
    def stop(self):
        self.threadActive = False
        if self.cap:
            self.cap.release()

# =========== QR 코드 결제 페이지
class QRPaymentPage(QWidget):
    paymentCompleted = pyqtSignal(dict)  # 결제 완료 시 주문 정보를 전달하는 신호
    
    def __init__(self, stack, order_data=None):
        super().__init__()
        self.stack = stack
        self.order_data = order_data
        
        # 레이아웃 설정
        layout = QVBoxLayout()
        
        # 안내 레이블
        label = QLabel("QR 코드를 카메라에 보여주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 24px; font-weight: bold")
        layout.addWidget(label)
        
        # 금액 표시 레이블
        self.amount_label = QLabel("금액 계산 중...")
        self.amount_label.setAlignment(Qt.AlignCenter)
        self.amount_label.setStyleSheet("font-size: 36px; color: green")
        layout.addWidget(self.amount_label)
        
        # 카메라 영상을 표시할 레이블
        self.camera_view = QLabel()
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setMinimumSize(640, 480)
        layout.addWidget(self.camera_view)
        
        # 취소 버튼
        cancel_btn = QPushButton("취소")
        cancel_btn.setStyleSheet("font-size: 18px")
        cancel_btn.clicked.connect(self.cancel_payment)
        layout.addWidget(cancel_btn)
        
        self.setLayout(layout)
        
        # QR 코드 인식 스레드 초기화
        self.qr_reader = None
    
    def showEvent(self, event):
        """페이지가 표시될 때 호출됨"""
        # 주문 금액 계산 및 표시
        self.calculate_amount()
        
        # QR 코드 인식 시작
        self.start_qr_reader()
        super().showEvent(event)
    
    def hideEvent(self, event):
        """페이지가 숨겨질 때 호출됨"""
        self.stop_qr_reader()
        super().hideEvent(event)
    
    def calculate_amount(self):
        """주문 금액 계산"""
        total = 0
        if self.order_data and 'menu' in self.order_data:
            menu = self.order_data['menu']
            total = menu['price'] * menu['qty']
            
            # 추가 옵션 금액 계산
            for option_type in ['sauce', 'vegetables', 'cheese']:
                if option_type in self.order_data and 'price' in self.order_data[option_type]:
                    total += self.order_data[option_type]['price']
        
        self.amount_label.setText(f"결제 금액: {total}원")
        
    def start_qr_reader(self):
        """QR 코드 인식 스레드 시작"""
        self.qr_reader = QRCodeReader()
        self.qr_reader.imageUpdate.connect(self.update_camera_view)
        self.qr_reader.qrCodeDetected.connect(self.process_qr_code)
        self.qr_reader.start()
    
    def stop_qr_reader(self):
        """QR 코드 인식 스레드 중지"""
        if self.qr_reader:
            self.qr_reader.stop()
            self.qr_reader = None
    
    def update_camera_view(self, image):
        """카메라 영상 업데이트"""
        self.camera_view.setPixmap(QPixmap.fromImage(image))
    
    def process_qr_code(self, qr_data):
        """QR 코드 처리"""
        # 여기서 QR 코드 데이터의 유효성을 검증
        # 실제 환경에서는 결제 서비스의 API 형식에 맞게 검증해야 함
        self.stop_qr_reader()  # QR 코드 인식 중지
        
        # 결제 처리 및 서버로 주문 정보 전송
        self.send_order_to_server(qr_data)
        
        # 결제 완료 신호 발생
        self.paymentCompleted.emit(self.order_data)
        
        # 완료 페이지로 이동
        self.stack.setCurrentIndex(8)
    
    def cancel_payment(self):
        """결제 취소"""
        self.stop_qr_reader()
        self.stack.setCurrentIndex(7)  # 결제 방식 선택 페이지로 돌아가기
    
    def send_order_to_server(self, payment_id):
        """주문 정보를 메인 서버로 전송"""
        try:
            import requests
            import json
            
            # 서버 URL (실제 서버 URL로 변경 필요)
            server_url = "http://your-server-url.com/api/orders"
            
            # 전송할 데이터 준비
            data = {
                "payment_id": payment_id,
                "order_data": self.order_data
            }
            
            # POST 요청 전송
            headers = {"Content-Type": "application/json"}
            response = requests.post(server_url, data=json.dumps(data), headers=headers)
            
            if response.status_code == 200:
                print("✅ 주문 정보가 서버에 성공적으로 전송되었습니다.")
            else:
                print(f"❌ 서버 통신 오류: {response.status_code}")
                
        except Exception as e:
            print(f"❌ 서버 통신 중 오류 발생: {e}")

# ============ 주문 데이터 감시 기능 추가 =========
class OrderDataWatcher(QThread):
    orderDataReceived = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.running = True
    
    def run(self):
        import os
        import json
        import time
        
        last_modified = 0
        
        while self.running:
            try:
                if os.path.exists("order_data.json"):
                    current_modified = os.path.getmtime("order_data.json")
                    
                    if current_modified > last_modified:
                        with open("order_data.json", "r", encoding="utf-8") as f:
                            order_data = json.load(f)
                        
                        self.orderDataReceived.emit(order_data)
                        last_modified = current_modified
            except Exception as e:
                print(f"주문 데이터 읽기 오류: {e}")
            
            time.sleep(1)  # 1초마다 확인
    
    def stop(self):
        self.running = False



# 1. Streamlit 서버 실행 관련 설정 -------------------------------
# ============ Streamlit 연동 및 메인 실행 =============
STREAMLIT_PORT = 8501
def start_streamlit():
    streamlit_script = os.path.abspath("Serboway_whisper_agent2.py")
    streamlit_cmd = [
        sys.executable, "-m", "streamlit", "run",
        streamlit_script,
        "--server.headless=True",
        "--server.port={}".format(STREAMLIT_PORT),
        "--browser.serverAddress=0.0.0.0"
    ]
    log_file = open("streamlit.log", "w")
    process = subprocess.Popen(
        streamlit_cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        shell=True if os.name == 'nt' else False
    )
    return process

def kill_streamlit(proc):
    try:
        if os.name == 'nt':
            subprocess.call(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
        else:
            proc.kill()
    except Exception as e:
        print(f"Error killing process: {e}")

def handle_received_order(data, stack, order_data):
    order_data.clear()
    order_data.update(data)
    stack.setCurrentIndex(7)  # 결제 방식 선택 페이지로 이동

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
    qr_payment_page = QRPaymentPage(stack, order_data)

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
    stack.addWidget(qr_payment_page)   # 9: QR 결제

    # 주문 데이터 감시 스레드
    order_watcher = OrderDataWatcher()
    order_watcher.orderDataReceived.connect(lambda data: handle_received_order(data, stack, order_data))
    order_watcher.start()

    # QR 결제 완료 시
    qr_payment_page.paymentCompleted.connect(lambda data: print("결제 완료:", data))

    # 메인 윈도우
    main_window = QWidget()
    main_layout = QVBoxLayout()
    main_layout.addWidget(stack)
    main_window.setLayout(main_layout)
    main_window.setWindowTitle("SerboWay Kiosk")
    main_window.resize(500, 600)
    main_window.show()

    # Streamlit 서버 실행
    streamlit_process = start_streamlit()
    atexit.register(kill_streamlit, streamlit_process)
    app.aboutToQuit.connect(order_watcher.stop)

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
