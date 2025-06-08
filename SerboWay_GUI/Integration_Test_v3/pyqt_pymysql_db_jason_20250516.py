# ===================== 표준 라이브러리 (Python Built-in Modules) =====================
import sys  # 시스템 관련 기능(프로그램 종료, 경로 조작 등)
import os  # 운영체제 인터페이스(파일 경로, 환경변수 등)
import json  # JSON 데이터 처리
import random  # 난수 생성(영수증 번호 생성용)
import string  # 문자열 유틸리티(영수증 번호 생성용)
import socket  # 네트워크 통신(음성 주문-키오스크 간 TCP 통신)
import subprocess  # 외부 프로세스 실행(Streamlit 서버 실행)
import signal  # 신호 처리(프로세스 제어)
from datetime import datetime  # 시간 관련 기능(주문 타임스탬프)
from typing import Optional, Dict, Any, List  # 타입 힌트
import pymysql  # MySQL 데이터베이스 연결
import requests  # HTTP 요청 처리(메인 서버 API 호출)

# ========= PyQt 모듈 ================
# GUI 위젯
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QListWidget, QLabel,
    QComboBox, QPushButton, QHBoxLayout, QStackedWidget, QMainWindow,
    QMessageBox
)
# 코어 기능
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QUrl  # 이벤트 루프, 시그널, 타이머
# 웹 엔진
from PyQt5.QtWebEngineWidgets import QWebEngineView  # 웹뷰(Streamlit 표시용)

# ============ 서버 API 설정 =============
MENU_SERVER_URL = "http://192.168.0.178:5003/api/menu"  # 메뉴 정보 API 주소
ORDER_SERVER_URL = "http://192.168.0.178:5003/api/order"  # 주문 전송 API 주소

# ============ Streamlit 설정 =============
STREAMLIT_PORT = 8502  # Streamlit 서버 포트
STREAMLIT_SCRIPT = "voice_agent.py"  # 음성 에이전트 스크립트 경로

def send_order_to_server(order_data):
    """주문 정보를 메인 서버로 전송"""
    try:
        response = requests.post(ORDER_SERVER_URL, json=order_data)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"주문 서버 연결 실패: {e}")
        return {"status": "fail", "message": str(e)}

def get_menu_json(server_url=MENU_SERVER_URL, local_file="menu_data.json"):
    """
    메뉴 JSON을 가져오는 함수 (메인 서버→로컬 파일→기본값 순서로 시도)
    """
    # 1. 메인 서버에서 가져오기 시도
    try:
        print(f"메인 서버({server_url})에서 메뉴 데이터 가져오기 시도...")
        response = requests.get(server_url, timeout=5)
        if response.status_code == 200:
            menu_data = response.json()
            if menu_data and "menu" in menu_data:
                print("메인 서버에서 메뉴 데이터를 성공적으로 불러왔습니다.")
                # 성공 시 로컬에도 저장해둠 (백업)
                try:
                    with open(local_file, "w", encoding="utf-8") as f:
                        json.dump(menu_data, f, ensure_ascii=False, indent=2)
                except Exception as e:
                    print(f"로컬 저장 실패: {e}")
                return menu_data
    except Exception as e:
        print(f"서버 연결 실패: {e}")
    
    # 2. 로컬 파일에서 가져오기 시도
    try:
        print(f"로컬 파일({local_file})에서 메뉴 데이터 가져오기 시도...")
        with open(local_file, "r", encoding="utf-8") as f:
            menu_data = json.load(f)
            if menu_data and "menu" in menu_data:
                print("로컬 파일에서 메뉴 데이터를 성공적으로 불러왔습니다.")
                return menu_data
    except Exception as e:
        print(f"로컬 파일 불러오기 실패: {e}")
    
    # 3. 기본값 반환
    print("메뉴 데이터를 불러오지 못했습니다. 기본 구조를 사용합니다.")
    return {"menu": {}, "sauce": {}, "vegetable": {}, "cheese": {}}

# 콤보박스에 아이템 표시 및 데이터 연결
def populate_combo(combo, items_dict):
    """콤보박스에 메뉴/옵션 항목 채우기"""
    combo.clear()
    for key, info in items_dict.items():
        price = info.get('price', 0)
        combo.addItem(f"{key} ({price}원)", (key, price))

# ======== KioskServer 클래스 ================
class KioskServer:
    """TCP 서버 클래스 (음성 에이전트와 통신)"""
    def __init__(self):
        self.socket = None
        self.running = False
        self.current_order = None
        self.payment_result = None

    def start(self, host='0.0.0.0', port=12345):
        """TCP 서버 시작"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind((host, port))
            self.socket.listen(1)
            self.running = True
            print(f"키오스크 TCP 서버 시작: {host}:{port}")
        except Exception as e:
            print(f"TCP 서버 시작 오류: {e}")

    def handle_connection(self):
        """클라이언트 연결 처리"""
        while self.running:
            try:
                client, addr = self.socket.accept()
                data = client.recv(4096)
                self.current_order = json.loads(data.decode())
                print("음성 주문 수신:", self.current_order)
                client.send(json.dumps({"status": "received"}).encode())
                client.close()
            except Exception as e:
                print(f"클라이언트 처리 오류: {e}")

# ======== 메뉴 페이지 ================
class MenuPage(QWidget):
    def __init__(self, menu_json, parent):
        super().__init__()
        self.parent = parent
        self.menu_json = menu_json
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 메뉴 리스트 위젯 생성 및 메뉴 데이터로 채우기
        self.listWidget = QListWidget()
        for name, info in self.menu_json['menu'].items():
            self.listWidget.addItem(f"{name} ({info.get('price', 0)}원)")
        
        # 옵션 콤보박스 생성 및 데이터 채우기
        self.sauceCombo = QComboBox()
        populate_combo(self.sauceCombo, self.menu_json['sauce'])
        
        self.vegCombo = QComboBox()
        populate_combo(self.vegCombo, self.menu_json['vegetable'])
        
        self.cheeseCombo = QComboBox()
        populate_combo(self.cheeseCombo, self.menu_json['cheese'])
        
        # 설명 레이블 및 버튼 생성
        self.descLabel = QLabel("메뉴와 옵션을 선택하세요.")
        self.addBtn = QPushButton("주문에 추가 (0건, 0원)")
        self.nextBtn = QPushButton("주문 요약으로 이동")
        
        # 레이아웃에 위젯 추가
        layout.addWidget(QLabel("샌드위치 메뉴 선택"))
        layout.addWidget(self.listWidget)
        layout.addWidget(QLabel("소스 선택"))
        layout.addWidget(self.sauceCombo)
        layout.addWidget(QLabel("야채 선택"))
        layout.addWidget(self.vegCombo)
        layout.addWidget(QLabel("치즈 선택"))
        layout.addWidget(self.cheeseCombo)
        layout.addWidget(self.descLabel)
        
        # 버튼을 위한 수평 레이아웃 생성 및 배치
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.addBtn)
        btn_layout.addWidget(self.nextBtn)
        layout.addLayout(btn_layout)
        
        # 시그널 연결 - 이벤트 핸들러 설정
        self.listWidget.currentItemChanged.connect(self.update_desc)
        self.sauceCombo.currentIndexChanged.connect(self.update_desc)
        self.vegCombo.currentIndexChanged.connect(self.update_desc)
        self.cheeseCombo.currentIndexChanged.connect(self.update_desc)
        self.addBtn.clicked.connect(self.add_order)
        self.nextBtn.clicked.connect(self.goto_summary)

    def update_desc(self, current=None, previous=None):
        """선택된 메뉴와 옵션에 따라 설명 업데이트"""
        item = self.listWidget.currentItem()
        if not item:
            return
            
        # 메뉴 이름과 기본 가격 추출
        menu_text = item.text()
        name = menu_text.split(' (')[0]
        base_price = int(menu_text.split('(')[1].replace('원)', ''))
        
        # 선택된 옵션 정보 가져오기
        sauce_name, sauce_price = self.sauceCombo.currentData()
        veg_name, veg_price = self.vegCombo.currentData()
        cheese_name, cheese_price = self.cheeseCombo.currentData()
        
        # 총 가격 계산 및 설명 업데이트
        total_price = base_price + sauce_price + veg_price + cheese_price
        self.descLabel.setText(
            f"{name}: {self.menu_json['menu'][name]['description']}\n"
            f"옵션: {sauce_name}, {veg_name}, {cheese_name} / 가격: {total_price}원"
        )

    def add_order(self):
        """선택된 메뉴와 옵션으로 주문 추가"""
        item = self.listWidget.currentItem()
        if not item:
            return
        
        # 메뉴 정보 및 옵션 정보 추출
        menu_text = item.text()
        name = menu_text.split(' (')[0]
        base_price = int(menu_text.split('(')[1].replace('원)', ''))
        sauce_name, sauce_price = self.sauceCombo.currentData()
        veg_name, veg_price = self.vegCombo.currentData()
        cheese_name, cheese_price = self.cheeseCombo.currentData()
        unit_price = base_price + sauce_price + veg_price + cheese_price
        
        # 기존 주문에 동일한 메뉴/옵션이 있는지 확인
        for order in self.parent.orders:
            if order['menu'] == name and order['options'] == [sauce_name, veg_name, cheese_name]:
                # 동일한 주문이 있으면 수량 증가
                order['quantity'] += 1
                order['price'] = order['unit_price'] * order['quantity']
                break
        else:
            # 새로운 주문 추가
            self.parent.orders.append({
                'menu': name,
                'options': [sauce_name, veg_name, cheese_name],
                'unit_price': unit_price,
                'quantity': 1,
                'price': unit_price
            })
        
        # 주문 버튼 텍스트 업데이트
        total_count = sum(o['quantity'] for o in self.parent.orders)
        total_sum = sum(o['price'] for o in self.parent.orders)
        self.addBtn.setText(f"주문에 추가 ({total_count}건, {total_sum}원)")

    def goto_summary(self):
        """주문 요약 페이지로 이동"""
        self.parent.summaryPage.show_summary()
        self.parent.setCurrentIndex(1)

# ============ 주문 요약 페이지 ================
class SummaryPage(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 주문 목록 위젯과 레이블, 버튼 생성
        self.orderList = QListWidget()
        self.totalLabel = QLabel("총 합계: 0원")
        self.payBtn = QPushButton("결제하기")
        
        # 레이아웃에 위젯 추가
        layout.addWidget(QLabel("주문 요약"))
        layout.addWidget(self.orderList)
        layout.addWidget(self.totalLabel)
        layout.addWidget(self.payBtn)
        
        # 결제 버튼 클릭 이벤트 연결
        self.payBtn.clicked.connect(self.go_payment)

    def show_summary(self):
        """주문 요약 정보 표시"""
        self.orderList.clear()
        total = 0
        
        # 주문 목록 표시
        for idx, order in enumerate(self.parent.orders, 1):
            opts = ", ".join(order['options'])
            line = f"{idx}. {order['menu']} x{order['quantity']} ({opts}) - {order['price']}원"
            self.orderList.addItem(line)
            total += order['price']
        
        # 총 금액 레이블 업데이트
        self.totalLabel.setText(f"총 합계: {total}원")

    def go_payment(self):
        """결제 페이지로 이동"""
        self.parent.receipt = ''.join(random.choices(string.digits, k=6))
        self.parent.paymentPage.update_payment()
        self.parent.setCurrentIndex(2)

# ============= 결제 페이지 ================
class PaymentPage(QWidget):
    payment_complete = pyqtSignal(dict)  # 결제 완료 시그널

    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 영수증 레이블과 버튼 생성
        self.receiptLabel = QLabel("영수증 번호: ")
        self.rfidBtn = QPushButton("RFID 결제")
        self.nextBtn = QPushButton("확인 화면")
        
        # 레이아웃에 위젯 추가
        layout.addWidget(QLabel("결제 방식 선택"))
        layout.addWidget(self.receiptLabel)
        layout.addWidget(self.rfidBtn)
        layout.addWidget(self.nextBtn)
        
        # 버튼 이벤트 연결
        self.rfidBtn.clicked.connect(self.process_payment)
        self.nextBtn.clicked.connect(self.confirm_payment)

    def update_payment(self):
        """영수증 번호 업데이트"""
        self.receiptLabel.setText(f"영수증 번호: {self.parent.receipt}")

    def process_payment(self):
        """결제 처리 (RFID/카드)"""
        self.receiptLabel.setText(f"영수증 번호: {self.parent.receipt} (결제 완료)")
        QMessageBox.information(self, "결제 성공", "결제가 성공적으로 완료되었습니다.")
        self.payment_complete.emit({"status": "paid"})

    def confirm_payment(self):
        """확인 화면으로 이동 및 주문 서버로 전송"""
        try:
            # 주문 데이터 생성 (시간, 영수증 번호 추가)
            order_data = {
                "orders": self.parent.orders,
                "total": sum(order['price'] for order in self.parent.orders),
                "receipt": self.parent.receipt,
                "timestamp": datetime.now().isoformat(),
                "table": 1  # 테이블 번호 (필요시 동적으로 설정)
            }
            
            # 메인 서버로 주문 데이터 전송
            result = send_order_to_server(order_data)
            if result.get('status') == 'fail':
                QMessageBox.warning(self, "서버 오류", "주문 저장 중 오류가 발생했습니다.")
            
            # 확인 화면으로 이동
            self.parent.setCurrentIndex(3)
        except Exception as e:
            QMessageBox.critical(self, "오류", f"처리 중 오류 발생: {str(e)}")

# ============== 주문 확인 페이지 =================
class ConfirmationPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("주문이 완료되었습니다!"))
        
        # 메인 화면으로 돌아가는 버튼
        self.homeBtn = QPushButton("메인 화면으로")
        layout.addWidget(self.homeBtn)

# ============== 음성 주문 메인 애플리케이션 =================
class KioskApp(QMainWindow):
    """메인 키오스크 애플리케이션"""
    payment_complete = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("서보웨이 AI 키오스크")
        self.setGeometry(100, 100, 1024, 768)
        
        # Streamlit 프로세스 관리
        self.streamlit_proc = None
        self.server = KioskServer()
        self.init_ui()
        self.start_services()

    def init_ui(self):
        """UI 초기화"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # 음성 주문 버튼
        self.voice_btn = QPushButton("🎤 음성 주문 시작")
        self.voice_btn.setFixedHeight(60)
        layout.addWidget(self.voice_btn)
        
        # 웹뷰 (Streamlit 표시)
        self.webview = QWebEngineView()
        layout.addWidget(self.webview, 1)
        
        # 시그널 연결
        self.voice_btn.clicked.connect(self.start_voice_order)
        self.payment_complete.connect(self.handle_payment_result)

    def start_services(self):
        """필요한 서비스 시작"""
        self.server.start()
        import threading
        threading.Thread(target=self.server.handle_connection, daemon=True).start()

    def start_voice_order(self):
        """음성 주문 시작"""
        if self.check_streamlit_running():
            self.show_streamlit()
            return
        
        # Streamlit 서버 시작
        self.streamlit_proc = subprocess.Popen(
            [sys.executable, "-m", "streamlit", "run",
            STREAMLIT_SCRIPT,
            "--server.port", str(STREAMLIT_PORT),
            "--server.headless", "true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        # 웹뷰 로드
        QTimer.singleShot(3000, self.show_streamlit)

    def show_streamlit(self):
        """웹뷰에 Streamlit 페이지 로드"""
        self.webview.load(QUrl(f"http://localhost:{STREAMLIT_PORT}"))

    def check_streamlit_running(self):
        """Streamlit 서버 실행 여부 확인"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(("localhost", STREAMLIT_PORT))
            sock.close()
            return True
        except:
            return False

    def handle_payment_result(self, result):
        """결제 결과 처리"""
        if result.get('status') == 'paid':
            self.save_order()
            self.show_confirmation()

    def save_order(self):
        """주문 데이터 저장"""
        try:
            order_data = {
                "orders": self.server.current_order,
                "timestamp": datetime.now().isoformat(),
                "receipt": ''.join(random.choices(string.digits, k=8))
            }
            filename = f"order_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(order_data, f, ensure_ascii=False, indent=2)
            print(f"주문 데이터 로컬 저장 완료: {filename}")
        except Exception as e:
            print(f"주문 저장 오류: {e}")

    def show_confirmation(self):
        """확인 페이지 표시"""
        QMessageBox.information(self, "주문 완료", "음성 주문이 정상적으로 처리되었습니다!")

    def closeEvent(self, event):
        """창 종료 시 리소스 정리"""
        try:
            if self.streamlit_proc:
                self.streamlit_proc.terminate()
            if self.server.socket:
                self.server.running = False
                self.server.socket.close()
        except Exception as e:
            print(f"종료 처리 오류: {e}")
        super().closeEvent(event)

# =============== 기존 주문 앱 ===============
class OrderApp(QStackedWidget):
    """기존 주문 시스템"""
    def __init__(self):
        super().__init__()
        self.orders = []  # 주문 목록을 저장할 리스트
        self.receipt = None  # 영수증 번호
        
        # 서버 또는 로컬에서 메뉴 데이터 가져오기
        menu_json = get_menu_json()
        
        # 각 페이지 생성
        self.menuPage = MenuPage(menu_json, self)
        self.summaryPage = SummaryPage(self)
        self.paymentPage = PaymentPage(self)
        self.confirmPage = ConfirmationPage()
        
        # 결제 완료 시그널 연결
        self.paymentPage.payment_complete.connect(self._handle_payment_complete)
        
        # 확인 페이지 홈버튼 연결
        self.confirmPage.homeBtn.clicked.connect(lambda: self.setCurrentIndex(0))
        
        # 스택 위젯에 페이지 추가
        for page in [self.menuPage, self.summaryPage, self.paymentPage, self.confirmPage]:
            self.addWidget(page)
    
    def _handle_payment_complete(self, result):
        """결제 완료 처리"""
        if result.get('status') == 'paid':
            # 여기서 주문 정보를 메인 서버로 전송
            try:
                order_data = {
                    "orders": self.orders,
                    "total": sum(order['price'] for order in self.orders),
                    "receipt": self.receipt,
                    "timestamp": datetime.now().isoformat(),
                    "table": 1  # 테이블 번호
                }
                send_order_to_server(order_data)
            except Exception as e:
                print(f"주문 전송 오류: {e}")

# ============= 애플리케이션 실행 부분 ==============
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 상황에 따라 적절한 앱을 선택
    if len(sys.argv) > 1 and sys.argv[1] == "--voice":
        # 음성 주문 모드로 실행
        win = KioskApp()
    else:
        # 일반 키오스크 모드로 실행
        win = OrderApp()
    
    win.setWindowTitle("서보웨이 주문 시스템")
    win.resize(800, 600)
    win.show()
    
    sys.exit(app.exec_())
