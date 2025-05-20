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
# import pymysql  # MySQL 데이터베이스 연결
import requests  # HTTP 요청 처리(메인 서버 API 호출)

# ============== PyQt 모듈 ==================
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget, QPushButton,
    QListWidget, QMessageBox, QLabel
)
from PyQt5 import uic
from PyQt5.QtGui import QIcon
import requests
from PyQt5.QtCore import QTimer, pyqtSignal, QUrl, Qt, QSize  # 이벤트 루프, 시그널, 타이머
# 웹 엔진
from PyQt5.QtWebEngineWidgets import QWebEngineView  # 웹뷰(Streamlit 표시용)

# ============== 메인 서버 설정 ================
MENU_SERVER_URL = "http://192.168.0.145:5003/"  # 메뉴 정보 API 주소
ORDER_SERVER_URL = "http://192.168.0.145:5003/"  # 주문 전송 API 주소
# MENU_SERVER_URL = "http://192.168.0.178:5003"  # 메뉴 정보 API 주소
# ORDER_SERVER_URL = "http://192.168.0.178:5003"  # 주문 전송 API 주소

# ============ Streamlit 설정 =============
STREAMLIT_PORT = 8502  # Streamlit 서버 포트
STREAMLIT_SCRIPT = "voice_agent.py"  # 음성 에이전트 스크립트 경로
TABLE_NUM = 1


# DB에서 최신 메뉴 JSON 불러오기
# def get_menu_json():
#     conn = pymysql.connect(
#         host="localhost", user="root", password="1",
#         db="serbobase", charset="utf8mb4"
#     )
#     with conn.cursor() as cursor:
#         cursor.execute("SELECT json_data FROM menu_json ORDER BY id DESC LIMIT 1")
#         row = cursor.fetchone()
#     conn.close()
#     return json.loads(row[0]) if row else None

# ========= 메인 서버와 통신 ============
def send_order_to_server(order_data):
    """주문 정보를 메인 서버로 전송"""
    try:
        response = requests.post(ORDER_SERVER_URL, json=order_data)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"주문 서버 연결 실패: {e}")
        return {"status": "fail", "message": str(e)}

# def send_json_file_to_server(file_path):
#     """json 파일에서 주문 내역을 읽어 서버에 전송 (데이터만 전송)"""
#     try:
#         with open(file_path, 'r', encoding='utf-8') as f:
#             order_data = json.load(f)  # 파일에서 딕셔너리로 파싱

#         response = requests.post(ORDER_SERVER_URL, json=order_data)  # 데이터만 전송
#         response.raise_for_status()
#         return response.json()
#     except Exception as e:
#         print(f"주문 서버 연결 실패: {e}")
#         return {"status": "fail", "message": str(e)}

def get_menu_json(server_url=MENU_SERVER_URL, local_file="menu.json"):
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

# ============ Kiosk server 실행 ====================
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

# ==================

class SerbowayApp(QMainWindow):
    """메인 키오스크 애플리케이션"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serboway Kiosk")
        self.setGeometry(200, 200, 600, 500)

        # JSON 메뉴 로드
        self.menu_json = get_menu_json()
        self.order_data = {'menu': []}
        self.current_sandwich = None
        self.selected_sauce = None
        self.selected_vegetable = None
        self.selected_cheese = None

        # 스택 위젯 설정
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # UI 페이지 로드
        self.page0 = uic.loadUi("UI/1_choose_ordermethod.ui")
        self.page1 = uic.loadUi("UI/2_choose_sandwich.ui")
        self.page2 = uic.loadUi("UI/3_choose_sauce.ui")
        self.page3 = uic.loadUi("UI/4_choose_vegetables.ui")
        self.page4 = uic.loadUi("UI/5_choose_cheese.ui")
        self.page5 = uic.loadUi("UI/6_confirm_order.ui")
        self.page6 = uic.loadUi("UI/7_choose_paymentmethod.ui")
        self.page7 = uic.loadUi("UI/8_order_complete.ui")
        for page in [self.page0, self.page1, self.page2, self.page3,
                     self.page4, self.page5, self.page6, self.page7]:
            self.stack.addWidget(page)

        # 버튼 연결 및 동적 매핑 설정
        self.connect_buttons()
        self.populate_dynamic_buttons()

    def connect_buttons(self):
        def btn(page, name):
            return page.findChild(QPushButton, name, Qt.FindChildrenRecursively)

        # 첫 페이지: 주문 방식 선택
        btn(self.page0, "voiceButton").clicked.connect(lambda: self.stack.setCurrentIndex(1))
        btn(self.page0, "touchButton").clicked.connect(lambda: self.stack.setCurrentIndex(1))

        # 결제/재시작/완료 버튼
        btn(self.page5, "confirmButton").clicked.connect(self.go_to_payment)
        btn(self.page5, "homeButton").clicked.connect(self.restart_order)
        btn(self.page6, "pushButton").clicked.connect(self.complete_order)

    def populate_dynamic_buttons(self):
        # -------------------------------
        # 샌드위치 버튼: (메뉴키, 이미지경로) 매핑
        # -------------------------------
        sandwich_map = {
            'BulgogiBtn': ('불고기 샌드위치', 'Menu.png'),
            'ShrimpBtn':  ('새우 샌드위치',  'Menu.png'),
            'BaconBtn':   ('베이컨 샌드위치','Menu.png')
        }

        for obj_name, (menu_key, img_path) in sandwich_map.items():
            btn = self.page1.findChild(QPushButton, obj_name, Qt.FindChildrenRecursively)
            # 버튼이 없거나 JSON에 메뉴키가 없으면 건너뛰기
            if not btn or menu_key not in self.menu_json.get('menu', {}):
                continue

            # 배경 이미지 제거
            btn.setStyleSheet("background-image: none;")

            # 아이콘 설정
            btn.setIcon(QIcon(self.menu_json['menu'][menu_key]['image']))
            btn.setIconSize(QSize(128, 128))

            # 텍스트 설정 (메뉴명 + 가격)
            price = self.menu_json['menu'][menu_key]['price']
            # btn.setText(f"{menu_key}\n({price}원)")
            
            print(menu_key)

            # 클릭 시 select_sandwich(menu_key) 호출
            try:
                btn.clicked.disconnect()
            except TypeError:
                pass
            btn.clicked.connect(lambda _, m=menu_key: self.select_sandwich(m))

        # -------------------------------
        # 소스 버튼 매핑 (텍스트만)
        # -------------------------------
        sauce_map = {'Italian': '이탈리안', 'Chilly': '칠리'}
        for obj_name, sauce_key in sauce_map.items():
            btn = self.page2.findChild(QPushButton, obj_name, Qt.FindChildrenRecursively)
            
            if not btn or sauce_key not in self.menu_json.get('sauce', {}):
                continue
            # if btn and sauce_key in self.menu_json.get('sauce', {}):
            #     continue

            price = self.menu_json['sauce'][sauce_key]['price']
            # btn.setIcon(f"{sauce_key}\n(+{price}원)")
            # btn.setIconSize(QSize(128, 128))
            btn.setText(f"{sauce_key}\n(+{price}원)")
            try:
                btn.clicked.disconnect()
            except TypeError:
                pass
            btn.clicked.connect(lambda _, s=sauce_key: self.select_sauce(s))

        # -------------------------------
        # 야채 버튼 매핑
        # -------------------------------
        veg_map = {'Lettuce': '양상추', 'Romaine': '로메인', 'Bazil': '바질'}
        for obj_name, veg_key in veg_map.items():
            btn = self.page3.findChild(QPushButton, obj_name, Qt.FindChildrenRecursively)
            if btn and veg_key in self.menu_json.get('vegetable', {}):
                price = self.menu_json['vegetable'][veg_key].get('price', 0)
                btn.setText(f"{veg_key}\n(+{price}원)")
                try:
                    btn.clicked.disconnect()
                except TypeError:
                    pass
                btn.clicked.connect(lambda _, v=veg_key: self.select_vegetable(v))

        # -------------------------------
        # 치즈 버튼 매핑
        # -------------------------------
        cheese_map = {
            'Slice':      '슬라이스 치즈',
            'Shred':      '슈레드 치즈',
            'Mozzarella': '모짜렐라 치즈'
        }
        for obj_name, cheese_key in cheese_map.items():
            btn = self.page4.findChild(QPushButton, obj_name, Qt.FindChildrenRecursively)
            if btn and cheese_key in self.menu_json.get('cheese', {}):
                price = self.menu_json['cheese'][cheese_key].get('price', 0)
                btn.setText(f"{cheese_key}\n(+{price}원)")
                try:
                    btn.clicked.disconnect()
                except TypeError:
                    pass
                btn.clicked.connect(lambda _, c=cheese_key: self.select_cheese(c))

        # 주문 리스트 위젯 참조
        self.order_list_widget = self.page5.findChild(
            QListWidget, "listWidget", Qt.FindChildrenRecursively
        )

    def select_sandwich(self, name):
        self.current_sandwich = name
        self.stack.setCurrentIndex(2)

    def select_sauce(self, sauce):
        self.selected_sauce = sauce
        self.stack.setCurrentIndex(3)

    def select_vegetable(self, veg):
        self.selected_vegetable = veg
        self.stack.setCurrentIndex(4)

    def select_cheese(self, cheese):
        self.selected_cheese = cheese
        self.save_order_item()
        self.stack.setCurrentIndex(5)
        self.update_order_list()

    def save_order_item(self):
        base_price = self.menu_json['menu'][self.current_sandwich]['price']
        opt_price = (
            self.menu_json['sauce'][self.selected_sauce].get('price', 0) +
            self.menu_json['vegetable'][self.selected_vegetable].get('price', 0) +
            self.menu_json['cheese'][self.selected_cheese].get('price', 0)
        )
        unit_price = base_price + opt_price
        self.order_data['menu'].append({
            'name': self.current_sandwich,
            'price': unit_price,
            'qty': 1,
            'sauce': self.selected_sauce,
            'vegetable': self.selected_vegetable,
            'cheese': self.selected_cheese
        })

        self.send_order_data={
            "table_number": TABLE_NUM,
            "sandwich": self.current_sandwich,
            "sauce": self.selected_sauce,
            "vegetable": self.selected_vegetable,  # Clean name without price
            "cheese": self.selected_cheese,        # Clean name without price
            "price": unit_price
        }

    def update_order_list(self):
        self.order_list_widget.clear()
        total = 0
        for item in self.order_data['menu']:
            text = (
                f"{item['name']} ({item['sauce']}/{item['vegetable']}/{item['cheese']}) "
                f"x{item['qty']} - {item['price']}원"
            )
            self.order_list_widget.addItem(text)
            total += item['price'] * item['qty']
        lbl = self.page5.findChild(QLabel, "summaryLabel", Qt.FindChildrenRecursively)
        if lbl:
            lbl.setText(f"총 합계: {total}원")

    def go_to_payment(self):
        if not self.order_data['menu']:
            QMessageBox.warning(self, "경고", "주문 내역이 없습니다.")
            return
        self.stack.setCurrentIndex(6)

    def complete_order(self):
        print("최종 주문:", self.order_data)
        
        # 타임스탬프 추가
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        
        # 주문 데이터에 타임스탬프 추가
        order_with_time = self.order_data.copy()
        order_with_time['timestamp'] = timestamp
        
        # JSON 파일로 저장
        order_filename = f"order_{timestamp}.json"
        with open(order_filename, 'w', encoding='utf-8') as f:
            json.dump(order_with_time, f, ensure_ascii=False, indent=4)
        
        print(f"주문 내역이 {order_filename}에 저장되었습니다.")
        
        result = send_order_to_server(self.send_order_data)
        if result.get('status') == 'fail':
            QMessageBox.warning(self, "서버 오류", "주문 저장 중 오류가 발생했습니다.")
        
        self.stack.setCurrentIndex(7)

    def restart_order(self):
        self.order_data = {'menu': []}
        self.stack.setCurrentIndex(1)
        
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
        self.layout.addWidget(self.voice_btn)
        
        # 웹뷰 (Streamlit 표시)
        self.webview = QWebEngineView()
        self.layout.addWidget(self.webview, 1)
        
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerbowayApp()
    window.show()
    sys.exit(app.exec_())
