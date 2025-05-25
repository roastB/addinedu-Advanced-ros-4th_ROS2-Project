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

import requests  # HTTP 요청 처리(메인 서버 API 호출)
import webbrowser  # [Streamlit 연동 추가] 웹브라우저 띄우기
import threading
from PyQt5.QtCore import QLoggingCategory

# ============== PyQt 모듈 ==================
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QStackedWidget,
    QPushButton,
    QListWidget,
    QMessageBox,
    QLabel,
    QTextEdit,
)
from PyQt5 import uic
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QSize

from PyQt5.QtCore import QEvent


class VoiceOrderEvent(QEvent):
    def __init__(self):
        super().__init__(QEvent.User)


# ============== 메인 서버 설정 ================
# ORDER_SERVER_URL = "http://192.168.0.6:5003/"  # 주문 전송 API 주소
ORDER_SERVER_URL = "http://192.168.55.177:5003"

# ============ Streamlit 설정 =============
STREAMLIT_PORT = 8501  # Streamlit 서버 포트
STREAMLIT_SCRIPT = "Serboway_voice_order.py"  # 음성 에이전트 스크립트 경로

# Table Number 고정
TABLE_NUM = 1


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


def get_menu_json(server_url=ORDER_SERVER_URL, local_file="menu.json"):
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


class SerbowayApp(QMainWindow):
    """메인 키오스크 애플리케이션"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serboway Kiosk")
        self.setGeometry(200, 200, 600, 500)

        self.voice_order_data = None

        self.tcp_server_thread = threading.Thread(
            target=self.run_tcp_server, daemon=True
        )
        self.tcp_server_thread.start()

        # JSON 메뉴 로드
        self.menu_json = get_menu_json()
        self.order_data = {"menu": []}
        self.current_sandwich = None
        self.selected_sauce = None
        self.selected_vegetable = None
        self.selected_cheese = None

        # [Streamlit 연동 추가] Streamlit 프로세스 핸들 저장
        self.streamlit_proc = None

        # 스택 위젯 설정
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # UI 페이지 로드
        self.page0 = uic.loadUi("kiosk_UI/1_choose_ordermethod.ui")
        self.page1 = uic.loadUi("kiosk_UI/2_choose_sandwich.ui")
        self.page2 = uic.loadUi("kiosk_UI/3_choose_sauce.ui")
        self.page3 = uic.loadUi("kiosk_UI/4_choose_vegetables.ui")
        self.page4 = uic.loadUi("kiosk_UI/5_choose_cheese.ui")
        self.page5 = uic.loadUi("kiosk_UI/6_confirm_order.ui")
        self.page6 = uic.loadUi("kiosk_UI/7_choose_paymentmethod.ui")
        self.page7 = uic.loadUi("kiosk_UI/8_order_complete.ui")
        self.page8 = uic.loadUi("kiosk_UI/9_pick_up.ui")  # 픽업 페이지 추가
        self.page9 = uic.loadUi(
            "kiosk_UI/10_request_collect.ui"
        )  # 수거 요청 페이지 추가
        self.page10 = uic.loadUi("kiosk_UI/11_collect_done.ui")  # 수거 완료 페이지 추가
        self.page11 = uic.loadUi("kiosk_UI/12_rfid_charge_x.ui")  # rfid 페이지 추가

        for page in [
            self.page0,
            self.page1,
            self.page2,
            self.page3,
            self.page4,
            self.page5,
            self.page6,
            self.page7,
            self.page8,
            self.page9,
            self.page10,
            self.page11,
        ]:
            self.stack.addWidget(page)

        # 버튼 연결 및 동적 매핑 설정
        self.connect_buttons()
        self.populate_dynamic_buttons()

    def connect_buttons(self):
        def btn(page, name):
            button = page.findChild(
                QPushButton, name, Qt.FindChildrenRecursively
            )  # 재귀적으로 버튼 찾기
            if not button:  # 버튼을 찾지 못했으면
                print(
                    f"⚠️ 버튼 누락: {name} 버튼을 찾을 수 없습니다."
                )  # 경고 메시지 출력
            return button  # 버튼 객체 반환 (없으면 None)

        # ========== 첫 페이지: 주문 방식 선택 버튼 수정 ==========
        # UI 파일의 실제 버튼명에 맞춰 수정 필요 (voiceButton → voiceBtn, touchButton → touchBtn)
        voice_btn = btn(self.page0, "voiceBtn")  # 음성 주문 버튼 찾기
        touch_btn = btn(self.page0, "touchBtn")  # 터치 주문 버튼 찾기

        if voice_btn:  # 음성 주문 버튼이 존재하면
            voice_btn.clicked.connect(
                self.launch_streamlit_voice_order
            )  # Streamlit 음성 주문 실행 함수와 연결
        if touch_btn:  # 터치 주문 버튼이 존재하면
            touch_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(1)
            )  # 샌드위치 선택 페이지로 이동

        # ========== 주문 확인 페이지 버튼들 ==========
        add_btn = btn(self.page5, "addorderBtn")  # 추가 주문 버튼
        restart_btn = btn(self.page5, "restartBtn")  # 주문 재시작 버튼
        pay_btn = btn(self.page5, "payBtn")  # 결제 버튼

        if add_btn:  # 추가 주문 버튼이 존재하면
            add_btn.clicked.connect(self.add_order)  # 추가 주문 함수와 연결
        if restart_btn:  # 재시작 버튼이 존재하면
            restart_btn.clicked.connect(self.restart_order)  # 주문 재시작 함수와 연결
        if pay_btn:  # 결제 버튼이 존재하면
            pay_btn.clicked.connect(self.go_to_payment)  # 결제 페이지 이동 함수와 연결

        # ========== 결제 페이지 버튼 ==========
        rfid_btn = btn(self.page6, "rfidBtn")  # RFID 결제 버튼
        if rfid_btn:  # RFID 버튼이 존재하면
            rfid_btn.clicked.connect(self.complete_order)  # 주문 완료 함수와 연결

        # ========== 나머지 페이지 버튼들 (기존 코드 유지) ==========
        # 8_order_complete.ui - OK 버튼
        ok_btn = btn(self.page7, "okBtn")
        if ok_btn:
            ok_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(8)
            )  # 픽업 페이지로 이동

        # 9_pick_up.ui - Pick Up Done 버튼
        pickup_btn = btn(self.page8, "pickupBtn")
        if pickup_btn:
            pickup_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(9)
            )  # 수거 요청 페이지로 이동

        # 10_request_collect.ui - Request Collect 버튼
        reqcol_btn = btn(self.page9, "reqcolBtn")
        if reqcol_btn:
            reqcol_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(10)
            )  # 수거 완료 페이지로 이동

        # 11_collect_done.ui - Collect Done 버튼
        coldone_btn = btn(self.page10, "colDone")
        if coldone_btn:
            coldone_btn.clicked.connect(
                lambda: self.stack.setCurrentIndex(0)
            )  # 메인 페이지로 돌아가기

    # [Streamlit 연동 추가] 음성 주문 버튼 클릭 시 Streamlit 서버 실행 및 브라우저 접속
    def launch_streamlit_voice_order(self):
        """
        음성 주문 버튼 클릭 시 Streamlit 서버를 실행하고 브라우저로 접속합니다.
        이미 실행 중이면 새로 실행하지 않습니다.
        """
        # Streamlit 서버가 이미 실행 중인지 확인
        if self.streamlit_proc is None or self.streamlit_proc.poll() is not None:
            # Streamlit 서버 실행 (subprocess)
            self.streamlit_proc = subprocess.Popen(
                [
                    "streamlit",
                    "run",
                    STREAMLIT_SCRIPT,
                    "--server.headless=true",
                    f"--server.port={STREAMLIT_PORT}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # 웹브라우저로 Streamlit UI 오픈
            webbrowser.open(f"http://localhost:{STREAMLIT_PORT}")

    def populate_dynamic_buttons(self):
        # ========== 샌드위치 버튼 매핑 수정 ==========
        sandwich_map = {
            # UI 파일의 실제 버튼 객체명과 일치시킴
            "BulgogiBtn": (
                "불고기 샌드위치",
                "image/Bulgogi141.png",
            ),  # 절대 경로로 수정
            "ShrimpBtn": ("새우 샌드위치", "image/Shrimp141.png"),  # 절대 경로로 수정
            "BaconBtn": ("베이컨 샌드위치", "image/Bacon141.png"),  # 절대 경로로 수정
        }

        for obj_name, (menu_key, img_path) in sandwich_map.items():
            btn = self.page1.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            )  # UI에서 버튼 찾기
            if not btn or menu_key not in self.menu_json.get(
                "menu", {}
            ):  # 버튼이 없거나 메뉴가 JSON에 없으면 스킵
                continue

            # 배경 이미지 제거 + 텍스트 정렬 (UI 파일의 배경 이미지를 제거하고 아이콘으로 대체)
            btn.setStyleSheet(
                """
                QPushButton {
                    background-image: none;  /* UI 파일의 기존 배경 이미지 제거 */
                    text-align: center;      /* 텍스트 중앙 정렬 */
                }
            """
            )

            # 아이콘 설정 (이미지 파일 존재 여부 확인 후 설정)
            if os.path.exists(img_path):  # 이미지 파일이 존재하면
                btn.setIcon(QIcon(img_path))  # 해당 이미지를 아이콘으로 설정
            else:  # 이미지 파일이 없으면
                btn.setIcon(
                    QIcon(self.menu_json["menu"][menu_key].get("image", img_path))
                )  # JSON에서 이미지 경로 가져오기
            btn.setIconSize(QSize(128, 128))  # 아이콘 크기를 128x128로 설정

            # 클릭 이벤트 연결 (기존 연결 해제 후 새로 연결)
            try:
                btn.clicked.disconnect()  # 기존에 연결된 시그널 해제
            except TypeError:  # 연결된 시그널이 없으면 TypeError 발생하므로 무시
                pass
            btn.clicked.connect(
                lambda _, m=menu_key: self.select_sandwich(m)
            )  # 샌드위치 선택 함수와 연결

        # ========== 소스 버튼 매핑 수정 (Btn 접미사 추가) ==========
        sauce_map = {
            "ItalianBtn": "이탈리안",
            "ChillyBtn": "칠리",
        }  # UI 파일의 실제 객체명에 맞춤
        for obj_name, sauce_key in sauce_map.items():
            btn = self.page2.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            )  # 소스 페이지에서 버튼 찾기
            if not btn or sauce_key not in self.menu_json.get(
                "sauce", {}
            ):  # 버튼이 없거나 소스가 JSON에 없으면 스킵
                continue
            price = self.menu_json["sauce"][sauce_key][
                "price"
            ]  # JSON에서 소스 가격 가져오기
            btn.setText(
                f"{sauce_key}\n(+{price}원)"
            )  # 버튼 텍스트를 "소스명\n(+가격원)" 형태로 설정
            try:
                btn.clicked.disconnect()  # 기존 연결 해제
            except TypeError:
                pass
            btn.clicked.connect(
                lambda _, s=sauce_key: self.select_sauce(s)
            )  # 소스 선택 함수와 연결

        # ========== 야채 버튼 매핑 수정 (Btn 접미사 추가) ==========
        veg_map = {
            "LettuceBtn": "양상추",
            "RomaineBtn": "로메인",
            "BazilBtn": "바질",
        }  # UI 파일의 실제 객체명에 맞춤
        for obj_name, veg_key in veg_map.items():
            btn = self.page3.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            )  # 야채 페이지에서 버튼 찾기
            if btn and veg_key in self.menu_json.get(
                "vegetable", {}
            ):  # 버튼이 있고 야채가 JSON에 있으면
                price = self.menu_json["vegetable"][veg_key].get(
                    "price", 0
                )  # JSON에서 야채 가격 가져오기 (없으면 0)
                btn.setText(f"{veg_key}\n(+{price}원)")  # 버튼 텍스트 설정
                try:
                    btn.clicked.disconnect()  # 기존 연결 해제
                except TypeError:
                    pass
                btn.clicked.connect(
                    lambda _, v=veg_key: self.select_vegetable(v)
                )  # 야채 선택 함수와 연결

        # ========== 치즈 버튼 매핑 수정 (Btn 접미사 추가) ==========
        cheese_map = {
            "SliceBtn": "슬라이스 치즈",  # UI 파일의 실제 객체명에 맞춤
            "ShredBtn": "슈레드 치즈",  # UI 파일의 실제 객체명에 맞춤
            "MozzarellaBtn": "모짜렐라 치즈",  # UI 파일의 실제 객체명에 맞춤
        }
        for obj_name, cheese_key in cheese_map.items():
            btn = self.page4.findChild(
                QPushButton, obj_name, Qt.FindChildrenRecursively
            )  # 치즈 페이지에서 버튼 찾기
            if btn and cheese_key in self.menu_json.get(
                "cheese", {}
            ):  # 버튼이 있고 치즈가 JSON에 있으면
                price = self.menu_json["cheese"][cheese_key].get(
                    "price", 0
                )  # JSON에서 치즈 가격 가져오기 (없으면 0)
                btn.setText(f"{cheese_key}\n(+{price}원)")  # 버튼 텍스트 설정
                try:
                    btn.clicked.disconnect()  # 기존 연결 해제
                except TypeError:
                    pass
                btn.clicked.connect(
                    lambda _, c=cheese_key: self.select_cheese(c)
                )  # 치즈 선택 함수와 연결

        # ========== 주문 리스트 위젯 참조 수정 ==========
        # QTextEdit에서 QListWidget으로 변경 (pyqt 파일과 동일하게)
        self.order_list_widget = self.page5.findChild(
            QTextEdit, "orderList", Qt.FindChildrenRecursively
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

    # 사용자가 샌드위치, 소스, 야채, 치즈를 선택한 뒤 해당 선택을 목록에 추가
    def save_order_item(self):
        base_price = self.menu_json["menu"][self.current_sandwich]["price"]
        opt_price = (
            self.menu_json["sauce"][self.selected_sauce].get("price", 0)
            + self.menu_json["vegetable"][self.selected_vegetable].get("price", 0)
            + self.menu_json["cheese"][self.selected_cheese].get("price", 0)
        )
        unit_price = base_price + opt_price
        self.order_data["menu"].append(
            {
                "name": self.current_sandwich,
                "price": unit_price,
                "qty": 1,
                "sauce": self.selected_sauce,
                "vegetable": self.selected_vegetable,
                "cheese": self.selected_cheese,
            }
        )
        self.send_order_data = {
            "table_number": TABLE_NUM,
            "sandwich": self.current_sandwich,
            "sauce": self.selected_sauce,
            "vegetable": self.selected_vegetable,
            "cheese": self.selected_cheese,
            "price": unit_price,
        }

    # 주문 내역을 UI에 표시 하고 총 금액을 계산해서 보여준다.
    def update_order_list(self):
        """주문 내역을 UI에 표시하고 총 금액을 계산해서 보여주는 함수"""
        # QListWidget 방식 (pyqt 파일과 동일)
        if (
            hasattr(self, "order_list_widget") and self.order_list_widget
        ):  # order_list_widget이 존재하면
            self.order_list_widget.clear()  # 기존 목록 지우기
            total = 0  # 총 금액 초기화

            for item in self.order_data["menu"]:  # 주문 데이터의 각 메뉴 아이템에 대해
                # 주문 아이템 텍스트 생성 (메뉴명, 옵션들, 수량, 가격)
                text = (
                    f"{item['name']} ({item['sauce']}/{item['vegetable']}/{item['cheese']}) "
                    f"x{item['qty']} - {item['price']}원"
                )
                # self.order_list_widget.addItem(text)  # 리스트 위젯에 아이템 추가
                # QTextEdit에는 addItem()이 없으므로 setPlainText 또는 append 사용
                self.order_list_widget.append(text)
                total += item["price"] * item["qty"]  # 총 금액에 (가격 × 수량) 추가

            # 총 합계 라벨 찾아서 업데이트
            lbl = self.page5.findChild(
                QLabel, "summaryLabel", Qt.FindChildrenRecursively
            )  # 총합계 라벨 찾기
            if lbl:  # 라벨이 존재하면
                lbl.setText(f"총 합계: {total}원")  # 총 합계 텍스트 설정

    # 결제 화면으로 이동
    def go_to_payment(self):
        if not self.order_data["menu"]:
            QMessageBox.warning(self, "경고", "주문 내역이 없습니다.")
            return
        self.stack.setCurrentIndex(6)

    def complete_order(self):
        print("최종 주문:", self.order_data)
        # 타임스탬프 추가
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        # 주문 데이터에 타임스탬프 추가
        order_with_time = self.order_data.copy()
        order_with_time["timestamp"] = timestamp
        # JSON 파일로 저장
        order_filename = f"order_{timestamp}.json"
        with open(order_filename, "w", encoding="utf-8") as f:
            json.dump(order_with_time, f, ensure_ascii=False, indent=4)
        print(f"주문 내역이 {order_filename}에 저장되었습니다.")
        result = send_order_to_server(self.send_order_data)
        if result.get("status") == "fail":
            QMessageBox.warning(self, "서버 오류", "주문 저장 중 오류가 발생했습니다.")
        self.stack.setCurrentIndex(7)

    def restart_order(self):
        self.order_data = {"menu": []}
        self.stack.setCurrentIndex(1)

    def add_order(self):
        """추가 주문을 위해 샌드위치 선택 페이지로 이동하는 함수"""
        # 현재 선택사항 초기화 (새로운 주문을 위해)
        self.current_sandwich = None  # 선택된 샌드위치 초기화
        self.selected_sauce = None  # 선택된 소스 초기화
        self.selected_vegetable = None  # 선택된 야채 초기화
        self.selected_cheese = None  # 선택된 치즈 초기화

        # 샌드위치 선택 페이지(page1)로 이동
        self.stack.setCurrentIndex(1)  # 스택 위젯의 인덱스 1번 페이지로 이동

    def show_webview(self):
        self.stack.addWidget(self.webview)
        self.stack.setCurrentWidget(self.webview)

    def handle_payment_result(self, result):
        """결제 결과 처리"""
        if result.get("status") == "paid":
            self.save_order()
            self.show_confirmation()

    def save_order(self):
        """주문 데이터 저장"""
        try:
            order_data = {
                "orders": self.server.current_order,
                "timestamp": datetime.now().isoformat(),
                "receipt": "".join(random.choices(string.digits, k=8)),
            }
            filename = f"order_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(order_data, f, ensure_ascii=False, indent=2)
            print(f"주문 데이터 로컬 저장 완료: {filename}")
        except Exception as e:
            print(f"주문 저장 오류: {e}")

    def show_confirmation(self):
        """확인 페이지 표시"""
        QMessageBox.information(
            self, "주문 완료", "음성 주문이 정상적으로 처리되었습니다!"
        )

    def closeEvent(self, event):
        """창 종료 시 리소스 정리"""
        try:
            # [Streamlit 연동 추가] Streamlit 프로세스 종료
            if self.streamlit_proc:
                self.streamlit_proc.terminate()
            if hasattr(self, "server") and self.server.socket:
                self.server.running = False
                self.server.socket.close()
        except Exception as e:
            print(f"종료 처리 오류: {e}")
        super().closeEvent(event)

    def run_tcp_server(self):
        # while true:
        #     print("Server!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # KIOSK_HOST = "192.168.0.55"
        KIOSK_HOST = "192.168.55.177"
        # KIOSK_PORT = 650
        KIOSK_PORT = 5050

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((KIOSK_HOST, KIOSK_PORT))
            s.listen(1)
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(4096)
                    if data:
                        try:
                            order_json = json.loads(data.decode())
                            self.voice_order_data = order_json
                            conn.sendall(b"received")
                            # 주문 UI 반영 (예시)
                            QApplication.postEvent(self, VoiceOrderEvent())
                        except Exception:
                            conn.sendall(b"fail")

    def process_voice_order(self):
        # 디버깅 확인 용
        print("음성 주문 처리 시작")
        print("수신된 주문:", self.voice_order_data)
        if self.voice_order_data:
            order = self.voice_order_data
            print(2)
            self.order_data = {"menu": []}
            for item in order.get("menu", []):
                self.order_data["menu"].append(
                    {
                        "name": item.get("name"),
                        "price": item.get("price"),
                        "qty": item.get("qty", 1),
                        "sauce": item.get("sauce"),
                        "vegetable": item.get("vegetable"),
                        "cheese": item.get("cheese"),
                    }
                )
            self.update_order_list()
            self.stack.setCurrentIndex(6)
            QMessageBox.information(
                self, "음성 주문", "음성 주문이 도착했습니다. 결제를 진행해주세요."
            )

    def complete_order(self):
        # 결제 완료 시
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        order_with_time = self.order_data.copy()
        order_with_time["timestamp"] = timestamp
        # 여기서만 JSON 파일 저장
        order_filename = f"order_{timestamp}.json"
        with open(order_filename, "w", encoding="utf-8") as f:
            json.dump(order_with_time, f, ensure_ascii=False, indent=4)
        # 그리고 서버로 POST
        result = send_order_to_server(order_with_time)

    def event(self, event):
        if event.type() == QEvent.User:
            print("VoiceOrderEvent 수신됨")
            self.process_voice_order()
            return True
        return super().event(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerbowayApp()
    window.show()
    sys.exit(app.exec_())
    QLoggingCategory.setFilterRules(
        "*.debug=false\n*.info=false\n*.warning=false\n*.critical=true"
    )  # 내부 디버그 메세지 숨김
