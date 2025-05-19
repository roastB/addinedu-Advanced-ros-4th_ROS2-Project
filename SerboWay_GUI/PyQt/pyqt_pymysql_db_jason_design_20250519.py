import sys
import pymysql
import json
import random
import string
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget, QPushButton,
    QListWidget, QMessageBox, QLabel
)
from PyQt5 import uic
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QIcon

# DB에서 최신 메뉴 JSON 불러오기
def get_menu_json():
    conn = pymysql.connect(
        host="localhost", user="root", password="1",
        db="serbobase", charset="utf8mb4"
    )
    with conn.cursor() as cursor:
        cursor.execute("SELECT json_data FROM menu_json ORDER BY id DESC LIMIT 1")
        row = cursor.fetchone()
    conn.close()
    return json.loads(row[0]) if row else None

class SerbowayApp(QMainWindow):
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
        self.page0 = uic.loadUi("1_choose_ordermethod.ui")
        self.page1 = uic.loadUi("2_choose_sandwich.ui")
        self.page2 = uic.loadUi("3_choose_sauce.ui")
        self.page3 = uic.loadUi("4_choose_vegetables.ui")
        self.page4 = uic.loadUi("5_choose_cheese.ui")
        self.page5 = uic.loadUi("6_confirm_order.ui")
        self.page6 = uic.loadUi("7_choose_paymentmethod.ui")
        self.page7 = uic.loadUi("8_order_complete.ui")
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
        btn(self.page0, "voiceBtn").clicked.connect(lambda: self.stack.setCurrentIndex(1))
        btn(self.page0, "touchBtn").clicked.connect(lambda: self.stack.setCurrentIndex(1))

        # 결제/재시작/완료 버튼
        btn(self.page5, "payBtn").clicked.connect(self.go_to_payment)
        btn(self.page5, "pushButton").clicked.connect(self.restart_order)
        btn(self.page6, "rfidBtn").clicked.connect(self.complete_order)

    def populate_dynamic_buttons(self):
        # -------------------------------
        # 샌드위치 버튼: (메뉴키, 이미지경로) 매핑
        # -------------------------------
        sandwich_map = {
            'BulgogiBtn': ('불고기 샌드위치', '/home/addinedu/roscamp-repo-3/SerboWay_GUI/PyQt/Bulgogi141.png'),
            'ShrimpBtn':  ('새우 샌드위치',  '/home/addinedu/roscamp-repo-3/SerboWay_GUI/PyQt/Shrimp141.png'),
            'BaconBtn':   ('베이컨 샌드위치','/home/addinedu/roscamp-repo-3/SerboWay_GUI/PyQt/Bacon141.png')
        }

        for obj_name, (menu_key, img_path) in sandwich_map.items():
            btn = self.page1.findChild(QPushButton, obj_name, Qt.FindChildrenRecursively)
            # 버튼이 없거나 JSON에 메뉴키가 없으면 건너뛰기
            if not btn or menu_key not in self.menu_json.get('menu', {}):
                continue

            # 배경 이미지 제거
            btn.setStyleSheet("background-image: none;")

            # 아이콘 설정
            btn.setIcon(QIcon(img_path))
            btn.setIconSize(QSize(128, 128))

            # 텍스트 설정 (메뉴명 + 가격)
            price = self.menu_json['menu'][menu_key]['price']
            # btn.setText(f"{menu_key}\n({price}원)")

            

            # 클릭 시 select_sandwich(menu_key) 호출
            try:
                btn.clicked.disconnect()
            except TypeError:
                pass
            btn.clicked.connect(lambda _, m=menu_key: self.select_sandwich(m))

        # -------------------------------
        # 소스 버튼 매핑 (텍스트만)
        # -------------------------------
        sauce_map = {'ItalianBtn': '이탈리안', 'ChillyBtn': '칠리'}
        for obj_name, sauce_key in sauce_map.items():
            btn = self.page2.findChild(QPushButton, obj_name, Qt.FindChildrenRecursively)
            if btn and sauce_key in self.menu_json.get('sauce', {}):
                price = self.menu_json['sauce'][sauce_key].get('price', 0)
                btn.setText(f"{sauce_key}\n(+{price}원)")
                try:
                    btn.clicked.disconnect()
                except TypeError:
                    pass
                btn.clicked.connect(lambda _, s=sauce_key: self.select_sauce(s))

        # -------------------------------
        # 야채 버튼 매핑
        # -------------------------------
        veg_map = {'LettuceBtn': '양상추', 'RomaineBtn': '로메인', 'BazilBtn': '바질'}
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
            'SliceBtn':      '슬라이스 치즈',
            'ShredBtn':      '슈레드 치즈',
            'MozzarellaBtn': '모짜렐라 치즈'
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
        self.stack.setCurrentIndex(7)

    def restart_order(self):
        self.order_data = {'menu': []}
        self.stack.setCurrentIndex(1)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerbowayApp()
    window.show()
    sys.exit(app.exec_())
