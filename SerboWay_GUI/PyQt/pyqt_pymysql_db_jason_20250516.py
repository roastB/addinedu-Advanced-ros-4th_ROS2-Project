import sys
import pymysql
import json
import random
import string
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QListWidget, QLabel,
    QComboBox, QPushButton, QHBoxLayout, QStackedWidget
)
from PyQt5.QtCore import Qt

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

# 콤보박스에 아이템 표시 및 데이터 연결
def populate_combo(combo, items_dict):
    combo.clear()
    for key, info in items_dict.items():
        price = info.get('price', 0)
        combo.addItem(f"{key} ({price}원)", (key, price))

class MenuPage(QWidget):
    def __init__(self, menu_json, parent):
        super().__init__()
        self.parent = parent
        self.menu_json = menu_json
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        # 메뉴 리스트
        self.listWidget = QListWidget()
        for name, info in self.menu_json['menu'].items():
            self.listWidget.addItem(f"{name} ({info.get('price', 0)}원)")
        # 옵션 콤보박스
        self.sauceCombo = QComboBox(); populate_combo(self.sauceCombo, self.menu_json['sauce'])
        self.vegCombo = QComboBox(); populate_combo(self.vegCombo, self.menu_json['vegetable'])
        self.cheeseCombo = QComboBox(); populate_combo(self.cheeseCombo, self.menu_json['cheese'])
        # 설명 및 버튼
        self.descLabel = QLabel("메뉴와 옵션을 선택하세요.")
        self.addBtn = QPushButton("주문에 추가 (0건, 0원)")
        self.nextBtn = QPushButton("주문 요약으로 이동")
        # 레이아웃 배치
        layout.addWidget(QLabel("샌드위치 메뉴 선택")); layout.addWidget(self.listWidget)
        layout.addWidget(QLabel("소스 선택")); layout.addWidget(self.sauceCombo)
        layout.addWidget(QLabel("야채 선택")); layout.addWidget(self.vegCombo)
        layout.addWidget(QLabel("치즈 선택")); layout.addWidget(self.cheeseCombo)
        layout.addWidget(self.descLabel)
        btn_layout = QHBoxLayout(); btn_layout.addWidget(self.addBtn); btn_layout.addWidget(self.nextBtn)
        layout.addLayout(btn_layout)
        # 시그널 연결
        self.listWidget.currentItemChanged.connect(self.update_desc)
        self.sauceCombo.currentIndexChanged.connect(self.update_desc)
        self.vegCombo.currentIndexChanged.connect(self.update_desc)
        self.cheeseCombo.currentIndexChanged.connect(self.update_desc)
        self.addBtn.clicked.connect(self.add_order)
        self.nextBtn.clicked.connect(self.goto_summary)

    def update_desc(self, current=None, previous=None):
        item = self.listWidget.currentItem()
        if not item:
            return
        menu_text = item.text()
        name = menu_text.split(' (')[0]
        base_price = int(menu_text.split('(')[1].replace('원)', ''))
        sauce_name, sauce_price = self.sauceCombo.currentData()
        veg_name, veg_price = self.vegCombo.currentData()
        cheese_name, cheese_price = self.cheeseCombo.currentData()
        total_price = base_price + sauce_price + veg_price + cheese_price
        self.descLabel.setText(
            f"{name}: {self.menu_json['menu'][name]['description']}\n"
            f"옵션: {sauce_name}, {veg_name}, {cheese_name} / 가격: {total_price}원"
        )

    def add_order(self):
        item = self.listWidget.currentItem()
        if not item:
            return
        menu_text = item.text()
        name = menu_text.split(' (')[0]
        base_price = int(menu_text.split('(')[1].replace('원)', ''))
        sauce_name, sauce_price = self.sauceCombo.currentData()
        veg_name, veg_price = self.vegCombo.currentData()
        cheese_name, cheese_price = self.cheeseCombo.currentData()
        unit_price = base_price + sauce_price + veg_price + cheese_price
        # 기존 주문 검사
        for order in self.parent.orders:
            if order['menu'] == name and order['options'] == [sauce_name, veg_name, cheese_name]:
                order['quantity'] += 1
                order['price'] = order['unit_price'] * order['quantity']
                break
        else:
            self.parent.orders.append({
                'menu': name,
                'options': [sauce_name, veg_name, cheese_name],
                'unit_price': unit_price,
                'quantity': 1,
                'price': unit_price
            })
        total_count = sum(o['quantity'] for o in self.parent.orders)
        total_sum = sum(o['price'] for o in self.parent.orders)
        self.addBtn.setText(f"주문에 추가 ({total_count}건, {total_sum}원)")

    def goto_summary(self):
        self.parent.summaryPage.show_summary()
        self.parent.setCurrentIndex(1)

class SummaryPage(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.orderList = QListWidget()
        self.totalLabel = QLabel("총 합계: 0원")
        payBtn = QPushButton("결제하기")
        layout.addWidget(QLabel("주문 요약")); layout.addWidget(self.orderList)
        layout.addWidget(self.totalLabel); layout.addWidget(payBtn)
        payBtn.clicked.connect(self.go_payment)

    def show_summary(self):
        self.orderList.clear()
        total = 0
        for idx, order in enumerate(self.parent.orders, 1):
            opts = ", ".join(order['options'])
            line = f"{idx}. {order['menu']} x{order['quantity']} ({opts}) - {order['price']}원"
            self.orderList.addItem(line)
            total += order['price']
        self.totalLabel.setText(f"총 합계: {total}원")

    def go_payment(self):
        self.parent.receipt = ''.join(random.choices(string.digits, k=6))
        self.parent.paymentPage.update_payment()
        self.parent.setCurrentIndex(2)

class PaymentPage(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.receiptLabel = QLabel("영수증 번호: ")
        rfidBtn = QPushButton("RFID 결제")
        nextBtn = QPushButton("확인 화면")
        layout.addWidget(QLabel("결제 방식 선택")); layout.addWidget(self.receiptLabel)
        layout.addWidget(rfidBtn); layout.addWidget(nextBtn)
        rfidBtn.clicked.connect(lambda: self.receiptLabel.setText(f"영수증 번호: {self.parent.receipt}"))
        nextBtn.clicked.connect(lambda: self.parent.setCurrentIndex(3))

    def update_payment(self):
        self.receiptLabel.setText(f"영수증 번호: {self.parent.receipt}")

class ConfirmationPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("주문이 완료되었습니다!"))

class OrderApp(QStackedWidget):
    def __init__(self):
        super().__init__()
        self.orders = []
        self.receipt = None
        menu_json = get_menu_json()
        self.menuPage = MenuPage(menu_json, self)
        self.summaryPage = SummaryPage(self)
        self.paymentPage = PaymentPage(self)
        self.confirmPage = ConfirmationPage()
        for page in [self.menuPage, self.summaryPage, self.paymentPage, self.confirmPage]:
            self.addWidget(page)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = OrderApp()
    win.setWindowTitle("샌드위치 주문 시스템")
    win.resize(400, 600)
    win.show()
    sys.exit(app.exec_())
