
import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget, QPushButton,
    QListWidget, QMessageBox
)
from PyQt5 import uic
from PyQt5.QtCore import Qt


class SerbowayApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serboway Kiosk")
        self.setGeometry(200, 200, 600, 500)

        self.order_data = {'menu': []}
        self.current_sandwich = None
        self.selected_sauce = None
        self.selected_vegetable = None
        self.selected_cheese = None

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # 각 UI 로딩
        self.page0 = uic.loadUi("/home/addinedu/dev_ws/PyQT/1_choose_ordermethod.ui")
        self.page1 = uic.loadUi("/home/addinedu/dev_ws/PyQT/2_choose_sandwich.ui")
        self.page2 = uic.loadUi("/home/addinedu/dev_ws/PyQT/3_choose_sauce.ui")
        self.page3 = uic.loadUi("/home/addinedu/dev_ws/PyQT/4_choose_vegetables.ui")
        self.page4 = uic.loadUi("/home/addinedu/dev_ws/PyQT/5_choose_cheese.ui")
        self.page5 = uic.loadUi("/home/addinedu/dev_ws/PyQT/6_confirm_order.ui")
        self.page6 = uic.loadUi("/home/addinedu/dev_ws/PyQT/7_choose_paymentmethod.ui")
        self.page7 = uic.loadUi("/home/addinedu/dev_ws/PyQT/8_order_complete.ui")

        for page in [self.page0, self.page1, self.page2, self.page3, self.page4, self.page5, self.page6, self.page7]:
            self.stack.addWidget(page)

        self.connect_buttons()

    def connect_buttons(self):
        def btn(page, object_name):
            return page.findChild(QPushButton, object_name, Qt.FindChildrenRecursively)

        self.page0.findChild(QPushButton, "voiceBtn", Qt.FindChildrenRecursively).clicked.connect(lambda: self.stack.setCurrentIndex(1))
        self.page0.findChild(QPushButton, "touchBtn", Qt.FindChildrenRecursively).clicked.connect(lambda: self.stack.setCurrentIndex(1))

        btn(self.page1, "BulgogiBtn").clicked.connect(lambda: self.select_sandwich("Bulgogi"))
        btn(self.page1, "ShrimpBtn").clicked.connect(lambda: self.select_sandwich("Shrimp"))
        btn(self.page1, "BaconBtn").clicked.connect(lambda: self.select_sandwich("Bacon"))

        btn(self.page2, "ItalianBtn").clicked.connect(lambda: self.select_sauce("Italian"))
        btn(self.page2, "ChillyBtn").clicked.connect(lambda: self.select_sauce("Chilly"))

        btn(self.page3, "LettuceBtn").clicked.connect(lambda: self.select_vegetable("Lettuce"))
        btn(self.page3, "RomaineBtn").clicked.connect(lambda: self.select_vegetable("Romaine"))
        btn(self.page3, "BazilBtn").clicked.connect(lambda: self.select_vegetable("Bazil"))

        btn(self.page4, "SliceBtn").clicked.connect(lambda: self.select_cheese("Slice"))
        btn(self.page4, "ShredBtn").clicked.connect(lambda: self.select_cheese("Shred"))
        btn(self.page4, "MozzarellaBtn").clicked.connect(lambda: self.select_cheese("Mozzarella"))

        self.order_list_widget = self.page5.findChild(QListWidget, "listWidget", Qt.FindChildrenRecursively)
        btn(self.page5, "payBtn").clicked.connect(self.go_to_payment)
        btn(self.page5, "pushButton").clicked.connect(self.restart_order)

        btn(self.page6, "rfidBtn").clicked.connect(self.complete_order)

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
        price_map = {"Bulgogi": 6500, "Shrimp": 6200, "Bacon": 6000}
        item = {
            "name": self.current_sandwich,
            "price": price_map.get(self.current_sandwich, 0),
            "qty": 1,
            "sauce": self.selected_sauce,
            "vegetable": self.selected_vegetable,
            "cheese": self.selected_cheese
        }
        self.order_data["menu"].append(item)

    def update_order_list(self):
        self.order_list_widget.clear()
        for item in self.order_data["menu"]:
            text = f"{item['name']} ({item['sauce']}/{item['vegetable']}/{item['cheese']}) x{item['qty']} - {item['price']}원"
            self.order_list_widget.addItem(text)

    def go_to_payment(self):
        if not self.order_data["menu"]:
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

