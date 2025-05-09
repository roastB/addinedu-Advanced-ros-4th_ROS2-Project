import sys
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QStackedWidget, QSpinBox, QListWidget, QMessageBox, QWidget
)
from PyQt5.QtCore import Qt


class WelcomePage(QMainWindow):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        uic.loadUi("/home/addinedu/dev_ws/PyQT/1_choose_ordermethod.ui", self)

        self.label = self.findChild(QLabel, "WelcomeLabel")
        self.voice_btn = self.findChild(QPushButton, "voiceButton")
        self.touch_btn = self.findChild(QPushButton, "touchButton")

        if self.voice_btn:
            self.voice_btn.clicked.connect(lambda: stack.setCurrentIndex(1))
        if self.touch_btn:
            self.touch_btn.clicked.connect(lambda: stack.setCurrentIndex(2))


class SandwichPage(QMainWindow):
    def __init__(self, stack, order_data):
        super().__init__()
        self.stack = stack
        self.order_data = order_data

        uic.loadUi("/home/addinedu/dev_ws/PyQT/2_choose_sandwich.ui", self)

        self.stacked_widget = self.findChild(QStackedWidget, "stackedwidget")
        self.sandwich_label = self.findChild(QLabel, "sandwichLabel")

        self.bulgogi_label = self.findChild(QLabel, "bulgogiLabel")
        self.shrimp_label = self.findChild(QLabel, "shrimpLabel")
        self.bacon_label = self.findChild(QLabel, "baconLabel")

        self.bulgogi_spin = self.findChild(QSpinBox, "spinBulgogi")
        self.shrimp_spin = self.findChild(QSpinBox, "spinShrimp")
        self.bacon_spin = self.findChild(QSpinBox, "spinBacon")
        self.cart_btn = self.findChild(QPushButton, "cartBtn")

        if self.bulgogi_label:
            self.bulgogi_label.mousePressEvent = lambda event: self.stacked_widget.setCurrentIndex(0)
        if self.shrimp_label:
            self.shrimp_label.mousePressEvent = lambda event: self.stacked_widget.setCurrentIndex(1)
        if self.bacon_label:
            self.bacon_label.mousePressEvent = lambda event: self.stacked_widget.setCurrentIndex(2)

        if self.cart_btn:
            self.cart_btn.clicked.connect(self.go_next)

    def go_next(self):
        price_map = {
            "Bulgogi": 6500,
            "Shrimp": 6200,
            "Bacon": 6000
        }

        selected_items = []
        for name, spin in [
            ("Bulgogi", self.bulgogi_spin),
            ("Shrimp", self.shrimp_spin),
            ("Bacon", self.bacon_spin)
        ]:
            if spin:
                qty = spin.value()
                if qty > 0:
                    selected_items.append({
                        'name': name,
                        'price': price_map[name],
                        'qty': qty
                    })

        if not selected_items:
            QMessageBox.warning(self, "주의", "하나 이상의 샌드위치를 선택해주세요.")
            return

        if 'menu' not in self.order_data:
            self.order_data['menu'] = selected_items
        else:
            existing_names = [item['name'] for item in self.order_data['menu']]
            for new_item in selected_items:
                if new_item['name'] in existing_names:
                    for old_item in self.order_data['menu']:
                        if old_item['name'] == new_item['name']:
                            old_item['qty'] += new_item['qty']
                            break
                else:
                    self.order_data['menu'].append(new_item)

        self.stack.setCurrentIndex(3)


class OptionPage(QMainWindow):
    def __init__(self, stack, order_data, title, options, next_index):
        super().__init__()
        self.stack = stack
        self.order_data = order_data
        self.title = title
        self.options = options
        self.next_index = next_index

        # UI 파일 경로를 제목 기준으로 설정
        ui_map = {
            "Sauce": "/home/addinedu/dev_ws/PyQT/3_choose_sauce.ui",
            "Vegetables": "/home/addinedu/dev_ws/PyQT/4_choose_vegetables.ui",
            "Cheese": "/home/addinedu/dev_ws/PyQT/5_choose_cheese.ui"
        }
        uic.loadUi(ui_map.get(title, ""), self)

        self.option_buttons = []

        # 각 옵션 버튼을 찾아 클릭 이벤트 연결
        for name, price in options:
            btn = self.findChild(QPushButton, name)
            if btn:
                btn.clicked.connect(lambda _, n=name, p=price: self.select_option(n, p))
                self.option_buttons.append(btn)

        cart_btn = self.findChild(QPushButton, "cartBtn")
        if cart_btn:
            cart_btn.clicked.connect(self.go_next)

    def select_option(self, name, price):
        self.order_data[self.title.lower()] = {'name': name, 'price': price}

    def go_next(self):
        self.stack.setCurrentIndex(self.next_index)


class ConfirmPage(QMainWindow):
    def __init__(self, stack, order_data):
        super().__init__()
        self.stack = stack
        self.order_data = order_data
        uic.loadUi("/home/addinedu/dev_ws/PyQT/6_confirm_order.ui", self)

        self.summary = self.findChild(QLabel, "summaryLabel")
        self.confirm_btn = self.findChild(QPushButton, "confirmButton")
        self.home_btn = self.findChild(QPushButton, "homeButton")

        self.confirm_btn.clicked.connect(lambda: stack.setCurrentIndex(7))
        self.home_btn.clicked.connect(lambda: stack.setCurrentIndex(0))

    def showEvent(self, event):
        text = "[주문 내역]\n"
        total = 0

        if isinstance(self.order_data.get('menu'), list):
            for item in self.order_data['menu']:
                name = item['name']
                price = item['price']
                qty = item.get('qty', 1)
                subtotal = price * qty
                text += f"- {name} x{qty}: {subtotal}원\n"
                total += subtotal
        else:
            item = self.order_data.get('menu')
            if item:
                name = item['name']
                price = item['price']
                qty = item.get('qty', 1)
                subtotal = price * qty
                text += f"- {name} x{qty}: {subtotal}원\n"
                total += subtotal

        for key in ['sauce', 'vegetables', 'cheese']:
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


class PaymentPage(QMainWindow):
    def __init__(self, stack):
        super().__init__()
        self.stack = stack

        widget = QWidget()
        layout = QVBoxLayout(widget)

        label = QLabel("결제 방식 선택")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 36px")

        card_btn = QPushButton("신용카드 결제")
        pay_btn = QPushButton("스마트페이")
        card_btn.setStyleSheet("font-size: 24px")
        pay_btn.setStyleSheet("font-size: 24px")

        card_btn.clicked.connect(lambda: stack.setCurrentIndex(8))
        pay_btn.clicked.connect(lambda: stack.setCurrentIndex(8))

        layout.addWidget(label)
        layout.addWidget(card_btn)
        layout.addWidget(pay_btn)

        self.setCentralWidget(widget)


class CompletePage(QMainWindow):
    def __init__(self):
        super().__init__()
        widget = QWidget()
        layout = QVBoxLayout(widget)

        label = QLabel("Order Completed! ")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 24px; color: green")
        layout.addWidget(label)

        self.setCentralWidget(widget)


if __name__ == '__main__':
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
    main_window.resize(591, 511)
    main_window.show()

    sys.exit(app.exec_())
