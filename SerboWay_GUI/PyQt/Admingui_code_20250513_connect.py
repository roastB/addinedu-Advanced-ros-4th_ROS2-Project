import sys
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QListWidget,
    QProgressBar, QDateEdit, QTableWidget
)
from PyQt5.QtCore import QDate


class AdminGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # ✅ .ui 파일 로드하여 현재 클래스에 적용
        uic.loadUi("/home/addinedu/dev_ws/PyQT/SerbowayAdmin_charge.ui", self)

        # ✅ UI 내부 요소 연결 (objectName 기준)
        self.camera_label = self.findChild(QLabel, "cameraLabel")

        self.robot_progress = {
            "Waiter": self.findChild(QProgressBar, "progressWaiter"),
            "Maker": self.findChild(QProgressBar, "progressMaker"),
            "Pick&Wash": self.findChild(QProgressBar, "progressPickWash")
        }

        self.order_labels = [
            self.findChild(QLabel, "orderLabel01"),
            self.findChild(QLabel, "orderLabel02"),
            self.findChild(QLabel, "orderLabel03")
        ]

        self.todo_lists = {
            "Maker": self.findChild(QListWidget, "todoListMaker"),
            "Waiter": self.findChild(QListWidget, "todoListWaiter"),
            "Pick&Wash": self.findChild(QListWidget, "todoListPickWash")
        }

        self.prog_lists = {
            "Maker": self.findChild(QListWidget, "progListMaker"),
            "Waiter": self.findChild(QListWidget, "progListWaiter"),
            "Pick&Wash": self.findChild(QListWidget, "progListPickWash")
        }

        self.front_bar = self.findChild(QProgressBar, "progressFront")
        self.back_bar = self.findChild(QProgressBar, "progressBack")

        self.date_edit = self.findChild(QDateEdit, "dateEdit")
        self.sales_table = self.findChild(QTableWidget, "salesTable")

        if self.date_edit:
            self.date_edit.setDate(QDate.currentDate())  # 기본 날짜 설정


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminGUI()
    window.setWindowTitle("SerboWay Admin (UI 연결됨)")
    window.resize(1261, 761)  # 필요 시 조정
    window.show()
    sys.exit(app.exec_())