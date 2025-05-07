import sys
import pymysql
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QStackedWidget, QMessageBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView


class SandwichKiosk(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("샌드위치 주문 키오스크")
        self.setGeometry(100, 100, 400, 400)

        self.selections = {}
        self.selection_prices = {}
        self.payment_method = None  # 결제수단 저장

        self.db_config = {
            'host': 'localhost',
            'user': 'root',
            'password': '1',
            'database': 'serbobase',
            'charset': 'utf8mb4'
        }

        self.stack = QStackedWidget()
        self.summary_label = QLabel("선택한 재료 없음")
        self.summary_label.setAlignment(Qt.AlignCenter)

        self.stack.addWidget(self.create_welcome_page())  # index 0

        # 화면 주문 단계 (1~4)
        self.categories = ["sauce", "lettuce", "cheese", "meat"]
        for idx, category in enumerate(self.categories):
            final_step = (idx == len(self.categories) - 1)
            self.stack.addWidget(self.create_selection_page(f"{category.capitalize()}를 선택하세요", category, final_step))

        self.stack.addWidget(self.create_payment_page())  # index 5
        self.stack.addWidget(self.create_voice_order_page())  # index 6

        layout = QVBoxLayout()
        layout.addWidget(self.summary_label)
        layout.addWidget(self.stack)
        self.setLayout(layout)

    def create_welcome_page(self):
        page = QWidget()
        vbox = QVBoxLayout()

        label = QLabel("환영합니다!\n어떤 방식으로 주문하시겠습니까?")
        label.setAlignment(Qt.AlignCenter)
        vbox.addWidget(label)

        screen_btn = QPushButton("🖐 화면 주문")
        screen_btn.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        vbox.addWidget(screen_btn)

        voice_btn = QPushButton("🎙 음성 주문")
        voice_btn.clicked.connect(lambda: self.stack.setCurrentIndex(6))
        vbox.addWidget(voice_btn)

        page.setLayout(vbox)
        return page

    def create_selection_page(self, title, category, final_step=False):
        page = QWidget()
        vbox = QVBoxLayout()

        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        vbox.addWidget(label)

        try:
            conn = pymysql.connect(**self.db_config)
            cursor = conn.cursor()
            cursor.execute("SELECT name, price FROM ingredients WHERE ingredient_type = %s", (category,))
            results = cursor.fetchall()
            conn.close()
        except Exception as e:
            QMessageBox.critical(self, "DB 오류", str(e))
            results = []

        for name, price in results:
            display_text = f"{name} ({price}원)" if price > 0 else name
            btn = QPushButton(display_text)
            btn.clicked.connect(lambda _, n=name, p=price, c=category: self.select_option(c, n, p, final_step))
            vbox.addWidget(btn)

        page.setLayout(vbox)
        return page

    def create_payment_page(self):
        page = QWidget()
        vbox = QVBoxLayout()

        self.payment_summary = QLabel("주문 요약을 불러오는 중...")
        self.payment_summary.setAlignment(Qt.AlignCenter)
        vbox.addWidget(self.payment_summary)

        # 결제 수단 선택 버튼
        card_btn = QPushButton("💳 신용카드 결제")
        card_btn.clicked.connect(lambda: self.set_payment_and_complete("card"))
        vbox.addWidget(card_btn)

        smartpay_btn = QPushButton("📱 스마트페이")
        smartpay_btn.clicked.connect(lambda: self.set_payment_and_complete("smartpay"))
        vbox.addWidget(smartpay_btn)

        page.setLayout(vbox)
        return page

    # 5/7 연동 코드 수정
    def create_voice_order_page(self):
        page = QWidget()
        vbox = QVBoxLayout()
        
        # Streamlit 앱을 위한 웹뷰 위젯 생성
        self.webview = QWebEngineView()
        
        # 시작 메시지
        label = QLabel("음성 주문 시스템을 시작합니다...")
        label.setAlignment(Qt.AlignCenter)
        vbox.addWidget(label)
        
        # Streamlit 앱 시작 버튼
        start_btn = QPushButton("🎤 음성 주문 시작")
        start_btn.clicked.connect(self.start_voice_agent)
        vbox.addWidget(start_btn)
        
        # 웹뷰 추가 (처음에는 숨김)
        self.webview.setVisible(False)
        vbox.addWidget(self.webview)
        
        # 돌아가기 버튼
        back_btn = QPushButton("⬅ 돌아가기")
        back_btn.clicked.connect(self.close_voice_agent)
        vbox.addWidget(back_btn)
        
        page.setLayout(vbox)
        return page

    def start_voice_agent(self):
        try:
            # 이미 실행 중인 프로세스가 있으면 종료
            if hasattr(self, 'streamlit_process') and self.streamlit_process:
                self.close_voice_agent()
            
            # Streamlit 앱 실행 (서브프로세스로)
            streamlit_cmd = "streamlit run Serboway_whisper_agent2.py --server.headless=True"
            self.streamlit_process = subprocess.Popen(streamlit_cmd.split(), stdout=subprocess.PIPE)
            
            # 프로세스 종료를 위한 atexit 등록
            atexit.register(self.kill_streamlit_process)
            
            # 웹뷰에 로컬 URL 로드 (약간의 지연 필요할 수 있음)
            import time
            time.sleep(3)  # Streamlit 서버가 시작될 때까지 대기
            
            # 웹뷰 표시 및 URL 로드
            self.webview.setVisible(True)
            self.webview.setUrl(QUrl("http://localhost:8501"))
            
        except Exception as e:
            QMessageBox.critical(self, "오류", f"음성 주문 시스템 시작 실패: {str(e)}")

def close_voice_agent(self):
    # 웹뷰 숨기기
    if hasattr(self, 'webview'):
        self.webview.setVisible(False)
    
    # Streamlit 프로세스 종료
    if hasattr(self, 'streamlit_process') and self.streamlit_process:
        self.kill_streamlit_process()
        self.streamlit_process = None
    
    # 기본 화면으로 돌아가기
    self.stack.setCurrentIndex(0)

def kill_streamlit_process(self):
    if hasattr(self, 'streamlit_process') and self.streamlit_process:
        if os.name == 'nt':  # Windows
            subprocess.call(['taskkill', '/F', '/T', '/PID', str(self.streamlit_process.pid)])
        else:  # Linux/Mac
            self.streamlit_process.kill()


    def select_option(self, category, name, price, final_step):
        self.selections[category] = name
        self.selection_prices[category] = price
        self.update_summary()

        if final_step:
            self.update_payment_summary()
            self.stack.setCurrentIndex(5)
        else:
            self.stack.setCurrentIndex(self.stack.currentIndex() + 1)

    def update_summary(self):
        summary = "선택 중: " + ", ".join([f"{k}: {v}" for k, v in self.selections.items()])
        self.summary_label.setText(summary)

    def update_payment_summary(self):
        total = sum(self.selection_prices.values())
        lines = [f"{k}: {self.selections[k]} ({self.selection_prices[k]}원)" for k in self.selections]
        text = f"[최종 주문 요약]\n" + "\n".join(lines) + f"\n\n총 가격: {total}원"
        self.payment_summary.setText(text)

    def set_payment_and_complete(self, method):
        self.payment_method = method
        self.complete_order()

    def complete_order(self):
        try:
            conn = pymysql.connect(**self.db_config)
            cursor = conn.cursor()

            final_menu_name = self.selections.get("meat", "") + "샌드위치"
            price = sum(self.selection_prices.values())

            cursor.execute("""
                INSERT INTO orders (table_number, order_method, final_menu_name, price, payment_method)
                VALUES (1, 'kiosk', %s, %s, %s)
            """, (final_menu_name, price, self.payment_method))
            order_id = cursor.lastrowid

            for ingredient_type, ingredient_name in self.selections.items():
                cursor.execute("""
                    INSERT INTO order_ingredients (order_id, ingredient_type, ingredient_name)
                    VALUES (%s, %s, %s)
                """, (order_id, ingredient_type, ingredient_name))

            conn.commit()
            conn.close()

            QMessageBox.information(self, "주문 완료",
                f"{final_menu_name}가 성공적으로 저장되었습니다!\n\n총 결제금액: {price}원\n결제수단: {self.payment_method}")
            self.close()
        except Exception as e:
            QMessageBox.critical(self, "DB 저장 오류", str(e))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SandwichKiosk()
    window.show()
    sys.exit(app.exec_())
#22
#222