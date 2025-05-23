# ===================== 필수 패치 =====================
import torch
torch.classes.__path__ = []

# ===================== 표준 라이브러리 =====================
import os
import json
import io
import logging
from datetime import datetime
from typing import Dict, Any, Optional
import requests
import socket
import numpy as np
import librosa

# ===================== 서드파티 라이브러리 =====================
import streamlit as st
import sounddevice as sd
import soundfile as sf
import whisper
from gtts import gTTS
from pydub import AudioSegment

# ===================== LangChain 관련 =====================
from langchain_core.messages import AIMessage, HumanMessage
from langchain_community.tools import tool
from langchain_openai import ChatOpenAI
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

# ----------- 환경 변수 로드 ----------
from dotenv import load_dotenv
load_dotenv()

# ===================== 시스템 초기화 =====================
class EnhancedOrderState:
    """강화된 주문 상태 관리 클래스"""
    def __init__(self, table_number=1):
        self.menu = None
        self.sauce = None
        self.vegetable = None
        self.cheese = None
        self.step = "menu"
        self.confirmed = False
        self.table_number = table_number
        self._validation_rules = {
            'menu': lambda x: x in st.session_state.menu_data,
            'sauce': lambda x: x in st.session_state.sauce_data,
            'vegetable': lambda x: x in st.session_state.vegetable_data,
            'cheese': lambda x: x in st.session_state.cheese_data
        }

    def validate_field(self, field: str, value: str) -> bool:
        """주문 필드 유효성 검사"""
        return self._validation_rules[field](value)

    def get_dict(self) -> Dict[str, Any]:
        """다단계 주문 데이터 생성"""
        order_data = {
            "table_number": self.table_number,
            "timestamp": datetime.now().strftime("%Y%m%d-%H%M%S"),
            "steps": {}
        }
        
        if self.menu: order_data["steps"]["menu"] = self._build_menu_item()
        if self.sauce: order_data["steps"]["sauce"] = self._build_sauce_item()
        if self.vegetable: order_data["steps"]["vegetable"] = self._build_vegetable_item()
        if self.cheese: order_data["steps"]["cheese"] = self._build_cheese_item()
        
        return order_data

    def _build_menu_item(self):
        return {
            "name": self.menu,
            "price": st.session_state.menu_data.get(self.menu, {}).get("price", 0)
        }

    def _build_sauce_item(self):
        return {
            "name": self.sauce,
            "price": st.session_state.sauce_data.get(self.sauce, {}).get("price", 0)
        }

    def _build_vegetable_item(self):
        return {
            "name": self.vegetable,
            "price": st.session_state.vegetable_data.get(self.vegetable, {}).get("price", 0)
        }

    def _build_cheese_item(self):
        return {
            "name": self.cheese,
            "price": st.session_state.cheese_data.get(self.cheese, {}).get("price", 0)
        }

    def reset(self):
        """상태 초기화"""
        self.__init__(self.table_number)

# ===================== 음성 처리 엔진 =====================
class SpeechProcessor:
    """고급 음성 처리 시스템"""
    def __init__(self):
        self.model = self._load_model()
        self.sample_rate = 16000
        self.noise_threshold = 0.025

    @st.cache_resource(ttl=3600)
    def _load_model(_self):
        """GPU 가속 모델 로딩"""
        return whisper.load_model(
            "small",
            device="cuda" if torch.cuda.is_available() else "cpu"
        )

    def _preprocess(self, audio_data):
        """음성 전처리 파이프라인"""
        audio_data = audio_data.astype(np.float32)
        audio_data /= np.max(np.abs(audio_data))
        audio_data = librosa.effects.trim(audio_data, top_db=30)[0]
        return audio_data

    def transcribe(self, audio_path):
        """향상된 음성 인식"""
        try:
            raw_audio, _ = sf.read(audio_path)
            processed = self._preprocess(raw_audio)
            
            if np.max(np.abs(processed)) < self.noise_threshold:
                raise ValueError("무음 감지")
                
            return self.model.transcribe(
                processed,
                language="ko",
                temperature=0.1,
                best_of=3
            )["text"].strip()
        except Exception as e:
            logging.error(f"음성 인식 오류: {str(e)}")
            return ""

# ===================== 도구 함수들 =====================
@tool
def update_order(menu: Optional[str] = None, sauce: Optional[str] = None,
                vegetable: Optional[str] = None, cheese: Optional[str] = None) -> str:
    """강화된 주문 업데이트 도구"""
    state = st.session_state.order_state
    updates = []
    
    if menu and state.validate_field('menu', menu):
        state.menu = menu
        updates.append(f"메뉴: {menu}")
    if sauce and state.validate_field('sauce', sauce):
        state.sauce = sauce
        updates.append(f"소스: {sauce}")
    if vegetable and state.validate_field('vegetable', vegetable):
        state.vegetable = vegetable
        updates.append(f"야채: {vegetable}")
    if cheese and state.validate_field('cheese', cheese):
        state.cheese = cheese
        updates.append(f"치즈: {cheese}")
    
    return f"업데이트 완료: {', '.join(updates)}" if updates else "변경사항 없음"

@tool
def confirm_order(confirm: bool) -> str:
    """개선된 주문 확정 도구"""
    if not confirm:
        return "주문이 취소되었습니다"
    
    try:
        order_data = st.session_state.order_state.get_dict()
        required_steps = ["menu", "sauce", "vegetable", "cheese"]
        
        if not all(step in order_data["steps"] for step in required_steps):
            missing = [step for step in required_steps if step not in order_data["steps"]]
            return f"❌ 누락된 항목: {', '.join(missing)}"
            
        send_result = send_order_to_kiosk.invoke(order_data)
        return (
            f"✅ 주문 확정 완료\n"
            f"{_format_order_summary(order_data)}\n"
            f"키오스크 응답: {send_result}"
        )
    except Exception as e:
        logging.error(f"주문 확정 오류: {str(e)}")
        return f"❌ 시스템 오류: {str(e)}"

def _format_order_summary(order_data: dict) -> str:
    """주문 요약 포매터"""
    total = sum(item["price"] for step in order_data["steps"].values() for item in [step])
    return (
        f"메뉴: {order_data['steps']['menu']['name']} ({order_data['steps']['menu']['price']}원)\n"
        f"소스: {order_data['steps']['sauce']['name']}\n"
        f"야채: {order_data['steps']['vegetable']['name']}\n"
        f"치즈: {order_data['steps']['cheese']['name']}\n"
        f"총액: {total}원"
    )
@tool
def send_order_to_kiosk(order_data: dict):
    """주문 정보를 키오스크로 전송합니다."""
    KIOSK_HOST = "192.168.0.159"
    KIOSK_PORT = 12345
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((KIOSK_HOST, KIOSK_PORT))
            client_socket.sendall(json.dumps(order_data).encode())
            response = client_socket.recv(1024).decode()
        return response
    except Exception as e:
        return f"키오스크 전송 오류: {str(e)}"

# ===================== 메인 애플리케이션 =====================
def main():
    st.set_page_config(page_title="서보웨이 AI 주문 2.0", page_icon="🥪", layout="wide")
    st.image("image/Menu.png", use_column_width=True)
    
    # 상태 초기화
    if "order_state" not in st.session_state:
        st.session_state.order_state = EnhancedOrderState()
    
    # 음성 처리기 초기화
    if "speech_processor" not in st.session_state:
        st.session_state.speech_processor = SpeechProcessor()
    
    # 에이전트 시스템 초기화
    tools = [
        update_order,
        confirm_order,
        # 다른 도구들...
    ]
    
    # UI 렌더링 로직...
    
if __name__ == "__main__":
    main()
