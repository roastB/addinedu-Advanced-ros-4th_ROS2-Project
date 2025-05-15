# ===================== 필수 패치 =====================
import torch
torch.classes.__path__ = []

# ===================== 표준 라이브러리 =====================
import os
import json
import io
from datetime import datetime
from typing import Dict, Any, Optional
import requests

# ===================== 서드파티 라이브러리 =====================
import streamlit as st
import sounddevice as sd
import soundfile as sf
import whisper
from gtts import gTTS

# ===================== LangChain 관련 =====================
from langchain_core.messages import AIMessage, HumanMessage
from langchain_community.tools import tool
from langchain_openai import ChatOpenAI
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

# ----------- 환경 변수 로드 ----------
from dotenv import load_dotenv
load_dotenv()

# ===================== 데이터 관리 =====================
@st.cache_data(ttl=300)
def load_menu_data(json_path: str = "menu_data.json") -> Dict[str, Any]:
    """JSON 파일에서 메뉴 데이터 로드"""
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        st.error(f"메뉴 로드 실패: {str(e)}")
        return {"menu": {}, "sauce": {}, "vegetable": {}, "cheese": {}}

def initialize_session():
    """세션 상태 초기화"""
    data = load_menu_data()
    st.session_state.update({
        "menu_data": data.get("menu", {}),
        "sauce_data": data.get("sauce", {}),
        "vegetable_data": data.get("vegetable", {}),
        "cheese_data": data.get("cheese", {})
    })

# ================== 주문 상태 관리 클래스 ==================
class OrderState:
    """주문 상태 관리 클래스"""
    def __init__(self):
        self.menu = None
        self.sauce = None
        self.vegetable = "양상추"
        self.cheese = "슬라이스 치즈"
        self.step = "menu"
        self.confirmed = False

    def get_dict(self) -> Dict[str, Any]:
        """주문 데이터를 딕셔너리로 변환"""
        return {
            "menu": {
                "name": self.menu,
                "price": st.session_state.menu_data.get(self.menu, {}).get("price", 0),
                "qty": 1
            },
            "sauce": {
                "name": self.sauce,
                "price": st.session_state.sauce_data.get(self.sauce, {}).get("price", 0)
            },
            "vegetables": {
                "name": self.vegetable,
                "price": st.session_state.vegetable_data.get(self.vegetable, {}).get("price", 0)
            },
            "cheese": {
                "name": self.cheese,
                "price": st.session_state.cheese_data.get(self.cheese, {}).get("price", 0)
            },
            "step": self.step,
            "confirmed": self.confirmed
        }

    def reset(self):
        """주문 상태 초기화"""
        self.__init__()

# ===================== 도구 함수들 =====================
@tool
def get_menu_list(tool_input: str = "") -> str:
    """메뉴 목록을 조회합니다. 특정 키워드가 포함된 메뉴만 필터링합니다."""
    result = "메뉴 목록:\n"
    for name, info in st.session_state.menu_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info['price']}원 ({info.get('description', '')})\n"
    return result

@tool
def get_sauce_list(tool_input: str = "") -> str:
    """소스 목록을 조회합니다. 특정 키워드가 포함된 소스만 필터링합니다."""
    result = "소스 목록:\n"
    for name, info in st.session_state.sauce_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def get_vegetable_list(tool_input: str = "") -> str:
    """야채 목록을 조회합니다. 특정 키워드가 포함된 야채만 필터링합니다."""
    result = "야채 목록:\n"
    for name, info in st.session_state.vegetable_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def get_cheese_list(tool_input: str = "") -> str:
    """치즈 목록을 조회합니다. 특정 키워드가 포함된 치즈만 필터링합니다."""
    result = "치즈 목록:\n"
    for name, info in st.session_state.cheese_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def update_order(menu: Optional[str] = None, sauce: Optional[str] = None) -> str:
    """주문 정보를 업데이트합니다. 메뉴와 소스를 변경할 수 있습니다."""
    order_state = st.session_state.order_state
    if menu and menu in st.session_state.menu_data:
        order_state.menu = menu
    if sauce and sauce in st.session_state.sauce_data:
        order_state.sauce = sauce
    return "주문이 업데이트되었습니다"

@tool
def confirm_order(confirm: bool) -> str:
    """주문을 확정하고, 주문별 JSON 파일을 생성한 뒤 서버로 전송합니다."""
    if confirm:
        try:
            order_data = st.session_state.order_state.get_dict()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"order_{timestamp}.json"
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(order_data, f, ensure_ascii=False, indent=2)
            server_url = "https://your-server.com/api/orders"
            try:
                response = requests.post(server_url, json=order_data, timeout=5)
                if response.status_code == 200:
                    return f"✅ 주문 완료 및 전송 성공!\n{get_order_summary.invoke({'tool_input': ''})['output']}"
                else:
                    return f"⚠️ 주문 저장 성공 (서버 전송 실패: {response.status_code})\n{get_order_summary.invoke({'tool_input': ''})['output']}"
            except Exception as e:
                return f"⚠️ 주문 저장 성공 (서버 전송 실패: {str(e)})\n{get_order_summary.invoke({'tool_input': ''})['output']}"
        except Exception as e:
            return f"❌ 저장 오류: {str(e)}"
    return "주문이 취소되었습니다"

@tool
def get_order_summary(tool_input: str = "") -> str:
    """현재 주문의 요약 정보를 반환합니다."""
    order_dict = st.session_state.order_state.get_dict()
    total = (
        order_dict["menu"]["price"]
        + order_dict["sauce"]["price"]
        + order_dict["vegetables"]["price"]
        + order_dict["cheese"]["price"]
    )
    return (
        f"=== 주문 요약 ===\n"
        f"메뉴: {order_dict['menu']['name']} ({order_dict['menu']['price']}원)\n"
        f"소스: {order_dict['sauce']['name']}\n"
        f"야채: {order_dict['vegetables']['name']}\n"
        f"치즈: {order_dict['cheese']['name']}\n"
        f"총액: {total}원"
    )

# ===================== 음성 기능 =====================
def load_whisper_model():
    """Whisper 모델을 세션 상태에 초기화합니다."""
    if "whisper_model" not in st.session_state:
        st.session_state.whisper_model = whisper.load_model("base")
    return st.session_state.whisper_model

@tool
def speech_to_text(tool_input: str = "") -> str:
    """5초간 음성 입력을 받아 텍스트로 변환합니다."""
    try:
        fs = 16000
        duration = 5
        recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
        sd.wait()
        with sf.SoundFile("temp.wav", mode='w', samplerate=fs, channels=1) as f:
            f.write(recording)
        model = load_whisper_model()
        result = model.transcribe("temp.wav", language="ko")
        return result["text"].strip()
    except Exception as e:
        return f"음성 인식 오류: {str(e)}"

def text_to_speech(text: str):
    """텍스트를 한국어 음성(mp3)으로 변환하여 재생합니다."""
    try:
        tts = gTTS(text=text, lang='ko')
        audio_bytes = io.BytesIO()
        tts.write_to_fp(audio_bytes)
        audio_bytes.seek(0)
        st.audio(audio_bytes, format='audio/mp3')
    except Exception as e:
        st.error(f"TTS 오류: {str(e)}")

# ===================== 에이전트 초기화 =====================
def initialize_agent(tools: list):
    """LangChain 에이전트 및 프롬프트 초기화"""
    llm = ChatOpenAI(
        model="gpt-4o",
        temperature=0.1,
        openai_api_key=os.getenv("OPENAI_API_KEY"),
        streaming=True
    )
    prompt = ChatPromptTemplate.from_messages([
        ("system", "너는 서보웨이 무인 주문을 도와주는 AI 어시스턴트입니다. 단계별로 차근차근 진행해주세요."),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}"),
        MessagesPlaceholder(variable_name="agent_scratchpad")
    ])
    return create_openai_tools_agent(llm, tools, prompt)

# ===================== 메인 앱 =====================
def main():
    st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
    st.image("Menu.png")

    tools = [
        get_menu_list,
        get_sauce_list,
        get_vegetable_list,
        get_cheese_list,
        update_order,
        confirm_order,
        get_order_summary,
        speech_to_text
    ]

    if "messages" not in st.session_state:
        st.session_state.messages = [AIMessage(content="무엇을 도와드릴까요?")]
    if "initialized" not in st.session_state:
        initialize_session()
        st.session_state.initialized = True
        st.session_state.order_state = OrderState()
        load_whisper_model()
        st.session_state.agent = initialize_agent(tools)
        st.session_state.agent_executor = AgentExecutor(
            agent=st.session_state.agent,
            tools=tools,
            verbose=True
        )

    # 채팅 메시지 표시
    for msg in st.session_state.messages:
        if isinstance(msg, AIMessage):
            with st.chat_message("assistant"):
                st.write(msg.content)
                text_to_speech(msg.content)
        elif isinstance(msg, HumanMessage):
            with st.chat_message("user"):
                st.write(msg.content)

    # ===== 채팅 입력란 바로 아래에 음성 버튼 배치 =====
    user_input = st.chat_input("주문을 입력하세요 (텍스트 또는 음성 버튼 사용)")
    col1, col2 = st.columns([8, 2])
    with col2:
        if st.button("🎤 음성으로 주문하기", key="voice_btn_bottom"):
            with st.spinner("🎤 5초간 말씀해주세요..."):
                # 반드시 invoke로 호출!
                voice_result = speech_to_text.invoke({"tool_input": ""})
                if voice_result and not voice_result.startswith("음성 인식 오류"):
                    st.session_state.messages.append(HumanMessage(content=voice_result))
                    st.rerun()

    # 텍스트 입력 처리
    if user_input:
        st.session_state.messages.append(HumanMessage(content=user_input))
        with st.spinner("처리 중..."):
            try:
                # 반드시 invoke로 호출!
                response = st.session_state.agent_executor.invoke({
                    "input": user_input,
                    "chat_history": st.session_state.messages
                })
                answer = response["output"] if isinstance(response, dict) else str(response)
                st.session_state.messages.append(AIMessage(content=answer))
                st.rerun()
            except Exception as e:
                st.error(f"처리 오류: {str(e)}")
                st.rerun()

if __name__ == "__main__":
    main()
