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
import socket
# ===================== 서드파티 라이브러리 =====================
import streamlit as st        # 웹 UI 프레임워크
import sounddevice as sd      # 마이크 입력 녹음용
import soundfile as sf        # 오디오 파일 저장/읽기
import whisper                # OpenAI 음성 인식 모델
from gtts import gTTS         # 구글 TTS(음성합성)

# ===================== LangChain 관련 =====================
from langchain_core.messages import AIMessage, HumanMessage
from langchain_community.tools import tool
from langchain_openai import ChatOpenAI
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

# ----------- 환경 변수 로드 ----------
from dotenv import load_dotenv
load_dotenv()


@st.cache_data(ttl=300)
def load_menu_data(json_path: str = "menu.json") -> Dict[str, Any]:
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
    def __init__(self, table_number=1):
        self.menu = None
        self.sauce = None
        self.vegetable = None
        self.cheese = None
        self.step = "menu"
        self.confirmed = False
        self.table_number = table_number

    def get_dict(self) -> Dict[str, Any]:
        """키오스크와 호환되는 주문 데이터 포맷으로 변환"""
        # 모든 항목이 선택된 경우에만 주문 생성
        if not all([self.menu, self.sauce, self.vegetable, self.cheese]):
            return {
                "menu": [],
                "table_number": self.table_number,
                "timestamp": datetime.now().strftime("%Y%m%d-%H%M%S")
            }
        # 단일 주문 항목을 리스트로 포장
        menu_item = {
            "name": self.menu,
            "price": (
                st.session_state.menu_data.get(self.menu, {}).get("price", 0)
                + st.session_state.sauce_data.get(self.sauce, {}).get("price", 0)
                + st.session_state.vegetable_data.get(self.vegetable, {}).get("price", 0)
                + st.session_state.cheese_data.get(self.cheese, {}).get("price", 0)
            ),
            "qty": 1,
            "sauce": self.sauce,
            "vegetable": self.vegetable,
            "cheese": self.cheese
        }
        return {
            "menu": [menu_item],
            "table_number": self.table_number,
            "timestamp": datetime.now().strftime("%Y%m%d-%H%M%S")
        }

    def reset(self):
        """주문 상태 초기화"""
        self.__init__(self.table_number)

# 5/21 디버깅 코드 클래스 추가
class OrderErrorHandler:
    ERROR_CODES = {
        100: "음성 인식 실패",
        200: "주문 데이터 불일치",
        300: "네트워크 통신 오류",
        400: "내부 시스템 오류"
    }
    
    @classmethod
    def handle(cls, error_code: int, context: str = ""):
        """에러 코드 기반 통합 처리"""
        error_msg = f"{cls.ERROR_CODES.get(error_code, '알 수 없는 오류')} [{error_code}]"
        if context:
            error_msg += f"\n상세 정보: {context}"
            
        # 오류 유형별 추가 조치
        if error_code == 100:
            st.session_state.order_state.reset()
        elif error_code == 300:
            st.cache_resource.clear()
            
        st.error(error_msg)
        text_to_speech("죄송합니다. 오류가 발생했습니다. 다시 시도해 주세요.")

# ===================== 도구 함수들 =====================
@tool
def get_menu_list(tool_input: str = "") -> str:
    """메뉴 목록을 조회합니다. 메뉴 정보가 담긴 Json 파일에서만 메뉴만 필터링합니다."""
    result = "메뉴 목록:\n"
    for name, info in st.session_state.menu_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info['price']}원 ({info.get('description', '')})\n"
    return result

@tool
def get_sauce_list(tool_input: str = "") -> str:
    """소스 목록을 조회합니다. 메뉴 정보가 담긴 Json 파일에서만 소스를 필터링합니다."""
    result = "소스 목록:\n"
    for name, info in st.session_state.sauce_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def get_vegetable_list(tool_input: str = "") -> str:
    """야채 목록을 조회합니다. 메뉴 정보가 담긴 Json 파일에서만 야채만 필터링합니다."""
    result = "야채 목록:\n"
    for name, info in st.session_state.vegetable_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def get_cheese_list(tool_input: str = "") -> str:
    """치즈 목록을 조회합니다. 메뉴 정보가 담긴 Json 파일에서만 포함된 치즈만 필터링합니다."""
    result = "치즈 목록:\n"
    for name, info in st.session_state.cheese_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def update_order(menu: Optional[str] = None, sauce: Optional[str] = None, vegetable: Optional[str] = None, cheese: Optional[str] = None) -> str:
    """주문 정보를 업데이트 합니다."""
    order_state = st.session_state.order_state

    if order_state.step == "menu" and menu and menu in st.session_state.menu_data:
        order_state.menu = menu
        order_state.step = "sauce"
        return "메뉴가 선택되었습니다. 소스를 골라주세요."

    if order_state.step == "sauce" and sauce and sauce in st.session_state.sauce_data:
        order_state.sauce = sauce
        order_state.step = "vegetable"
        return "소스가 선택되었습니다. 야채를 골라주세요."

    if order_state.step == "vegetable" and vegetable and vegetable in st.session_state.vegetable_data:
        order_state.vegetable = vegetable
        order_state.step = "cheese"
        return "야채가 선택되었습니다. 치즈를 골라주세요."

    if order_state.step == "cheese" and cheese and cheese in st.session_state.cheese_data:
        order_state.cheese = cheese
        order_state.step = "confirm"
        return "치즈가 선택되었습니다. 주문을 확인해주세요."

    return "주문 단계에 맞는 정보를 입력해 주세요."




@tool
def get_order_summary(tool_input: str = "") -> str:
    """현재 주문의 요약 정보를 반환합니다."""
    order_dict = st.session_state.order_state.get_dict()
    total = (
        order_dict["menu"]["price"]
        + order_dict["sauce"]["price"]
        + order_dict["vegetable"]["price"]
        + order_dict["cheese"]["price"]
    )
    return (
        f"=== 주문 요약 ===\n"
        f"메뉴: {order_dict['menu']['name']} ({order_dict['menu']['price']}원)\n"
        f"소스: {order_dict['sauce']['name']}\n"
        f"야채: {order_dict['vegetable']['name']}\n"
        f"치즈: {order_dict['cheese']['name']}\n"
        f"총액: {total}원"
    )


# @tool
# def confirm_order(confirm: bool) -> str:
#     """주문을 확정하고 키오스크로 전송합니다."""
#     if confirm:
#         try:
#             order_data = st.session_state.order_state.get_dict()
#             if order_data["menu"]:
#                 # 주문 정보를 키오스크로 전송
#                 send_result = send_order_to_kiosk.invoke(order_data)
#                 return (
#                     f"주문이 확정되었습니다.\n"
#                     f"{get_order_summary.invoke({'tool_input': ''})}\n"
#                     f"(키오스크 응답: {send_result})"
#                 )
#             else:
#                 return "❌ 주문 정보가 불완전합니다. 모든 항목을 선택해주세요."
#         except Exception as e:
#             return f"❌ 주문 확정 오류: {str(e)}"
#     return "주문이 취소되었습니다"


@tool
def confirm_order(confirm: bool) -> str:
    """주문을 확정하고 키오스크로 전송합니다."""
    if confirm:
        try:
            order_data = st.session_state.order_state.get_dict()
            print("전송할 주문 데이터:", order_data)  # 🟡 추가
            if order_data["menu"]:
                send_result = send_order_to_kiosk.invoke(order_data)
                print("키오스크 응답:", send_result)  # 🟡 추가
                return (
                    f"주문이 확정되었습니다.\n"
                    f"{get_order_summary.invoke({'tool_input': ''})}\n"
                    f"(키오스크 응답: {send_result})"
                )
            else:
                return "❌ 주문 정보가 불완전합니다. 모든 항목을 선택해주세요."
        except Exception as e:
            return f"❌ 주문 확정 중 오류 발생: {str(e)}"
    return "주문이 취소되었습니다"


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




# ===================== 음성 기능 =====================
def load_whisper_model():
    """Whisper 모델을 세션 상태에 초기화합니다."""
    if "whisper_model" not in st.session_state:
        st.session_state.whisper_model = whisper.load_model("base")
    return st.session_state.whisper_model

# @tool
# def speech_to_text(tool_input: str = "") -> str:
#     """4초간 음성 입력을 받아 텍스트로 변환합니다."""
#     try:
#         fs = 16000
#         duration = 4
#         recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
#         sd.wait()
#         with sf.SoundFile("temp.wav", mode='w', samplerate=fs, channels=1) as f:
#             f.write(recording)
#         model = load_whisper_model()
#         result = model.transcribe("temp.wav", language="ko")
#         return result["text"].strip()
#     except Exception as e:
#         return f"음성 인식 오류: {str(e)}"

# 개선된 Whisper 모델 로딩 함수
# @st.cache_resource(ttl=3600, show_spinner="음성 모델 초기화 중...")
# def load_whisper_model_v2():
#     """GPU 가속 및 모델 검증 기능 추가"""
#     try:
#         model = whisper.load_model(
#             "small",
#             device="cuda" if torch.cuda.is_available() else "cpu"
#         )
#         # 모델 검증
#         test_audio = torch.randn(16000 * 3)  # 3초 더미 데이터
#         _ = model.transcribe(test_audio)
#         return model
#     except Exception as e:
#         st.error(f"모델 초기화 실패: {str(e)}")
#         raise RuntimeError("음성 모델 로딩 실패") from e
@tool
def speech_to_text(tool_input: str = "") -> str:
    """음성을 텍스트로 변환합니다."""
    try:
        st.info("말씀해주세요", icon="🎤")
        sd.default.samplerate = 16000
        sd.default.channels = 1
        recording = sd.rec(int(3 * 16000))
        sd.wait()
        wav_path = "temp_whisper.wav"
        sf.write(wav_path, recording, 16000)
        # 녹음된 오디오 직접 확인
        st.audio(wav_path, format="audio/wav")
        # Whisper 모델 가져오기
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model = st.session_state.whisper_model
        result = model.transcribe(
            wav_path,
            language="ko",
            fp16=True if device == "cuda" else False,
            temperature=0.1,
            best_of=1,
            beam_size=1
        )
        text = result.get("text", "").strip()
        if not text:
            return "음성이 인식되지 않았습니다. 다시 시도해 주세요."
        return text
    except Exception as e:
        return f"음성 인식 중 오류 발생: {str(e)}"
    
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
        temperature=0,
        openai_api_key=os.getenv("OPENAI_API_KEY"),
        streaming=True
    )
    prompt = ChatPromptTemplate.from_messages([
        ("system", """ 당신은 서보웨이 무인 샌드위치 주문 시스템의 AI 도우미입니다.
    주문 단계에 따라 적절한 도구를 사용해 고객을 안내하세요.
    각 단계는 다음과 같습니다:
    1. 메뉴 선택 → get_menu_list 사용
    2. 소스 선택 → get_sauce_list 사용
    3. 야채 선택 → get_vegetable_list 사용
    4. 치즈 선택 → get_cheese_list 사용
    5. 주문 확인 → confirm_order 사용

    각 단계에서 사용자가 올바른 재료를 선택할 때까지 반복해서 안내하고, 선택이 완료되면 다음 단계로 넘어가세요.
    항상 update_order로 상태를 업데이트하세요.
    ."""),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}"),
        MessagesPlaceholder(variable_name="agent_scratchpad")
    ])
    return create_openai_tools_agent(llm, tools, prompt)

# ===================== 메인 앱 =====================
# def main():
#     st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
#     st.image("image/Menu.png")

#     tools = [
#         get_menu_list,
#         get_sauce_list,
#         get_vegetable_list,
#         get_cheese_list,
#         update_order,
#         confirm_order,
#         get_order_summary,
#         # speech_to_text,
#         send_order_to_kiosk
#     ]

#     if "messages" not in st.session_state:
#         st.session_state.messages = [AIMessage(content="무엇을 도와드릴까요?")]
#     if "initialized" not in st.session_state:
#         initialize_session()
#         st.session_state.initialized = True
#         st.session_state.order_state = OrderState()
#         load_whisper_model()
#         st.session_state.agent = initialize_agent(tools)
#         st.session_state.agent_executor = AgentExecutor(
#             agent=st.session_state.agent,
#             tools=tools,
#             verbose=True
#         )

#     # 채팅 메시지 표시
#     for msg in st.session_state.messages:
#         if isinstance(msg, AIMessage):
#             with st.chat_message("assistant"):
#                 st.write(msg.content)
#                 text_to_speech(msg.content)
#         elif isinstance(msg, HumanMessage):
#             with st.chat_message("user"):
#                 st.write(msg.content)

#     # ===== 채팅 입력란 바로 아래에 음성 버튼 배치 =====
#     user_input = st.chat_input("주문을 입력하세요 (텍스트 또는 음성 버튼 사용)")
#     col1, col2 = st.columns([8, 2])
#     with col2:
#         if st.button("🎤 음성으로 주문하기", key="voice_btn_bottom"):
#             with st.spinner("🎤 5초간 말씀해주세요..."):
#                 # 반드시 invoke로 호출!
#                 voice_result = speech_to_text.invoke({"tool_input": ""})
#                 if voice_result and not voice_result.startswith("음성 인식 오류"):
#                     st.session_state.messages.append(HumanMessage(content=voice_result))
#                     st.rerun()

#     # 텍스트 입력 처리
#     if user_input:
#         st.session_state.messages.append(HumanMessage(content=user_input))
#         with st.spinner("처리 중..."):
#             try:
#                 # 반드시 invoke로 호출!
#                 response = st.session_state.agent_executor.invoke({
#                     "input": user_input,
#                     "chat_history": st.session_state.messages
#                 })
#                 answer = response["output"] if isinstance(response, dict) else str(response)
#                 st.session_state.messages.append(AIMessage(content=answer))
#                 st.rerun()
#             except Exception as e:
#                 st.error(f"처리 오류: {str(e)}")
#                 st.rerun()
def main():
    st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
    st.image("image/Menu.png")

    tools = [
        get_menu_list,
        get_sauce_list,
        get_vegetable_list,
        get_cheese_list,
        update_order,
        confirm_order,
        get_order_summary,
        # speech_to_text,
        send_order_to_kiosk
    ]

    # 세션 상태 및 에이전트 초기화
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
