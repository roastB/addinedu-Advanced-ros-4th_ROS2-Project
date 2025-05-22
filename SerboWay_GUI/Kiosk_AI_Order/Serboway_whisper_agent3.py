# ===================== 필수 패치 =====================
import torch
torch.classes.__path__ = []

# ===================== 표준 라이브러리 =====================
import os
import json
import io
from datetime import datetime
from typing import Dict, Any, Optional

import socket

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
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

from dotenv import load_dotenv
load_dotenv()

# ===================== 메뉴 데이터 로드 =====================
@st.cache_data(ttl=300)
def load_menu_data(json_path: str = "menu.json") -> Dict[str, Any]:
    """JSON 파일에서 메뉴, 소스, 야채, 치즈 데이터를 읽어와 반환합니다. 5분간 캐시되어 반복 호출시 성능을 높입니다."""
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        st.error(f"메뉴 로드 실패: {str(e)}")
        return {"menu": {}, "sauce": {}, "vegetable": {}, "cheese": {}}

def initialize_session():
    """Streamlit 세션 상태에 메뉴, 소스, 야채, 치즈 데이터를 저장, 앱 시작시 반드시 호출 되어야 한다."""
    data = load_menu_data()
    st.session_state.update({
        "menu_data": data.get("menu", {}),
        "sauce_data": data.get("sauce", {}),
        "vegetable_data": data.get("vegetable", {}),
        "cheese_data": data.get("cheese", {})
    })

# ================== 부분 일치 매칭 함수 ==================
def find_best_match(user_input: str, candidates: dict, threshold: float = 0.5) -> Optional[str]:
    """
    사용자의 입력(user_input)과 후보 목록(candidates)에서 부분 일치 또는 유사도가 가장 높은 항목을 반환.
    threshold는 0~1 사이(0.5 이상 권장).
    """
    import difflib
    user_input_norm = user_input.strip().lower()        # 앞뒤 공백을 제거하고 소문자로 변환하여 비교가 쉽도록 한다.
    candidates_norm = {k.lower(): k for k in candidates.keys()} # 후보군의 키도 모두 소문자로 변환해서 사전을 만듭니다.
    # 완전 일치
    if user_input_norm in candidates_norm:
        return candidates_norm[user_input_norm]
    # 부분 포함
    for norm, orig in candidates_norm.items():
        if user_input_norm in norm:
            return orig
    # 유사도 기반
    best = difflib.get_close_matches(user_input_norm, candidates_norm.keys(), n=1, cutoff=threshold)
    if best:
        return candidates_norm[best[0]]
    return None

# ================== 주문 상태 관리 클래스 ==================
class OrderState:
    """주문 상태 관리 클래스"""
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
        if not all([self.menu, self.sauce, self.vegetable, self.cheese]):
            return {
                "menu": [],
                "table_number": self.table_number,
                "timestamp": datetime.now().strftime("%Y%m%d-%H%M%S")
            }
        menu_item = {
            "name": self.menu,
            "price": (
                st.session_state.menu_data.get(self.menu, {}).get("price", 0)
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
        self.__init__(self.table_number)

# ===================== 도구 함수들 =====================
@tool
def get_menu_list(tool_input: str = "") -> str:
    """메뉴 목록과 가격을 조회합니다."""
    result = "메뉴 목록:\n"
    for name, info in st.session_state.menu_data.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info['price']}원 - {info.get('description', '')}\n"
    return result

@tool
def get_sauce_list(tool_input: str = "") -> str:
    """소스 목록을 조회합니다."""
    result = "소스 목록:\n"
    for name, info in st.session_state.sauce_data.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info.get('description', '')}\n"
    return result

@tool
def get_vegetable_list(tool_input: str = "") -> str:
    """야채 목록과 추가 가격을 조회합니다."""
    result = "야채 목록:\n"
    for name, info in st.session_state.vegetable_data.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info.get('description', '')}\n"
    return result

@tool
def get_cheese_list(tool_input: str = "") -> str:
    """치즈 목록과 추가 가격을 조회합니다."""
    result = "치즈 목록:\n"
    for name, info in st.session_state.cheese_data.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info.get('description', '')}\n"
    return result

@tool
def update_order(
    menu: Optional[str] = None,
    sauce: Optional[str] = None,
    vegetable: Optional[str] = None,
    cheese: Optional[str] = None,
) -> str:
    """주문 정보를 업데이트하고 단계별로 진행합니다. (부분 일치 및 오류 안내 포함)"""
    order_state = st.session_state.order_state

    # 메뉴 단계
    if order_state.step == "menu" and menu:
        best_menu = find_best_match(menu, st.session_state.menu_data)
        if best_menu:
            order_state.menu = best_menu
            order_state.step = "sauce"
            return f"'{menu}'(으)로 인식된 메뉴: '{best_menu}' 선택됨.\n메뉴가 선택되었습니다. 소스를 골라주세요."
        else:
            return f"'{menu}' 메뉴는 없습니다. 다시 말씀해 주세요.\n{get_menu_list('')}"
    # 소스 단계
    if order_state.step == "sauce" and sauce:
        best_sauce = find_best_match(sauce, st.session_state.sauce_data)
        if best_sauce:
            order_state.sauce = best_sauce
            order_state.step = "vegetable"
            return f"'{sauce}'(으)로 인식된 소스: '{best_sauce}' 선택됨.\n소스가 선택되었습니다. 야채를 골라주세요."
        else:
            return f"'{sauce}' 소스는 없습니다. 다시 말씀해 주세요.\n{get_sauce_list('')}"
    # 야채 단계
    if order_state.step == "vegetable" and vegetable:
        best_veg = find_best_match(vegetable, st.session_state.vegetable_data)
        if best_veg:
            order_state.vegetable = best_veg
            order_state.step = "cheese"
            return f"'{vegetable}'(으)로 인식된 야채: '{best_veg}' 선택됨.\n야채가 선택되었습니다. 치즈를 골라주세요."
        else:
            return f"'{vegetable}' 야채는 없습니다. 다시 말씀해 주세요.\n{get_vegetable_list('')}"
    # 치즈 단계
    if order_state.step == "cheese" and cheese:
        best_cheese = find_best_match(cheese, st.session_state.cheese_data)
        if best_cheese:
            order_state.cheese = best_cheese
            order_state.step = "confirm"
            return f"'{cheese}'(으)로 인식된 치즈: '{best_cheese}' 선택됨.\n치즈가 선택되었습니다. 주문을 확인해주세요."
        else:
            return f"'{cheese}' 치즈는 없습니다. 다시 말씀해 주세요.\n{get_cheese_list('')}"
    return "[DEBUG] 주문 단계에 맞는 정보를 입력해 주세요."

@tool
def get_order_summary(tool_input: str = "") -> str:
    """현재 주문 요약 정보를 반환합니다."""
    order_state = st.session_state.order_state
    if not order_state.menu:
        return "아직 메뉴를 선택하지 않았습니다."
    base_price = st.session_state.menu_data[order_state.menu]["price"]
    veg_price = st.session_state.vegetable_data.get(order_state.vegetable, {}).get("price", 0)
    cheese_price = st.session_state.cheese_data.get(order_state.cheese, {}).get("price", 0)
    total = base_price + veg_price + cheese_price
    summary = (
        f"테이블 {order_state.table_number}번에서 주문했습니다.\n"
        f"메뉴: {order_state.menu} ({base_price}원)\n"
        f"소스: {order_state.sauce}\n"
        f"야채: {order_state.vegetable} (+{veg_price}원)\n"
        f"치즈: {order_state.cheese} (+{cheese_price}원)\n"
        f"총 결제 금액: {total}원"
    )
    return summary

@tool
def send_order_to_kiosk(order_data: dict) -> str:
    """주문 정보를 키오스크로 전송합니다. (디버깅 메시지 포함)"""
    KIOSK_HOST = "192.168.0.159"  # 실제 환경에 맞게 수정
    KIOSK_PORT = 12345
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((KIOSK_HOST, KIOSK_PORT))
            client_socket.sendall(json.dumps(order_data).encode())
            response = client_socket.recv(1024).decode()
            st.write(f"[DEBUG] 키오스크 응답: {response}")
            return response
    except Exception as e:
        st.error(f"[ERROR] 키오스크 전송 오류: {str(e)}")
        return f"키오스크 전송 오류: {str(e)}"

@tool
def confirm_order(confirm: bool) -> str:
    """주문을 확정하고 키오스크로 전송합니다."""
    order_state = st.session_state.order_state
    if confirm:
        order_state.confirmed = True
        order_data = order_state.get_dict()
        st.write("[DEBUG] 전송할 주문 데이터:", order_data)
        if order_data["menu"]:
            send_result = send_order_to_kiosk.invoke(order_data)
            return (
                f"주문이 완료되었습니다.\n{get_order_summary('')}\n(키오스크 응답: {send_result})"
            )
        else:
            return "❌ 주문 정보가 불완전합니다. 모든 항목을 선택해주세요."
    else:
        order_state.reset()
        return "🔄 주문을 처음부터 다시 시작합니다."

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
        st.audio(wav_path, format="audio/wav")
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

# ===================== Whisper 모델 초기화 =====================
def load_whisper_model():
    """Whisper 모델을 세션 상태에 초기화합니다."""
    if "whisper_model" not in st.session_state:
        st.session_state.whisper_model = whisper.load_model("base")
    return st.session_state.whisper_model

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
        ("system", """당신은 서보웨이 무인 샌드위치 음성주문 시스템입니다.
주문 단계에 따라 적절한 도구를 사용해 고객을 안내하세요.
메뉴, 소스, 야채, 치즈 선택 시 json 파일에 있는 목록만 선택할 수 있습니다.
사용자가 각 단계에서 없는 재료를 말하면 다시 선택할 수 있도록 하세요.
[주문 단계]
1. 메뉴 선택 → get_menu_list 사용
2. 소스 선택 → get_sauce_list 사용
3. 야채 선택 → get_vegetable_list 사용
4. 치즈 선택 → get_cheese_list 사용
5. 주문 확인 → confirm_order 사용
각 단계에서 사용자 입력을 분석해 update_order로 상태 업데이트
주문 완료 시 send_order_to_kiosk로 kiosk에 주문정보를 전송해주세요.
- 사용자가 "주문 내역", "가격", "요약", 등과 관련된 질문을 하면 반드시 get_order_summary 도구를 호출해 그 결과를 답변에 포함하세요.
"""),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}"),
        MessagesPlaceholder(variable_name="agent_scratchpad")
    ])
    return create_tool_calling_agent(llm, tools, prompt)

# ===================== 메인 앱 (voice_agent UI 포맷) =====================
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
        send_order_to_kiosk,
        speech_to_text
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

    # 채팅 메시지 표시 (voice_agent 포맷)
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
                voice_result = speech_to_text.invoke({"tool_input": ""})
                if voice_result and not voice_result.startswith("음성 인식 오류"):
                    st.session_state.messages.append(HumanMessage(content=voice_result))
                    st.rerun()

    # 텍스트 입력 처리
    if user_input:
        st.session_state.messages.append(HumanMessage(content=user_input))
        with st.spinner("처리 중..."):
            try:
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
