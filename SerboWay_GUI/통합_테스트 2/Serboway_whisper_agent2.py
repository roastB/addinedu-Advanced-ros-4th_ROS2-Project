import re
import streamlit as st
from typing import Dict, List, Optional, Any
import openai
import os

# LangChain 관련 임포트
from langchain_openai import ChatOpenAI
from langchain.agents import Tool, AgentExecutor, create_tool_calling_agent
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.tools import tool

# 음성 관련 임포트
import whisper
import sounddevice as sd
import soundfile as sf
from gtts import gTTS
import io

import os
from dotenv import load_dotenv
import torch

import torch
import soundfile as sf

# 통신 모듈 추가
import websockets

import json


load_dotenv()
# print(os.getenv("OPENAI_API_KEY"))  # 값이 출력되는지 확인



# --- 데이터 구조 정의 ---
MENU_DATA = {
    "불고기 샌드위치": {
        "price": 6500,
        "description": "부드러운 불고기가 들어간 샌드위치",
    },
    "새우 샌드위치": {"price": 6200, "description": "신선한 새우가 들어간 샌드위치"},
    "베이컨 샌드위치": {
        "price": 6000,
        "description": "바삭한 베이컨이 들어간 샌드위치",
    },
}

SAUCE_DATA = {
    "이탈리안": {"description": "이탈리안 스타일의 소스"},
    "칠리": {"description": "매콤한 칠리 소스"},
}

VEGETABLE_DATA = {
    "양상추": {"price": 0, "description": "기본 제공되는 양상추"},
    "로메인": {"price": 700, "description": "신선한 로메인 (+700원)"},
    "바질": {"price": 800, "description": "향긋한 바질 (+800원)"},
}

CHEESE_DATA = {
    "슬라이스 치즈": {"price": 0, "description": "기본 제공되는 슬라이스 치즈"},
    "슈레드 치즈": {"price": 1000, "description": "풍부한 슈레드 치즈 (+1000원)"},
    "모짜렐라 치즈": {"price": 1300, "description": "쫄깃한 모짜렐라 치즈 (+1300원)"},
}


# --- 주문 상태 관리 클래스 ---
class OrderState:
    def __init__(self):
        self.menu = None
        self.sauce = None
        self.vegetable = "양상추"
        self.cheese = "슬라이스 치즈"
        self.step = "menu"
        self.confirmed = False

    # OrderState 클래스 내 get_dict() 메서드 수정
    def get_dict(self):
        return {
            "menu": {
                "name": self.menu,
                "price": MENU_DATA.get(self.menu, {}).get("price", 0),
                "qty": 1
            },
            "sauce": {"name": self.sauce, "price": 0},
            "vegetables": {
                "name": self.vegetable,
                "price": VEGETABLE_DATA.get(self.vegetable, {}).get("price", 0)
            },
            "cheese": {
                "name": self.cheese,
                "price": CHEESE_DATA.get(self.cheese, {}).get("price", 0)
            },
            "step": self.step,
            "confirmed": self.confirmed
        }



    def reset(self):
        self.__init__()


# ====== 툴 정의 ======
@tool
def get_menu_list(tool_input: str = "") -> str:
    """메뉴 목록과 가격을 조회합니다."""
    result = "메뉴 목록:\n"
    for name, info in MENU_DATA.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info['price']}원 - {info['description']}\n"
    return result


@tool
def get_sauce_list(tool_input: str = "") -> str:
    """소스 목록을 조회합니다."""
    result = "소스 목록:\n"
    for name, info in SAUCE_DATA.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info['description']}\n"
    return result


@tool
def get_vegetable_list(tool_input: str = "") -> str:
    """야채 목록과 추가 가격을 조회합니다."""
    result = "야채 목록:\n"
    for name, info in VEGETABLE_DATA.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info['description']}\n"
    return result


@tool
def get_cheese_list(tool_input: str = "") -> str:
    """치즈 목록과 추가 가격을 조회합니다."""
    result = "치즈 목록:\n"
    for name, info in CHEESE_DATA.items():
        if tool_input and tool_input not in name:
            continue
        result += f"- {name}: {info['description']}\n"
    return result


@tool
def update_order(
    menu: Optional[str] = None,
    sauce: Optional[str] = None,
    vegetable: Optional[str] = None,
    cheese: Optional[str] = None,
) -> str:
    """주문 정보를 업데이트합니다."""
    order_state = st.session_state.order_state

    if menu and menu in MENU_DATA:
        order_state.menu = menu
        order_state.step = "sauce" if order_state.step == "menu" else order_state.step

    if sauce and sauce in SAUCE_DATA:
        order_state.sauce = sauce
        order_state.step = (
            "vegetable" if order_state.step == "sauce" else order_state.step
        )

    if vegetable and vegetable in VEGETABLE_DATA:
        order_state.vegetable = vegetable
        order_state.step = (
            "cheese" if order_state.step == "vegetable" else order_state.step
        )

    if cheese and cheese in CHEESE_DATA:
        order_state.cheese = cheese
        order_state.step = (
            "confirm" if order_state.step == "cheese" else order_state.step
        )

    return get_order_summary("")


@tool
def get_order_summary(tool_input: str = "") -> str:
    """현재 주문 요약 정보를 반환합니다."""
    order_state = st.session_state.order_state

    if not order_state.menu:
        return "아직 메뉴를 선택하지 않았습니다."

    base_price = MENU_DATA[order_state.menu]["price"]
    veg_price = VEGETABLE_DATA[order_state.vegetable]["price"]
    cheese_price = CHEESE_DATA[order_state.cheese]["price"]
    total = base_price + veg_price + cheese_price

    summary = (
        "테이블 1번에서 주문했습니다.\n"
        f"메뉴: {order_state.menu} ({base_price}원)\n"
        f"소스: {order_state.sauce}\n"
        f"야채: {order_state.vegetable} (+{veg_price}원)\n"
        f"치즈: {order_state.cheese} (+{cheese_price}원)\n"
        f"총 결제 금액: {total}원"
    )

    return summary


@tool
def confirm_order(confirm: bool) -> str:
    """주문을 확정하거나 취소합니다."""
    order_state = st.session_state.order_state
    if confirm:
        order_state.confirmed = True
        # 딕셔너리 구조로 저장
        with open("order_data.json", "w", encoding="utf-8") as f:   # 대문자 W -> w 로 수정
            json.dump(order_state.get_dict(), f, ensure_ascii=False)
            return f"주문이 완료되었습니다.\n{get_order_summary('')}"
    else:
        order_state.reset()
        return "🔄 주문을 처음부터 다시 시작합니다."


if "whisper_model" not in st.session_state:
    st.session_state.whisper_model = whisper.load_model("base")

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

@tool
def confirm_order(confirm: bool) -> str:
    """주문을 확정하거나 취소합니다."""
    order_state = st.session_state.order_state
    if confirm:
        order_state.confirmed = True
        
        # PyQt 앱에 주문 데이터 전송 (파일 사용)
        try:
            import json
            with open("order_data.json", "w", encoding="utf-8") as f:
                json.dump(order_state.get_dict(), f, ensure_ascii=False)
            
            # 또는 HTTP 요청 사용
            # requests.post("http://localhost:5000/order", json=order_state.get_dict())
        except Exception as e:
            print(f"❌ 주문 데이터 전송 오류: {e}")
        
        return f"✅ 주문이 완료되었습니다!\n{get_order_summary('')}\n\n결제를 진행해 주세요."
    else:
        order_state.reset()
        return "🔄 주문을 처음부터 다시 시작합니다."




# ====== Agent 초기화 ======
def initialize_agent():
    tools = [
        get_menu_list,
        get_sauce_list,
        get_vegetable_list,
        get_cheese_list,
        update_order,
        get_order_summary,
        confirm_order,
        speech_to_text,
    ]

    system_prompt = """
    당신은 서보웨이 무인 샌드위치 주문 시스템의 AI 도우미입니다.
    주문 단계에 따라 적절한 도구를 사용해 고객을 안내하세요.
    
    사용자가 각 단계에서 없는 재료를 말하면 다시 선택할 수 있도록 하세요
    각 단계 어떤 메뉴가 있는지도 안내 

    [주문 단계]
    1. 메뉴 선택 → get_menu_list 사용
    2. 소스 선택 → get_sauce_list 사용
    3. 야채 선택 → get_vegetable_list 사용
    4. 치즈 선택 → get_cheese_list 사용
    5. 주문 확인 → confirm_order 사용
    
    각 단계에서 사용자 입력을 분석해 update_order로 상태 업데이트
    주문 완료 시 confirm_order(True) 호출

    - 사용자가 "주문 내역", "가격", "요약", 등과 관련된 질문을 하면 반드시 get_order_summary 도구를 호출해 그 결과를 답변에 포함할 것
    """

    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", system_prompt),
            MessagesPlaceholder(variable_name="chat_history"),
            ("human", "{input}"),
            MessagesPlaceholder(variable_name="agent_scratchpad"),
        ]
    )

    llm = ChatOpenAI(temperature=0)
    agent = create_tool_calling_agent(llm, tools, prompt)

    return AgentExecutor(
        agent=agent, tools=tools, verbose=True, handle_parsing_errors=True
    )


# ====== Streamlit UI ======
torch.classes.__path__ = []

def main():
    st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
    st.title("🥪 서보웨이 AI 주문 시스템")
    st.image("Menu.png")

    # 세션 상태 초기화
    if "messages" not in st.session_state:
        st.session_state.messages = [
            AIMessage(
                content="어서오세요! 서보웨이에 오신 것을 환영합니다. 주문 하시겠습니까?"
            )
        ]

    if "order_state" not in st.session_state:
        st.session_state.order_state = OrderState()

    if "agent" not in st.session_state:
        st.session_state.agent = initialize_agent()

    # 채팅 메시지 출력
    for msg in st.session_state.messages:
        role = "user" if isinstance(msg, HumanMessage) else "assistant"
        with st.chat_message(role):
            st.markdown(msg.content)

    # 입력 처리
    col1, col2 = st.columns([8, 1])
    with col1:
        user_input = st.chat_input("주문을 입력하세요...")
    with col2:
        if st.button("🎤", use_container_width=True):
            with st.spinner("음성 인식 중..."):
                user_input = speech_to_text.invoke("")  # tool_input 필수

    if user_input:
        st.session_state.messages.append(HumanMessage(content=user_input))

        with st.chat_message("user"):
            st.markdown(user_input)

        with st.spinner("처리 중..."):
            result = st.session_state.agent.invoke(
                {"input": user_input, "chat_history": st.session_state.messages[:-1]}
            )

            response = result["output"]
            st.session_state.messages.append(AIMessage(content=response))

            with st.chat_message("assistant"):
                st.markdown(response)

            # TTS 출력
            tts = gTTS(response, lang="ko")
            buf = io.BytesIO()
            tts.write_to_fp(buf)
            buf.seek(0)
            st.audio(buf.read(), format="audio/mp3")


if __name__ == "__main__":
    main()
