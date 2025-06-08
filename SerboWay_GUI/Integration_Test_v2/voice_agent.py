# ===================== 필수 패치 =====================
import torch
torch.classes.__path__ = []
# 일부 환경에서 whisper 등 torch extension 로딩 오류 방지용 패치

# ===================== 표준 라이브러리 =====================
import os
import json
import io
from datetime import datetime
from typing import Dict, Any, Optional
import requests   # HTTP 통신을 위한 requests 라이브러리
import socket     # PyQt 키오스크와의 TCP 통신용

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
load_dotenv()  # .env 파일에서 API 키 등 환경변수 불러오기

# ===================== 메인 서버에서 메뉴 JSON 받아오기 =====================
def fetch_menu_json_from_server(api_url: str) -> Dict[str, Any]:
    """
    메인 서버에서 최신 메뉴 JSON을 받아오는 함수
    :param api_url: 메뉴 JSON을 제공하는 메인 서버의 API 엔드포인트 URL
    :return: 메뉴 데이터(dict). 실패 시 기본 빈 구조 반환
    """
    try:
        # GET 요청으로 메뉴 데이터 받아오기
        response = requests.get(api_url, timeout=5)
        if response.status_code == 200:
            return response.json()
        else:
            st.error(f"메뉴 서버 응답 오류: {response.status_code}")
    except Exception as e:
        st.error(f"메뉴 서버 연결 실패: {str(e)}")
    # 실패 시 빈 구조 반환
    return {"menu": {}, "sauce": {}, "vegetable": {}, "cheese": {}}

@st.cache_data(ttl=300)
def load_menu_data(json_path: str = "menu.json", api_url: Optional[str] = None) -> Dict[str, Any]:
    """
    메뉴 데이터를 로컬 파일 또는 서버에서 불러옴. 서버 우선, 실패시 로컬 파일 사용.
    :param json_path: 로컬 JSON 파일 경로
    :param api_url: 서버에서 메뉴를 받아올 API URL
    :return: 메뉴 데이터(dict)
    """
    # 서버에서 받아오기 시도
    if api_url:
        data = fetch_menu_json_from_server(api_url)
        if data and data.get("menu"):
            # 받아온 데이터를 로컬에도 저장(백업)
            try:
                with open(json_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
            except Exception as e:
                st.warning(f"메뉴 백업 저장 실패: {str(e)}")
            return data
    # 서버 실패 시 로컬 파일 사용
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        st.error(f"메뉴 로드 실패: {str(e)}")
        return {"menu": {}, "sauce": {}, "vegetable": {}, "cheese": {}}

def initialize_session():
    """
    세션 상태 초기화(메뉴 데이터 등 Streamlit 세션에 저장)
    - 메인 서버에서 메뉴를 받아오고, 실패시 로컬 파일 사용
    """
    # 메인 서버 메뉴 API 주소를 환경변수 또는 고정값으로 지정
    api_url = os.getenv("MENU_SERVER_API_URL", "http://127.0.0.1:8080/api/menu_json")
    data = load_menu_data(api_url=api_url)
    st.session_state.update({
        "menu_data": data.get("menu", {}),
        "sauce_data": data.get("sauce", {}),
        "vegetable_data": data.get("vegetable", {}),
        "cheese_data": data.get("cheese", {})
    })

# ================== PyQt 키오스크로 주문 전송 함수 ==================
def send_order_to_kiosk(order_data, host='127.0.0.1', port=12345):
    """
    PyQt 키오스크로 주문 정보를 TCP로 전송하고 결제 결과를 수신
    :param order_data: dict 형태의 주문 정보
    :param host: 키오스크 서버 IP
    :param port: 키오스크 서버 포트
    :return: dict (예: {"status": "paid"} 또는 {"status": "failed"})
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        json_str = json.dumps(order_data)
        sock.sendall(json_str.encode('utf-8'))
        # 결제 결과(예: {"status": "paid"}) 수신 대기
        result = sock.recv(1024)
        result_data = json.loads(result.decode('utf-8'))
        return result_data
    except Exception as e:
        print(f"키오스크 전송 오류: {e}")
        return {"status": "error"}
    finally:
        sock.close()

# ================== 주문 상태 관리 클래스 ==================
class OrderState:
    """주문 상태 관리 클래스 (메뉴, 소스, 야채, 치즈 선택 및 주문 단계)"""
    def __init__(self):
        self.menu = None
        self.sauce = None
        self.vegetable = None
        self.cheese = None
        self.step = "menu"
        self.confirmed = False

    def get_dict(self) -> Dict[str, Any]:
        """주문 상태를 딕셔너리로 반환"""
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
    """메뉴 목록을 조회(특정 키워드 포함 메뉴만 필터링)"""
    result = "메뉴 목록:\n"
    for name, info in st.session_state.menu_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info['price']}원 ({info.get('description', '')})\n"
    return result

@tool
def get_sauce_list(tool_input: str = "") -> str:
    """소스 목록 조회(키워드 필터)"""
    result = "소스 목록:\n"
    for name, info in st.session_state.sauce_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def get_vegetable_list(tool_input: str = "") -> str:
    """야채 목록 조회(키워드 필터)"""
    result = "야채 목록:\n"
    for name, info in st.session_state.vegetable_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def get_cheese_list(tool_input: str = "") -> str:
    """치즈 목록 조회(키워드 필터)"""
    result = "치즈 목록:\n"
    for name, info in st.session_state.cheese_data.items():
        if tool_input.lower() not in name.lower():
            continue
        result += f"- {name}: {info.get('price', 0)}원\n"
    return result

@tool
def update_order(menu: Optional[str] = None, sauce: Optional[str] = None, vegetable: Optional[str] = None, cheese: Optional[str] = None) -> str:
    """주문 단계별 선택 성보를 업데이트 하고 다음 단계로 진행"""
    order_state = st.session_state.order_state
    msg = ""
    if menu and menu in st.session_state.menu_data:
        order_state.menu = menu
        order_state.step = "sauce"  # 메뉴 선택 후 소스 단계로
        msg += "메뉴가 선택되었습니다. 소스를 골라주세요.\n"
    if sauce and sauce in st.session_state.sauce_data:
        order_state.sauce = sauce
        order_state.step = "vegetable"
        msg += "소스가 선택되었습니다. 야채를 골라주세요.\n"
    # ... 이하 생략
    return msg or "주문이 업데이트되었습니다"



# confirm_order 함수 수정
@tool
def confirm_order(confirm: bool) -> str:
    """주문 정보를 키오스크로 전송하고 결제 후 저장"""
    if not confirm:
        return "주문이 취소되었습니다"
    
    try:
        order_data = st.session_state.order_state.get_dict()
        
        # 키오스크로 전송
        result = send_order_to_kiosk(order_data)
        
        if result.get("status") == "paid":
            return "✅ 결제 및 주문 완료!"
        elif result.get("status") == "failed":
            return "❌ 결제 실패"
        else:
            return "⚠️ 결제 시스템 오류"
            
    except Exception as e:
        return f"❌ 처리 오류: {str(e)}"

# TCP 통신 함수 추가
def send_order_to_kiosk(order_data, host='127.0.0.1', port=12345):
    """키오스크 서버로 주문 전송"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((host, port))
            sock.sendall(json.dumps(order_data).encode())
            response = sock.recv(1024)
            return json.loads(response.decode())
    except Exception as e:
        print(f"키오스크 통신 오류: {e}")
        return {"status": "error"}

@tool
def get_order_summary(tool_input: str = "") -> str:
    """현재 주문의 요약 반환"""
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
    """Whisper 모델을 세션 상태에 초기화(최초 1회만 로딩)"""
    if "whisper_model" not in st.session_state:
        st.session_state.whisper_model = whisper.load_model("base")
    return st.session_state.whisper_model

@tool
def speech_to_text(tool_input: str = "") -> str:
    """마이크로 4초간 음성 녹음 후 텍스트 변환"""
    try:
        fs = 16000
        duration = 4
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
    """텍스트를 한국어 음성(mp3)으로 변환 후 재생"""
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
        ("system", 
    """주문 단계에 따라 적절한 도구를 사용해 고객을 안내하세요.
    
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

    - 사용자가 "주문 내역", "가격", "요약", 등과 관련된 질문을 하면 반드시 get_order_summary 도구를 호출해 그 결과를 답변에 포함할 것."""),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}"),
        MessagesPlaceholder(variable_name="agent_scratchpad")
    ])
    return create_openai_tools_agent(llm, tools, prompt)

# ===================== 메인 앱 =====================
def main():
    st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
    st.image("Menu.png")

    # 사용할 도구(함수) 리스트
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

    # 세션 상태 초기화 및 Whisper 모델 로딩
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

    # 이전 채팅 메시지(어시스턴트/사용자) 표시 및 TTS 재생
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
