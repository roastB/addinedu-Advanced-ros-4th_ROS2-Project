import re
import streamlit as st
from langgraph.graph import StateGraph, END
from typing import TypedDict, List, Annotated, Union
from langchain_core.messages import AIMessage, HumanMessage
from langgraph.graph.message import add_messages
from gtts import gTTS
import io
import whisper
import sounddevice as sd
import soundfile as sf

# --- 가격 테이블 ---
MENU_PRICES = {
    "불고기 샌드위치": 6500,
    "새우 샌드위치": 6200,
    "베이컨 샌드위치": 6000
}
SAUCE_LIST = ["이탈리안", "칠리"]
VEGETABLE_PRICES = {"양상추": 0, "로메인": 700, "바질": 800}
CHEESE_PRICES = {"슬라이스 치즈": 0, "슈레드 치즈": 1000, "모짜렐라 치즈": 1300}

# --- 상태 관리 타입 ---
class OrderState(TypedDict):
    messages: Annotated[List[Union[AIMessage, HumanMessage]], add_messages]
    order: dict

# --- 음성 인식 함수 ---
def speech_to_text():
    st.info("말씀해주세요...", icon="🎤")
    sd.default.samplerate = 16000
    sd.default.channels = 1
    recording = sd.rec(int(5 * 16000))
    sd.wait()
    wav_path = "temp_whisper.wav"
    sf.write(wav_path, recording, 16000)
    model = whisper.load_model("base")
    result = model.transcribe(wav_path, language="ko")
    return result["text"]

# --- 주문 상태 초기화 ---
def get_initial_order():
    return {
        "menu": None,
        "sauce": None,
        "vegetable": "양상추",
        "cheese": "슬라이스 치즈",
        "step": "menu",
        "done": False,
        "confirmed": False,
    }

# --- 단계별 입력 파싱 ---
def parse_menu(text):
    for k in MENU_PRICES:
        if k[:2] in text:
            return k
    return None

def parse_sauce(text):
    for s in SAUCE_LIST:
        if s in text:
            return s
    return None

def parse_vegetable(text):
    for v in VEGETABLE_PRICES:
        if v in text:
            return v
    return None

def parse_cheese(text):
    for c in CHEESE_PRICES:
        if c in text:
            return c
    return None

# --- 주문 요약 및 가격 계산 ---
def format_order_summary(order):
    base_price = MENU_PRICES[order["menu"]] if order["menu"] else 0
    veg_price = VEGETABLE_PRICES[order["vegetable"]]
    cheese_price = CHEESE_PRICES[order["cheese"]]
    total = base_price + veg_price + cheese_price
    summary = (
        f"메뉴: {order['menu']} ({base_price}원)\n"
        f"소스: {order['sauce']}\n"
        f"야채: {order['vegetable']} (+{veg_price}원)\n"
        f"치즈: {order['cheese']} (+{cheese_price}원)\n"
        f"총 결제 금액: {total}원"
    )
    return summary

# --- 주문 처리 노드 ---
def process_order_node(state: OrderState):
    order = state["order"]
    last_msg = state["messages"][-1].content.strip()

    if order["step"] == "menu":
        menu = parse_menu(last_msg)
        if not menu:
            return {"messages": [AIMessage("메뉴를 선택해주세요. (불고기/새우/베이컨 샌드위치)")], "order": order}
        order["menu"] = menu
        order["step"] = "sauce"
        return {"messages": [AIMessage(f"{menu}를 선택하셨습니다.\n소스를 선택해주세요. (이탈리안/칠리)")], "order": order}

    if order["step"] == "sauce":
        sauce = parse_sauce(last_msg)
        if not sauce:
            return {"messages": [AIMessage("소스를 선택해주세요. (이탈리안/칠리)")], "order": order}
        order["sauce"] = sauce
        order["step"] = "vegetable"
        return {"messages": [AIMessage("야채를 선택해주세요. (양상추/로메인/바질)\n기본은 양상추. 변경 시 추가금이 있습니다.")], "order": order}

    if order["step"] == "vegetable":
        veg = parse_vegetable(last_msg)
        if not veg:
            veg = "양상추"
        order["vegetable"] = veg
        order["step"] = "cheese"
        return {"messages": [AIMessage("치즈를 선택해주세요. (슬라이스 치즈/슈레드 치즈/모짜렐라 치즈)\n기본은 슬라이스 치즈입니다. 변경 시 추가금이 있습니다.")], "order": order}

    if order["step"] == "cheese":
        cheese = parse_cheese(last_msg)
        if not cheese:
            cheese = "슬라이스 치즈"
        order["cheese"] = cheese
        order["step"] = "confirm"
        summary = format_order_summary(order)
        return {"messages": [AIMessage(f"주문 내역입니다:\n{summary}\n주문을 완료하시려면 '네'라고 입력해주세요. 취소는 '아니오'")], "order": order}

    if order["step"] == "confirm":
        if "네" in last_msg:
            order["confirmed"] = True
            summary = format_order_summary(order)
            return {"messages": [AIMessage(f"✅ 주문이 완료되었습니다!\n{summary}")], "order": order}
        elif "아니오" in last_msg:
            return {"messages": [AIMessage("🔄 주문을 처음부터 다시 시작합니다.")], "order": get_initial_order()}
        else:
            return {"messages": [AIMessage("주문을 완료하시려면 '네' 또는 '아니오'로 답해주세요.")], "order": order}

    return {"messages": [AIMessage("오류가 발생했습니다. 처음부터 다시 시도해주세요.")], "order": get_initial_order()}

# --- 라우팅 함수 ---
def route_message(state: OrderState):
    if state["order"].get("confirmed"):
        return END
    return "process"

# --- Streamlit UI ---
st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
st.title("🥪 서보웨이 AI 주문 시스템")
st.image("Menu.png")

if "messages" not in st.session_state or "order" not in st.session_state:
    st.session_state.messages = [AIMessage("어서오세요! 주문하실 샌드위치를 선택해주세요. (불고기/새우/베이컨 샌드위치)")]
    st.session_state.order = get_initial_order()

# 채팅 메시지 출력
for msg in st.session_state.messages:
    role = "user" if isinstance(msg, HumanMessage) else "assistant"
    with st.chat_message(role):
        st.markdown(msg.content)

# 입력창 및 음성 버튼
col1, col2 = st.columns([8, 1])
with col1:
    user_input = st.chat_input("주문을 입력하세요...")
with col2:
    if st.button("🎤", use_container_width=True):
        user_input = speech_to_text()
        if user_input:
            st.session_state.messages.append(HumanMessage(user_input))
            st.rerun()

# 입력이 있으면 처리
if user_input:
    st.session_state.messages.append(HumanMessage(user_input))

    workflow = StateGraph(OrderState)
    workflow.add_node("process", process_order_node)
    workflow.add_conditional_edges(
    "process",
    route_message,
    {
        "confirm": "confirm", 
        "process": "process",  # process 노드로 재귀
        END: END  # END 키 명시적 추가
    }
)

    workflow.set_entry_point("process")
    compiled_workflow = workflow.compile()

    result = compiled_workflow.invoke({"messages": st.session_state.messages, "order": st.session_state.order})
    ai_response = result["messages"][-1]
    st.session_state.messages.append(ai_response)
    st.session_state.order = result["order"]

    st.rerun()  # 응답 후 즉시 rerun하여 대화가 바로 갱신

# 마지막 메시지(assistant)가 있으면 음성 출력
if st.session_state.messages:
    last_msg = st.session_state.messages[-1]
    if isinstance(last_msg, AIMessage) and last_msg.content.strip():
        tts = gTTS(last_msg.content, lang="ko")
        buf = io.BytesIO()
        tts.write_to_fp(buf)
        buf.seek(0)
        st.audio(buf.read(), format="audio/mp3")
