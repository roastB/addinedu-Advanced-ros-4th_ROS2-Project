import os
import json
import io
import streamlit as st
import whisper
import sounddevice as sd
import soundfile as sf
import openai
from gtts import gTTS

from langgraph.graph import StateGraph, END
from langchain_core.messages import AIMessage, HumanMessage
from typing import TypedDict, List, Annotated, Union
from langgraph.graph.message import add_messages

# ─── Configuration ────────────────────────────────────────────────────────────

openai.api_key = os.getenv("OPENAI_API_KEY")

st.set_page_config(page_title="서보웨이 AI 주문", page_icon="🥪")
st.title("🥪 서보웨이 AI 주문 시스템")

# ─── State Type ────────────────────────────────────────────────────────────────

class OrderState(TypedDict):
    messages: Annotated[List[Union[AIMessage, HumanMessage]], add_messages]
    order: dict

# ─── Whisper STT ───────────────────────────────────────────────────────────────

def speech_to_text() -> str:
    """5초 Whisper 녹음 후 한국어로 텍스트 반환"""
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

# ─── GPT Parsing & Guidance ────────────────────────────────────────────────────

def gpt_parse_and_respond(last_text: str, history: List[dict]) -> tuple[dict, str]:
    """
    GPT function_call을 이용해
      - menu: 고정 '서보위치'
      - ingredients: ['햄','치즈','양상추'] 중 최소 1개 최대 3개
      - confirmed: bool
    을 파싱하고,
    GPT가 사용자에게 할 안내 문구도 함께 반환합니다.
    """
    functions = [{
        "name": "parse_order",
        "description": "사용자 발화에서 고정 메뉴와 재료/완료여부를 파싱합니다.",
        "parameters": {
            "type": "object",
            "properties": {
                "menu": {
                    "type": "string",
                    "enum": ["서보위치"]
                },
                "ingredients": {
                    "type": "array",
                    "items": {
                        "type": "string",
                        "enum": ["햄", "치즈", "양상추"]
                    },
                    "maxItems": 3
                },
                "confirmed": {
                    "type": "boolean"
                }
            },
            "required": ["menu", "ingredients", "confirmed"]
        }
    }]

    resp = openai.Chat.Completion.create(
        model="gpt-4o",
        messages=history + [{"role": "user", "content": last_text}],
        functions=functions,
        function_call={"name": "parse_order"}
    )

    msg = resp.choices[0].message
    args = json.loads(msg.function_call.arguments)
    reply = msg.content or ""

    return args, reply

# ─── StateGraph Nodes ─────────────────────────────────────────────────────────

def process_order_node(state: OrderState):
    last = state["messages"][-1].content
    history = [
        {"role": "assistant", "content": m.content} if isinstance(m, AIMessage)
        else {"role": "user", "content": m.content}
        for m in state["messages"]
    ]
    order, reply = gpt_parse_and_respond(last, history)
    state["order"] = order

    if not reply:
        ing = order["ingredients"]
        if not ing:
            reply = "재료를 한 가지 이상 선택해주세요."
        elif not order["confirmed"]:
            sel = ", ".join(ing)
            reply = f"{sel} 재료로 주문을 확정하시겠습니까? (네/아니오)"
        else:
            reply = "✅ 주문 완료! 매장에서 바로 준비합니다."

    return {"messages": [AIMessage(reply)], "order": order}

def confirm_order_node(state: OrderState):
    last = state["messages"][-1].content.lower()
    if "네" in last:
        state["order"]["confirmed"] = True
        return {
            "messages": [AIMessage("✅ 주문 완료! 매장에서 바로 준비합니다.")],
            "order": state["order"]
        }
    else:
        return {
            "messages": [AIMessage("🔄 주문을 다시 시작해주세요.")],
            "order": {}
        }

def route_message(state: OrderState):
    if state["order"].get("confirmed"):
        return END
    last = state["messages"][-1].content.lower()
    if any(kw in last for kw in ["네", "아니오"]):
        return "confirm"
    return "process"

# ─── Streamlit UI & Workflow ──────────────────────────────────────────────────

# 초기 환영 메시지
if "messages" not in st.session_state:
    st.session_state.messages = [AIMessage("어서오세요! 어떤 메뉴를 주문하시겠어요?")]

# 대화 기록 출력
for msg in st.session_state.messages:
    role = "user" if isinstance(msg, HumanMessage) else "assistant"
    st.chat_message(role).write(msg.content)

# 사용자 입력: 텍스트 + 음성
input_col, voice_col = st.columns([5, 1])
with input_col:
    text_input = st.chat_input("주문을 입력하세요…")
with voice_col:
    if st.button("🎤", use_container_width=True):
        text_input = speech_to_text()

if text_input:
    # 사용자 메시지 세션에 저장
    st.session_state.messages.append(HumanMessage(text_input))
    st.chat_message("user").write(text_input)

    # StateGraph 세팅
    wf = StateGraph(OrderState)
    wf.add_node("process", process_order_node)
    wf.add_node("confirm", confirm_order_node)
    wf.add_conditional_edges("process", route_message, {
        "confirm": "confirm",
        "process": END
    })
    wf.add_edge("confirm", END)
    wf.set_entry_point("process")

    # ─── 여기서 compile() 후 invoke() 해야 합니다 ──────────────────────────────
    compiled_wf = wf.compile()
    result = compiled_wf.invoke({
        "messages": st.session_state.messages,
        "order": {}
    })

    # AI 응답 표시
    ai_res = result["messages"][-1]
    st.session_state.messages.append(ai_res)
    st.chat_message("assistant").write(ai_res.content)

    # TTS: gTTS로 음성 재생
    tts = gTTS(ai_res.content, lang="ko")
    buf = io.BytesIO()
    tts.write_to_fp(buf)
    buf.seek(0)
    st.audio(buf.read(), format="audio/mp3")

    # '다시 주문' 키워드로 초기화
    if "다시 주문" in ai_res.content:
        st.session_state.messages = [
            AIMessage("새 주문을 시작합니다. 메뉴를 선택해주세요.")
        ]
        st.rerun()
