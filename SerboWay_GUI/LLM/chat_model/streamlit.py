import streamlit as st
import warnings
from typing import Annotated, List
from typing_extensions import TypedDict
from dotenv import load_dotenv

from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnableConfig
from langchain_openai import ChatOpenAI
from langchain_teddynote import logging
from langchain_teddynote.graphs import visualize_graph
from langchain_teddynote.messages import random_uuid, stream_graph
from langchain_teddynote.models import LLMs, get_model_name
from langgraph.graph import END, StateGraph
from langgraph.graph.message import add_messages

# 초기 설정
load_dotenv()
warnings.filterwarnings("ignore")
logging.langsmith("test.ipynb")

MODEL_NAME = "gpt-4o"

# State 정의
class State(TypedDict):
    messages: Annotated[list, add_messages]

# 상담사 챗봇 호출 함수
def call_chatbot(messages: List[BaseMessage]) -> dict:
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You're a system that takes orders from Servoway, Answer in Korean"),
        MessagesPlaceholder(variable_name="messages")
    ])
    model = ChatOpenAI(model=MODEL_NAME, temperature=0.6)
    chain = prompt | model | StrOutputParser()
    return chain.invoke({"messages": messages})

# 고객 시나리오 생성 함수
def create_scenario(name: str, instructions: str):
    system_prompt_template = """ 당신은 서보웨이의 고객입니다.\n단일 샌드위치를 주문할 수 있고, 햄, 치즈, 양상추를 3개 제한으로 추가할 수 있습니다.\n\n[중요]\n- 주문과 관련된 대답만 해야 합니다.\n- 한국어로 대화를 해야 합니다."""
    prompt = ChatPromptTemplate.from_messages([
        ("system", system_prompt_template),
        MessagesPlaceholder(variable_name="message")
    ])
    return prompt.partial(name=name, instructions=instructions)

# 역할 스왑 함수
def _swap_roles(messages):
    new_messages = []
    for m in messages:
        if isinstance(m, AIMessage):
            new_messages.append(HumanMessage(content=m.content))
        else:
            new_messages.append(AIMessage(content=m.content))
    return new_messages

# Streamlit UI 시작
st.set_page_config(page_title="AI 상담사 시뮬레이션", page_icon="💬")
st.title("💬 AI 상담사-고객 대화 시뮬레이션")

user_input = st.text_input("👤 사용자 첫 입력:", "안녕하세요 샌드위치 주문하려고 합니다")

if st.button("▶️ 시뮬레이션 시작"):
    with st.spinner("시뮬레이션 실행 중..."):
        instructions = "단일 샌드위치를 주문하고 싶습니다."
        name = "customer"

        simulated_user = create_scenario(name, instructions) | ChatOpenAI(model=MODEL_NAME, temperature=0.1) | StrOutputParser()

        def ai_assistant_node(state: State):
            ai_response = call_chatbot(state["messages"])
            return {"messages": [("assistant", ai_response)]}

        def simulated_user_node(state: State):
            new_messages = _swap_roles(state["messages"])
            response = simulated_user.invoke({"messages": new_messages})
            return {"messages": [("user", response)]}

        def should_continue(state: State):
            if len(state["messages"]) > 6 or state["messages"][-1].content == "FINISHED":
                return "end"
            else:
                return "continue"

        graph_builder = StateGraph(State)
        graph_builder.add_node("simulated_user", simulated_user_node)
        graph_builder.add_node("ai_assistant", ai_assistant_node)
        graph_builder.add_conditional_edges(
            "simulated_user",
            should_continue,
            {
                "end": END,
                "continue": "ai_assistant",
            },
        )
        graph_builder.set_entry_point("ai_assistant")
        simulation = graph_builder.compile()

        config = RunnableConfig(recursion_limit=10, configurable={"thread_id": random_uuid()})
        inputs = {"messages": [HumanMessage(content=user_input)]}

        st.success("👏 시뮬레이션 완료! 대화 로그를 아래에서 확인하세요.")
        stream_graph(simulation, inputs, config, node_names=["simulated_user", "ai_assistant"])
