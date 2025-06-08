#!/usr/bin/env python
# coding: utf-8

from typing import Annotated, List
import warnings

from dotenv import load_dotenv
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder  # 수정: ChatPromptTemplate 추가
from langchain_core.runnables import RunnableConfig
from langchain_openai import ChatOpenAI  # 추가: ChatOpenAI import
from langchain_teddynote import logging
from langchain_teddynote.graphs import visualize_graph
from langchain_teddynote.messages import random_uuid, stream_graph
from langchain_teddynote.models import LLMs, get_model_name
from langgraph.graph import END, StateGraph
from langgraph.graph.message import add_messages
from typing_extensions import TypedDict

# In[65]:
# .env 파일에서 환경변수 로드
# API key를 환경변수로 관리하기 위한 설정 파일
# API Key 정보 로드
load_dotenv()

# In[66]:
# LangSmith 추적 활성화
# LangSmith 추적 허용
# projetct name
logging.langsmith("test.ipynb")

# In[67]:
# 경고 메시지 무시 설정
warnings.filterwarnings("ignore")

# **상담사 역할 정의**
# 시뮬레이션에서 상담사 역할을 하는 챗봇을 정의합니다.
# 참고
# * call_chatbot 내의 구현은 설정 가능하며, 내부에서 사용한 모델을 Agent 로 변경하는 것도 가능합니다.
# * call_chatbot 은 사용자로부터 메시지를 입력으로 받아, 고객을 상담하는 역할을 부여하겠습니다.
# * 고객 지원 시나리오에서의 대화 응답 생성에 활용될 수 있습니다.

# In[68]:
# State 클래스 정의 (대화 메시지 상태 저장)
# State 정의
class State(TypedDict):
    messages: Annotated[list, add_messages]  # 사용자 - 상담사 간의 대화 메시지

# ## 상담사 역할 정의

# In[69]:
# 상담사 챗봇 응답 함수(call_chatbot) 정의
# model name (임시 테스트)
MODEL_NAME = "gpt-4o"  # get_model_name(LLMs) 대신 직접 지정

def call_chatbot(messages: List[BaseMessage]) -> dict:
    # 프롬프트 템플릿 설정
    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", "You're a system that takes orders from Servoway, Answer in Korean"),
            MessagesPlaceholder(variable_name="messages")
        ]
    )
    # 모델 초기화 (오타 수정)
    model = ChatOpenAI(model=MODEL_NAME, temperature=0.6)
    # 체인 구성
    chain = prompt | model | StrOutputParser()
    # 실행
    return chain.invoke({"messages": messages})  # 수정: "message" → "messages"

# In[70]:
# 상담사 챗봇 함수 테스트 호출
call_chatbot(["user", "샌드위치 한 개 주세요"])

# ## 고객 역할 정의

# In[71]:
# 고객 시나리오 생성 함수 정의
def create_scenario(name: str, instructions: str):
    # system 프롬프트를 정의: 필요에 따라 변경
    system_prompt_template = """ 당신은 서보웨이의 고객입니다.
    단일 샌드위치를 주문할 수 있고, 햄, 치즈, 양사추를 3개 제한으로 추가할 수 있습니다.

    [중요]
    - 주문과 관련된 대답만 해야합니다.
    - 한국어로 대화를 해야 합니다.
"""
    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", system_prompt_template),
            MessagesPlaceholder(variable_name="message"),
        ]
    )
    prompt = prompt.partial(name=name, instructions=instructions)
    return prompt

# In[72]:
# 예시 지시사항으로 시나리오 생성 및 출력
instructions = """당신은 무인 샌드위치 매장 '서보웨이'의 고객입니다.  
이 매장은 24시간 무인으로 운영되며, 직원이 없습니다.  
고객은 원하는 샌드위치와 음료를 직접 선택하고, 키오스크에서 바코드를 찍어 결제합니다.  
매장에는 다양한 샌드위치(예: 오리지널 햄, 치즈, 햄치즈, 옥수수크림, 트리플치즈 등)와 음료, 간식류가 있습니다.  
샌드위치는 단일 메뉴로, **햄**, **치즈**, **양상추** 중 원하는 재료를 최대 3개까지 선택할 수 있습니다.

[시나리오 지시사항]
- 당신은 샌드위치를 주문하고 싶습니다.
- 키오스크 사용이 처음이라 궁금한 점이 있거나, 바코드 인식이 잘 안 되는 등 불편함이 있으면 적극적으로 질문하거나 도움을 요청하세요.
- 결제, 재고, 메뉴 선택, 포장, 영수증 발급 등 무인매장에서 실제로 겪을 수 있는 상황을 자연스럽게 시도하고, 필요한 경우 불편을 표현하세요.
- 답변은 반드시 한국어로 하세요.
- 주문과 관련 없는 대화는 하지 마세요.

[예시 상황]
- “샌드위치에 치즈 두 장, 햄 한 장 추가할 수 있나요?”
- “키오스크에서 바코드가 인식이 안 돼요. 어떻게 해야 하죠?”
- “포장도 가능한가요?”
- “재고가 없을 때는 어떻게 해야 하나요?”
- “결제는 카드만 가능한가요, 현금도 되나요?”

[목표]
- 무인 매장에서 실제 고객이 겪을 수 있는 다양한 주문·이용 경험을 시뮬레이션합니다."""
name = "bear"
create_scenario(name, instructions).pretty_print()

# In[73]:
# OpenAI 모델 초기화 및 시뮬레이션 사용자 생성
model = ChatOpenAI(model=MODEL_NAME, temperature=0.1)
simulated_user = create_scenario(name, instructions) | model | StrOutputParser()

# In[74]:
# 시뮬레이션 사용자에게 초기 메시지 전달 및 응답 생성
messages = [HumanMessage(content="안녕하세요? Serboway 입니다.")]
simulated_user.invoke({"message": messages})

# ## 에이전트 시뮬레이션 정의

# In[75]:
# AI 상담사 노드 함수 정의 (사용자 메시지 응답)
def ai_assistant_node(messages):
    # 상담사 응답 호출
    ai_response = call_chatbot(messages)
    # AI 상담사의 응답을 반환
    return {"messages": [("assistant", ai_response)]}

# In[76]:
# 예시 대화로 AI 상담사 노드 동작 테스트
ai_assistant_node(
    [
        ("user", "안녕하세요"),
        ("assistant", "안녕하세요! 무엇을 주문하시겠습니까?"),
        ("user", "어떤 메뉴가 있을까요?")
    ]
)

# In[77]:
# 시뮬레이션을 위한 역할 교환 함수 및 노드 정의
def _swap_roles(messages):
    # 메시지의 역할을 교환: 시뮬레이션 사용자 단계에서 메시지 타입을 AI -> Human, Human -> AI 로 교환합니다.
    new_messages = []
    for m in messages:
        if isinstance(m, AIMessage):
            # AIMessage 인 경우, HumanMessage 로 변환합니다.
            new_messages.append(HumanMessage(content=m.content))
        else:
            # HumanMessage 인 경우, AIMessage 로 변환합니다.
            new_messages.append(AIMessage(content=m.content))
    return new_messages

def ai_assistant_node(state: State):
    # 상담사 응답 호출
    ai_response = call_chatbot(state["messages"])
    # AI 상담사의 응답을 반환
    return {"messages": [("assistant", ai_response)]}

def simulated_user_node(state: State):
    # 메시지 타입을 교환: AI -> Human, Human -> AI
    new_messages = _swap_roles(state["messages"])
    # 시뮬레이션된 사용자를 호출
    response = simulated_user.invoke({"messages": new_messages})
    return {"messages": [("user", response)]}

# In[78]:
# 대화 종료 조건 판단 함수 정의
def should_continue(state: State):
    # 메시지 리스트의 길이가 6보다 크면 'end'를 반환합니다.
    if len(state["messages"]) > 6:
        return "end"
    # 마지막 메시지의 내용이 'FINISHED'라면 'end'를 반환합니다.
    elif state["messages"][-1].content == "FINISHED":
        return "end"
    # 위의 조건에 해당하지 않으면 'continue'를 반환합니다.
    else:
        return "continue"

# In[79]:
# 대화 시뮬레이션 그래프 구성 및 컴파일
graph_builder = StateGraph(State)  # grape_builder → graph_builder로 수정
graph_builder.add_node("simulated_user", simulated_user_node)
graph_builder.add_node("ai_assistant", ai_assistant_node)
graph_builder.add_conditional_edges(
    "simulated_user",
    should_continue,
    {
        "end": END,  # 종료 조건이 충족되면 시뮬레이션을 중단
        "continue": "ai_assistant",  # 종료 조건이 충족되지 않으면 상담사 역할 노드로 메시지를 전달.
    },
)
graph_builder.set_entry_point("ai_assistant")
simulation = graph_builder.compile()

# In[80]:
# 시뮬레이션 그래프 시각화
visualize_graph(simulation)

# In[81]:
# 시뮬레이션 실행을 위한 설정 및 그래프 스트리밍
config = RunnableConfig(recursion_limit=10, configurable={"thread_id": random_uuid()})
inputs = {
    "messages": [HumanMessage(content="안녕하세요 샌드위치 주문하려고 합니다")]
}
stream_graph(simulation, inputs, config, node_names=["simulated_user", "ai_assistant"])
