import random
from collections.abc import AsyncIterator
from typing import Callable

# 에이전트 및 음성 워크플로우 관련 클래스와 유틸리티 불러오기
from agents import Agent, Runner, TResponseInputItem, function_tool
from agents.extensions.handoff_prompt import prompt_with_handoff_instructions
from agents.voice import VoiceWorkflowBase, VoiceWorkflowHelper


# 날씨 정보를 반환하는 툴 함수 정의 (에이전트가 호출 가능)
@function_tool
def get_weather(city: str) -> str:
    """Returns the weather condition for a given city.

    Args:
        city: Name of the city.

    Returns:
        A string describing the weather in the specified city.
    """
    print(f"[debug] get_weather called with city: {city}")
    choices = ["sunny", "cloudy", "rainy", "snowy"]  # 랜덤 날씨 선택
    return f"The weather in {city} is {random.choice(choices)}."


# 한국어 에이전트 정의 (예시로 만든 별도 에이전트)
korean_agent = Agent(
    name="Korean",
    handoff_description="A Korean order agent.",  # 핸드오프 기준 설명
    instructions=prompt_with_handoff_instructions(
        "You're speaking to a human, so be polite and concise. Speak in Korean.",
    ),
    model="gpt-4o-mini",
)


# 메인 Assistant 에이전트 정의
assistant_agent = Agent(
    name="Assistant",
    instructions=prompt_with_handoff_instructions(
        "You're speaking to a human, so be polite and concise. "
        "If the user speaks in Spanish, handoff to the Spanish agent.",
    ),
    model="gpt-4o-mini",
    handoffs=[korean_agent],     # 핸드오프 가능한 에이전트 리스트
    tools=[get_weather],          # 사용할 수 있는 도구 (get_weather)
)


# 음성 기반 에이전트 워크플로우 정의
class MyWorkflow(VoiceWorkflowBase):
    """Voice workflow that responds to speech and checks for a secret word."""

    def __init__(self, secret_word: str, on_start: Callable[[str], None]):
        """Initializes the workflow.

        Args:
            secret_word: The secret word to detect (e.g., hidden trigger).
            on_start: Callback invoked at the start with the transcription.
        """
        self._input_history: list[TResponseInputItem] = []  # 사용자/에이전트 대화 기록
        self._current_agent = assistant_agent  # 현재 응답할 에이전트
        self._secret_word = secret_word.lower()  # 소문자로 변환된 시크릿 워드
        self._on_start = on_start  # 트랜스크립션 수신 시 콜백

    async def run(self, transcription: str) -> AsyncIterator[str]:
        """Executes the workflow logic.

        Args:
            transcription: User speech converted to text.

        Yields:
            Text chunks to be spoken as a response.
        """
        self._on_start(transcription)  # 음성 입력 시작 시 콜백 호출

        # 사용자 입력을 히스토리에 추가
        self._input_history.append({
            "role": "user",
            "content": transcription,
        })

        # 시크릿 워드 감지 시 별도 응답 로직 수행
        if self._secret_word in transcription.lower():
            response = "You guessed the secret word!"
            yield response
            self._input_history.append({
                "role": "assistant",
                "content": response,
            })
            return  # 핸들링 후 워크플로우 종료

        # 일반적인 흐름: 에이전트에게 입력 전달하여 응답 생성
        result = Runner.run_streamed(self._current_agent, self._input_history)

        # 응답을 스트리밍으로 출력
        async for chunk in VoiceWorkflowHelper.stream_text_from(result):
            yield chunk

        # 결과를 기준으로 히스토리와 에이전트 업데이트
        self._input_history = result.to_input_list()
        self._current_agent = result.last_agent
