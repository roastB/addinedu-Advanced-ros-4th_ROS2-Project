import streamlit as st
from audio_recorder_streamlit import audio_recorder
import openai
from elevenlabs import generate, play, set_api_key
from audio_recorder_streamlit import audio_recorder

# API 키 입력
# .env 파일의 환경변수 불러오기
load_dotenv()

# 환경변수에서 API 키 읽기
api_key = os.environ.get("OPENAI_API_KEY")

set_api_key("YOUR_ELEVENLABS_API_KEY")

st.title("음성 에이전트 데모")

# 1. 음성 녹음
audio_bytes = audio_recorder(text="마이크로 질문을 녹음하세요!", pause_threshold=2.0)

if audio_bytes:
    st.audio(audio_bytes, format="audio/wav")
    st.info("음성 인식 중...")

    # 2. Whisper로 음성 → 텍스트 변환
    transcript = openai.audio.transcriptions.create(
        model="whisper-1",
        file=audio_bytes,
        response_format="text"
    )
    st.write(f"음성 인식 결과: {transcript}")

    # 3. GPT로 답변 생성
    response = openai.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": transcript}]
    )
    answer = response.choices[0].message.content
    st.write(f"GPT 답변: {answer}")

    # 4. ElevenLabs로 답변을 음성으로 변환
    st.info("답변을 음성으로 변환 중...")
    audio = generate(
        text=answer,
        voice="Bella",  # 원하는 목소리 이름
        model="eleven_multilingual_v2"
    )
    st.audio(audio, format="audio/wav", autoplay=True)
