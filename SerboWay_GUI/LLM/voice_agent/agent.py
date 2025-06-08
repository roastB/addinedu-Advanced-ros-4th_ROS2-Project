# 필요한 라이브러리 import
import os
import numpy as np
import wave
import asyncio
import sounddevice as sd
import openai
from dotenv import load_dotenv
import edge_tts
import simpleaudio as sa

wave_obj = sa.WaveObject.from_wave_file("response.wav")
play_obj = wave_obj.play()
play_obj.wait_done()


# OpenAI API 키 로드 (환경 변수 또는 .env 파일에서 불러오기)
load_dotenv()  # .env 파일에서 환경변수 불러오기
openai.api_key = os.getenv("OPENAI_API_KEY")  # OpenAI API 키 설정
if openai.api_key is None:
    raise Exception("OPENAI_API_KEY를 환경변수 또는 .env에 설정해주세요.")

# Whisper 및 오디오 설정
sample_rate = 16000  # 마이크 샘플링 레이트 (16kHz)
sd.default.samplerate = sample_rate
sd.default.channels = 1  # 모노 채널 사용
# 음성 녹음 종료를 위한 음량 임계치 설정
silence_threshold = 0.01  # 무음으로 판단할 RMS 임계값
silence_duration = 0.8    # 연속 무음 기간 (초) - 이 기간 만큼 무음이면 녹음 종료

# 대화 초기 설정: 시스템 메시지 정의 (챗봇에게 부여할 역할)
messages = [
    {
        "role": "system",
        "content": (
            "당신은 가상의 샌드위치 가게 '서보웨이(Servoway)'의 점원입니다. "
            "고객의 샌드위치 주문을 도와주세요. 항상 정중하고 한국어로만 응답하세요. "
            "주문과 관련된 질문에만 답변하고, 그 외의 불필요한 응답은 피하세요."
        )
    }
]

print("=== 음성 주문 에이전트 시작 ===")
print("마이크에 대고 말씀하세요. (종료하려면 '종료'라고 말씀하시거나 Ctrl+C를 누르세요)")

try:
    while True:
        # 안내 출력
        print("\n[시스템] 녹음을 시작합니다. 말씀을 멈추시면 자동으로 인식됩니다...")

        # 마이크로부터 음성 입력 녹음 시작
        recording = []  # 음성 데이터를 담을 배열
        silence_count = 0  # 무음이 지속된 chunk 수 초기화

        # sounddevice InputStream을 사용하여 실시간으로 입력 받기
        with sd.InputStream(dtype='float32') as stream:
            while True:
                # 짧은 간격으로 데이터를 읽어와 무음 여부 체크
                audio_chunk, _ = stream.read(int(sample_rate * 0.1))  # 0.1초 분량 데이터
                if audio_chunk.size == 0:
                    continue
                recording.append(audio_chunk)
                # 현재 chunk의 음량 계산 (RMS)
                volume = np.linalg.norm(audio_chunk) / np.sqrt(len(audio_chunk))
                if volume < silence_threshold:
                    # 볼륨이 임계치보다 낮으면 무음으로 간주
                    silence_count += 1
                else:
                    # 음성이 들리면 무음 카운터 리셋
                    silence_count = 0
                # 일정 시간 이상 지속적으로 무음이면 녹음 종료
                if silence_count >= int(silence_duration / 0.1):
                    break

        # numpy 배열로 변환 및 오디오 데이터를 16-bit PCM 포맷으로 저장
        audio_data = np.concatenate(recording, axis=0)
        # 녹음된 데이터가 너무 짧으면 (음성이 인식되지 않은 경우) 다시 시도
        if len(audio_data) == 0:
            print("[시스템] 음성이 인식되지 않았습니다. 다시 시도합니다...")
            continue

        # 음성 데이터를 WAV 파일로 저장 (Whisper API는 파일 입력 필요)
        # float32(-1~1)을 int16 범위(-32768~32767)로 변환하여 저장
        audio_int16 = (np.clip(audio_data, -1.0, 1.0) * 32767).astype(np.int16)
        wav_filename = "input.wav"
        wf = wave.open(wav_filename, 'wb')
        wf.setnchannels(1)            # 모노
        wf.setsampwidth(2)           # 16비트 = 2바이트
        wf.setframerate(sample_rate)
        wf.writeframes(audio_int16.tobytes())
        wf.close()

        # Whisper API를 호출하여 음성을 텍스트로 변환
        print("[시스템] Whisper 모델로 음성 인식 중...")
        with open(wav_filename, "rb") as audio_file:
            transcript = openai.Audio.transcribe(model="whisper-1", file=audio_file, language="ko")
        user_text = transcript['text'].strip()
        print(f"[사용자] {user_text}")  # 인식된 사용자 발화 출력

        # 종료 명령 처리
        if user_text == "":
            # 인식된 텍스트가 없는 경우 재시도
            print("[시스템] 음성을 제대로 인식하지 못했습니다. 다시 시도하세요.")
            continue
        if "종료" in user_text:
            # 사용자 발화에 '종료'가 포함되면 대화 종료
            print("[시스템] '종료' 명령이 확인되어 대화를 종료합니다.")
            break

        # 대화 내역에 사용자 발화 추가
        messages.append({"role": "user", "content": user_text})

        # OpenAI ChatGPT API를 호출하여 응답 생성
        print("[시스템] 챗봇 응답 생성 중...")
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",  # 또는 "gpt-4" 등 사용 가능한 모델
            messages=messages
        )
        assistant_text = response['choices'][0]['message']['content'].strip()
        # 대화 내역에 챗봇 응답 추가
        messages.append({"role": "assistant", "content": assistant_text})
        print(f"[챗봇] {assistant_text}")  # 챗봇 응답 텍스트 출력 (디버그/확인용)

        # TTS를 사용하여 챗봇 응답을 음성으로 출력
        print("[시스템] 챗봇 응답을 음성으로 변환 중...")
        tts_output_file = "response.wav"
        # Edge-TTS 사용하여 음성 합성 (ko-KR SunHiNeural 음성 사용 예시)
        async def synthesize(text, outfile):
            communicate = edge_tts.Communicate(text=text, voice="ko-KR-SunHiNeural", rate="+0%")
            await communicate.save(outfile)
        # 비동기 음성 합성 실행
        asyncio.run(synthesize(assistant_text, tts_output_file))
        # 합성된 음성 파일 재생 (플랫폼별 기본 미디어 플레이어 활용)
        playsound(tts_output_file)
        # -- 응답 재생 완료 --

        # 루프 계속 (다음 사용자 발화 대기)
        print("...대화를 계속합니다. (종료하려면 '종료'라고 말씀하세요)")

except KeyboardInterrupt:
    print("\n[시스템] 프로그램이 수동으로 종료되었습니다.")

print("=== 음성 주문 에이전트 종료 ===")
