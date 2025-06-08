import curses  # curses 라이브러리: 터미널에서 UI 표시 및 키 입력 처리를 위해 사용
import numpy as np  # NumPy: 오디오 데이터를 배열로 관리하고 처리하기 위해 사용
import sounddevice as sd  # sounddevice: 오디오 입출력 스트림(마이크, 스피커)을 제어하기 위해 사용

def _record_audio(stdscr):
    """
    터미널 UI를 사용하여 오디오를 녹음하는 함수.
    '스페이스바' 키로 녹음을 시작/중지하며, 최종적으로 녹음된 오디오 데이터를 NumPy 배열로 반환한다.
    curses 라이브러리를 통해 키 입력을 실시간으로 처리한다.
    """
    # curses 초기 설정
    curses.curs_set(0)            # 커서를 화면에 표시하지 않음 (UI를 깔끔하게 하기 위해)
    stdscr.nodelay(True)          # 키 입력을 논블로킹(non-blocking) 모드로 설정 (getch()가 즉시 반환되도록)
    curses.noecho()               # 입력한 키가 화면에 표시되지 않도록 함
    
    # 안내 메시지 출력
    stdscr.addstr(0, 0, "Press SPACE to start/stop recording, then press 'q' to quit.")  # 사용자 안내 문구 표시
    stdscr.refresh()             # 화면에 출력 내용 적용
    
    recording = False            # 현재 녹음 중인지 여부를 나타내는 플래그
    recorded_frames = []         # 녹음된 오디오 프레임들을 담을 리스트 (나중에 NumPy 배열로 결합)
    samplerate = 44100           # 오디오 샘플레이트(Hz) 설정 (1초당 44100개의 샘플)
    channels = 1                 # 오디오 채널 수 (1 = 모노, 2 = 스테레오)
    
    # 마이크로부터 오디오 입력을 받기 위한 InputStream 열기 (컨텍스트 매니저 사용으로 자동 시작/종료)
    with sd.InputStream(samplerate=samplerate, channels=channels) as stream:
        # sd.InputStream: 마이크 입력 스트림 (여기서 as stream으로 얻은 객체는 자동으로 start() 된다)
        while True:
            key = stdscr.getch()            # 키 입력 읽기 (논블로킹, 입력이 없으면 -1 반환)
            if key == ord(' '):             # 스페이스바를 누른 경우 (녹음 시작/중지 토글)
                if not recording:           # 녹음을 시작하지 않은 상태였으면
                    recording = True        # 녹음 시작 상태로 전환
                    stdscr.addstr(1, 0, "Recording...")        # 상태 메시지: 녹음 중 표시
                else:                       # 이미 녹음 중인 상태였으면 (두 번째 스페이스바 입력)
                    recording = False       # 녹음 중지 상태로 전환
                    stdscr.addstr(1, 0, "Recording stopped.")  # 상태 메시지: 녹음 중지 표시
                stdscr.clrtoeol()           # 커서 위치부터 줄 끝까지 지우기 (메시지 뒤 남은 문자 제거)
                stdscr.refresh()            # 화면에 변경 사항 갱신
            elif key == ord('q'):           # 'q' 키를 누른 경우 (녹음 종료 명령)
                break                       # 녹음 루프를 빠져나가서 종료
            if recording:
                # 녹음 중인 경우 마이크로부터 오디오 데이터를 읽어서 버퍼에 저장
                data = stream.read(1024)[0]        # 마이크 입력에서 1024 프레임(frames)의 오디오 데이터를 읽음 (NumPy 배열 반환)
                recorded_frames.append(data.copy())# 읽어온 오디오 데이터를 복사하여 리스트에 추가 (버퍼링)
                # Note: stream.read()는 (data, overflow) 튜플을 반환하며, [0]으로 오디오 데이터만 가져옴
    
    # InputStream 컨텍스트를 벗어나면 스트림이 자동으로 stop되고 리소스가 해제됨
    # 이제 recorded_frames에 저장된 여러 조각의 오디오 데이터를 하나로 합친다
    if recorded_frames:
        audio_data = np.concatenate(recorded_frames, axis=0)  # 리스트에 담긴 다수의 오디오 프레임 배열을 한 배열로 결합
    else:
        audio_data = np.empty((0, channels))                  # 녹음된 데이터가 없을 경우 빈 배열 반환 (예: 사용자가 즉시 종료한 경우)
    return audio_data  # 녹음된 전체 오디오 데이터를 NumPy 배열로 반환

def record_audio():
    """
    curses.wrapper를 사용하여 _record_audio 함수를 실행한다.
    curses.wrapper는 터미널 환경을 설정하고 _record_audio를 호출한 뒤 종료 시 원래 상태로 복원한다.
    이 함수는 _record_audio의 반환값(녹음된 오디오 데이터 배열)을 그대로 반환한다.
    """
    return curses.wrapper(_record_audio)  # _record_audio를 curses 환경에서 실행하고 그 결과(오디오 데이터)를 반환

class AudioPlayer:
    """
    오디오 재생을 위한 컨텍스트 매니저 클래스.
    sounddevice.OutputStream을 사용하여 스피커로 오디오를 출력한다.
    with 문 진입 시(__enter__) 출력 스트림을 열고, with 문 종료 시(__exit__) 스트림을 닫아 자원을 정리한다.
    add_audio() 메서드를 통해 녹음된 오디오 데이터(np.ndarray)를 재생한다.
    """
    def __init__(self, samplerate=44100, channels=1):
        self.samplerate = samplerate  # 재생에 사용할 샘플레이트 (녹음 때와 동일한 Hz)
        self.channels = channels     # 재생에 사용할 채널 수 (녹음 때와 동일해야 함)
        self.stream = None           # OutputStream 객체를 저장할 변수 (초기에는 None)
    
    def __enter__(self):
        # 오디오 출력 스트림을 생성하고 시작 (스피커 출력용)
        self.stream = sd.OutputStream(samplerate=self.samplerate, channels=self.channels)
        self.stream.start()     # 출력 스트림 시작: 이제부터 데이터 쓰면 바로 재생됨
        return self             # 자기 자신을 반환하여 with문에서 사용하게 함 (as player)
    
    def add_audio(self, audio_data: np.ndarray):
        # 녹음된 오디오 데이터를 출력 스트림에 보내 재생하는 메서드
        if self.stream:
            self.stream.write(audio_data)  # NumPy 배열로 된 오디오 데이터를 스트림에 기록 (스피커로 출력됨)
    
    def __exit__(self, exc_type, exc_value, traceback):
        # 컨텍스트 매니저 종료: 오디오 출력 스트림을 정지하고 리소스를 해제
        if self.stream:
            self.stream.stop()   # 스트림 중지 (재생 완료 후 정지)
            self.stream.close()  # 스트림 닫기 (자원 해제)
