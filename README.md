![배너](https://github.com/user-attachments/assets/2f572254-e9e0-4c69-803c-64fc96cfd2ba)

## 🎬 프로젝트 영상

> **👉 [통합 시연 영상](https://www.youtube.com/watch?v=tyQN5nz1XKo) 🥪✨**  
> *: 음성 및 터치로 샌드위치를 주문하고, 조리부터 서빙·관제까지 자동으로 수행하는 통합 F&B 로봇 서비스 시스템*

# 1. 프로젝트 개요
![1-1.배경](https://github.com/user-attachments/assets/88fd2122-2859-4f51-a76a-6df3a9d204eb)
![1-2.목표](https://github.com/user-attachments/assets/4be521ec-6397-4555-8637-ff225889b253)

# 2. 개발 정보
## 2-1. 기술 스택
| **분류** | **기술** |
|:--------:|---------|
| **개발환경** | ![Ubuntu](https://img.shields.io/badge/Ubuntu%2024.04-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white) ![ROS2](https://img.shields.io/badge/ROS2%20Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white) |
| **언어/DB** | ![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white) ![C](https://img.shields.io/badge/C-A8B9CC?style=for-the-badge&logo=C&logoColor=white) ![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white) ![MySQL](https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=MySQL&logoColor=white) |
| **라이브러리/API** | ![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white) ![PyQt](https://img.shields.io/badge/PyQt-41CD52?style=for-the-badge&logo=qt&logoColor=white) ![OpenAI](https://img.shields.io/badge/OpenAI%20API-412991?style=for-the-badge&logo=openai&logoColor=white) ![LangChain](https://img.shields.io/badge/LangChain-0097A7?style=for-the-badge&logo=langchain&logoColor=white) |
| **하드웨어** | ![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi%205-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white) ![Arduino](https://img.shields.io/badge/Arduino%20Nano-00878F?style=for-the-badge&logo=arduino&logoColor=white) ![Jetcobot](https://img.shields.io/badge/Jetcobot%20280%20M5-00A82D?style=for-the-badge&logo=probot&logoColor=white) ![Pinky](https://img.shields.io/badge/Pinky-FF3850?style=for-the-badge&logo=probot&logoColor=white) |
| **협업도구** | ![Git](https://img.shields.io/badge/Git-F05032?style=for-the-badge&logo=git&logoColor=white) ![Confluence](https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=confluence&logoColor=white) ![Jira](https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=jira&logoColor=white) ![Slack](https://img.shields.io/badge/Slack-4A154B?style=for-the-badge&logo=slack&logoColor=white) |

## 2-2. 팀원 구성 및 역할
| 역할 | 이름 | 주요 담당업무 |
|------|------|---------------|
| **팀장** | 조연하 | **시스템 설계 및 프로젝트 관리** <br/> • SW/HW 아키텍처 설계 및 기능 명세서 작성 <br/> • 근접센서 기반 픽업 확인 시스템 개발 <br/> • 주행로봇 인터페이스 설계 및 구현 <br/> • 프로젝트 관리 도구(Jira, Confluence) 운영 및 업무 할당 관리 |
| **팀원** | 조성현 | **통합 통신 시스템 및 메인 서버 개발** <br/> • 시스템 간 통신 프로토콜 설계 및 구현 <br/> • ROS2 기반 Main Control Server 개발 <br/> • 작업 스케줄링 및 할당 알고리즘 구현 <br/> • Jetcobot 카메라 캘리브레이션 시스템 구축 |
| **팀원** | 모민규 | **AI 음성인식 및 사용자 인터페이스 개발** <br/> • ChatGPT API 기반 음성인식 주문 시스템 구현 <br/> • Streamlit 프레임워크를 활용한 사용자 키오스크 개발 <br/> • 소스코드 버전 및 GitHub 관리 |
| **팀원** | 장윤정 | **관리자 시스템 및 데이터베이스 개발** <br/> • PyQt5 기반 관리자 모니터링 시스템 구축 <br/> • 사용자 키오스크 UI/UX 설계 및 구현 <br/> • 데이터베이스 스키마 설계 및 데이터 관리 시스템 개발 |
| **팀원** | 정희준 | **자율주행 및 경로계획 시스템 개발** <br/> • ROS2 기반 동적 경로 생성 알고리즘 구현 <br/> • 다중 로봇 제어를 위한 경로 최적화 시스템 개발 <br/> • 실시간 장애물 감지 및 회피 기능 구현 |
| **팀원** | 함수린 | **로봇 비전 및 자동화 시스템 개발** <br/> • 컴퓨터 비전 기반 색상 인식 및 좌표 변환 시스템 구현 <br/> • Jetcobot 기반 샌드위치 자동 제조 시스템 개발 <br/> • 식기 수거 및 정리 자동화 기능 구현 <br/> • 빈 그릇 수거를 위한 통합 통신 시스템 구현 |

# 3. 시스템 설계
![3-1.기능리스트](https://github.com/user-attachments/assets/a1016e74-0ea3-4867-8f6b-fa7804f8d777)
![3-2.SW아키텍쳐](https://github.com/user-attachments/assets/2305e8bf-13d3-44a6-b60f-c04be4f1f87c)
![3-3.MainControlServer상세설계](https://github.com/user-attachments/assets/0c665271-b265-44dc-964e-c1f19dcf2c61)
## 전체 시스템 맵
![4-0.system_overview_map](https://github.com/user-attachments/assets/29bda9a3-1cf6-4f45-a47f-574c0bb33dd2)

# 4. 기능 구현

## 4-1. 주문 시스템
### 4-1-1. 터치 주문
![4-1.터치 주문](https://github.com/user-attachments/assets/4781298d-91c0-4bbf-97aa-8e6744e52601)

### 4-1-2. 음성 주문
*[음성 주문 관련 GIF 또는 설명 추가 예정]*

### 4-1-3. 주문 접수 및 작업 할당
![04_terminal_order_received_and_task_assigned](https://github.com/user-attachments/assets/42971dd2-2038-4e0e-9235-91d59e4dbcfb)

## 4-2. 자동화 제조 및 배송 프로세스

### 4-2-1. Pinky 로봇 제조 위치 이동
![4-2-1.핑키 제조 위치로 이동](https://github.com/user-attachments/assets/7f8c0d88-2cae-42fe-aa30-aedb2b7f86c3)

### 4-2-2. Jetcobot 샌드위치 제조 시작
![4-2-2.샌드위치 제조 시작](https://github.com/user-attachments/assets/88f5350a-f8c8-4cc0-bf9f-4d003865b8a0)

### 4-2-3. Main Control Server 제조 진행 상태 모니터링
![05_terminal_making_in_progress](https://github.com/user-attachments/assets/fb3af0a9-f832-435b-a3cd-f3bef28de05f)

### 4-2-4. Jetcobot 샌드위치를 Pinky 적재함으로 푸시
![4-2-4. 샌드위치 적재함으로 푸시](https://github.com/user-attachments/assets/45463ee2-482f-4d4c-9ea9-9f44c9c65cce)

### 4-2-5. Main Control Server Pinky 적재함으로 푸시 명령
![06_terminal_ready_to_deliver_push_command](https://github.com/user-attachments/assets/26d47864-6e7c-4c1c-b7d0-e82f3f614f70)

### 4-2-6. Pinky 로봇 서빙 위치로 배송
![4-2-6.핑키 서빙](https://github.com/user-attachments/assets/da29d63a-117e-4179-83c5-8f9eb9596a6d)

### 4-2-7. Main Control Server 배송 준비 완료시 서빙 명령 및 배송 완료 처리
![07_terminal_push_complete_and_delivery_command](https://github.com/user-attachments/assets/d53b8402-750c-41ff-8f10-37277551c7c7)


# 5. 주요 기술 (Main Control Server)
### 5-1. **작업 할당**
> 메인 컨트롤 서버에서 주문 버퍼를 확인한 뒤, 로봇팔과 주행로봇의 상태 및 위치 정보를 바탕으로 작업을 동적으로 할당합니다. 샌드위치 제조, 서빙, 빈그릇 수거 등 각 작업별로 최적의 로봇을 선정하며, 효율적인 운영을 위해 작업 우선순위와 로봇 간 거리 등을 고려합니다.

![5-1-1.작업할당](https://github.com/user-attachments/assets/00a12934-9d5e-49a4-9acf-2fbb79249a0a)
![5-1-2.작업할당](https://github.com/user-attachments/assets/af37a66e-8705-48fe-90f4-aec056afba06)
![5-1-3.작업할당](https://github.com/user-attachments/assets/e1bc890c-0aef-4389-ba9b-d9511e490f59)
![5-1-4.작업할당](https://github.com/user-attachments/assets/3d9bb9e7-3a1e-49fb-a204-7ff7e85186d0)
![5-1-5.작업할당](https://github.com/user-attachments/assets/880e2862-ede6-4cac-8940-f9d62cc1ed93)
![5-1-6.작업할당](https://github.com/user-attachments/assets/572b095e-0e7b-4aaa-92fe-c10a00392443)

### 5-2. **경로 생성 알고리즘**
> 맵의 꼭지점 마커를 기준으로 200x100 크기의 좌표계를 설정한 뒤, 주요 노드와 중간 노드를 생성합니다. 각 주행로봇은 DFS(깊이 우선 탐색) 알고리즘을 활용해 가능한 모든 경로를 탐색하고, 최단 경로를 선택해 샌드위치 배달 및 빈그릇 수거를 수행합니다. 장애물 감지 시 우선순위 기반의 회피 경로를 동적으로 생성하여 안전한 주행을 보장합니다. 

### 5-3. **LLM 기반 음성 인식**
> Langchain Agent, OpenAI Whisper, GPT-4o, gTTS를 결합하여 음성 주문 시스템을 구축하였습니다. Whisper로 고객 음성을 텍스트로 변환하고, Langchain과 GPT-4o를 활용해 자연어 이해 및 주문 의도 파악, 메뉴 안내 메시지 생성을 처리합니다. gTTS로 안내 메시지를 자연스러운 음성으로 변환하여, 키오스크와 연동되는 음성 기반 주문 경험을 제공합니다.
> 
### 5-4. **색상 인식 기반 좌표 변환**
> 카메라로 인식한 마커의 좌표를 변환행렬을 통해 로봇팔 좌표계로 변환합니다. HSV 색공간에서 특정 색상을 추출하고, 모폴로지 연산(잡음 제거 및 윤곽 강조)을 적용해 사각형 마커를 검출합니다. 이를 통해 재료의 위치를 정확하게 파악하고, 로봇팔이 해당 위치로 이동할 수 있도록 좌표를 변환합니다

