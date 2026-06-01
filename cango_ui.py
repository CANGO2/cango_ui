import asyncio
import math
from nicegui import app, ui
import rclpy
from rclpy.node import Node

# 커스텀 메시지 포맷 임포트
from cango_msgs.msg import RobotControl, LlmRequest
from std_msgs.msg import String, Float32


class RobotWebUI(Node):

    def __init__(self):
        super().__init__("robot_web_ui")

        # --- [1] ROS 2 토픽 구독 설정 ---
        # 위치/대화 관련 정보는 /master2llm, 조종 모드는 /master2control 토픽에서 수신합니다.
        self.llm_request_sub = self.create_subscription(
            LlmRequest, "/master2llm", self.llm_request_callback, 10
        )

        self.control_sub = self.create_subscription(
            RobotControl, "/master2control", self.control_callback, 10
        )
        
        # RMD 모터 실시간 전류값 수신 토픽 (우측 상단 그래프 렌더링용)
        self.current_sub = self.create_subscription(
            Float32, "/rmd_motor_current", self.motor_current_callback, 10
        )

        # 조이스틱/레버 피드백 화면용 가상 데이터 바인딩 변수
        self.ui_joystick_linear = 0.0
        self.ui_joystick_side = 0.0
        self.ui_lever_linear = 0.0

        # --- [2] 상태 관리 내부 변수 ---
        self.is_auto = False   # RobotControl.mode == 0 일 때 True (Auto Driving)
        self.is_stand = False  # LlmRequest.stand == 1 일 때 True (Stand)

        self.start_location = ""  # local_candi1 ~ local_candi2 형태 가공 매핑 변수
        self.goal_location = ""   # goalpoint 목적지 변수

        # SVG 동적 컴포넌트 렌더링용 데이터 문자열
        self.joystick_svg = ""
        self.lever_svg = ""
        self.robot_vector_svg = ""

        # LLM 기본 로깅 데이터 구조 초기화
        self.llm_messages = [
            {"text": "시스템이 시작되었습니다. CANGO 로봇 명령 대기 중.", "sent": False}
        ]

        # 초기 그래픽 및 데이터 셋 바인딩 후 웹 엔진 빌드
        self.update_graphics()
        self.update_robot_vector(0.0, 0.0)
        self.build_ui()

    # --- [4] 모든 대시보드 인터락이 집중된 통합 콜백 제어 함수 ---
    def control_callback(self, msg):
        """ /master2control (RobotControl) 토픽에서 조종/자율 모드를 동기화 """

        # RobotControl.mode: 1 = 조종, 0 = 자율
        self.is_auto = (msg.mode == 0)

        if self.is_auto:
            self.ui_joystick_linear, self.ui_joystick_side = 0.0, 0.0
        else:
            self.ui_lever_linear = 0.0
        self.update_graphics()

        self.render_top_buttons.refresh()
        self.render_control_panel.refresh()
        ui.update()

    def llm_request_callback(self, msg):
        """ /master2llm (LlmRequest) 토픽을 단일 수신하여 모든 UI 상태를 동기화 """

        # 1) 기립 상태 판단 (LlmRequest 메시지 내의 stand가 1일 때 일어남 상태 활성화)
        self.is_stand = (msg.stand == 1)

        # 3) 출발 위치 가공 처리: "local_candi1 ~ local_candi2"
        if msg.local_candi1 and msg.local_candi2:
            self.start_location = f"{msg.local_candi1} ~ {msg.local_candi2}"
        elif msg.local_candi1 or msg.local_candi2:
            self.start_location = msg.local_candi1 if msg.local_candi1 else msg.local_candi2
        else:
            self.start_location = ""

        # 4) 목적지 추출
        self.goal_location = msg.goalpoint if msg.goalpoint else ""

        self.update_graphics()

        # 5) LLM 가상 대화창 로그 자동 생성 (목적지가 유효하게 들어왔을 때)
        if msg.goalpoint:
            log_text = f"🎯 목적지 탐색 요청 수신: [{msg.goalpoint}]"
            if msg.waypoints:
                log_text += f" (경유지: {', '.join(msg.waypoints)})"
            
            if not self.llm_messages or self.llm_messages[-1]["text"] != log_text:
                self.llm_messages.append({"text": log_text, "sent": False})
                self.render_chat.refresh()

        # 6) 실시간 데이터 변경 통지에 따른 자성 비동기 UI 컴포넌트 일제 갱신
        self.render_top_buttons.refresh()
        self.render_control_panel.refresh()
        self.render_rviz_boxes.refresh()
        ui.update()

    def motor_current_callback(self, msg):
        current_time = self.get_clock().now().to_msg().sec
        chart.options["series"][0]["data"].append([current_time, msg.data])
        if len(chart.options["series"][0]["data"]) > 40:
            chart.options["series"][0]["data"].pop(0)
        chart.update()

    # --- [5] SVG 동적 그래픽 기하학적 연산 메서드 ---
    def update_graphics(self):
        # 1. 조이스틱 컴포넌트 연산
        js_center_x, js_center_y = 75, 75
        js_max_length = 35
        js_dx = self.ui_joystick_side * js_max_length
        js_dy = -self.ui_joystick_linear * js_max_length
        js_target_x = js_center_x + js_dx
        js_target_y = js_center_y + js_dy

        self.joystick_svg = f"""
        <svg width="150" height="150" class="mx-auto">
            <rect x="10" y="10" width="130" height="130" fill="#1e1e1e" rx="15" />
            <line x1="{js_center_x}" y1="{js_center_y}" x2="{js_target_x}" y2="{js_target_y}" stroke="#a3a3a3" stroke-width="16" stroke-linecap="round" />
            <circle cx="{js_target_x}" cy="{js_target_y}" r="28" fill="#ef4444" stroke="#dc2626" stroke-width="2" />
        </svg>
        """

        # 2. 회전형 기계식 레버 컴포넌트 연산
        pivot_x, pivot_y = 35, 35
        lever_length = 100
        clipped_linear = max(-1.5, min(1.5, self.ui_lever_linear))
        angle_deg = 45.0 - (clipped_linear / 1.5) * 45.0
        angle_rad = math.radians(angle_deg)

        lv_target_x = pivot_x + lever_length * math.cos(angle_rad)
        lv_target_y = pivot_y + lever_length * math.sin(angle_rad)

        self.lever_svg = f"""
        <svg width="150" height="150" class="mx-auto">
            <path d="M 20,20 L 140,20 L 140,35 L 35,35 L 35,140 L 20,140 Z" fill="#1e1e1e" />
            <path d="M 35,35 L 130,35 M 35,35 L 35,130" stroke="#404040" stroke-width="1.5" stroke-dasharray="3" />
            <line x1="{pivot_x}" y1="{pivot_y}" x2="{lv_target_x}" y2="{lv_target_y}" stroke="#a3a3a3" stroke-width="20" stroke-linecap="round" />
            <circle cx="{pivot_x}" cy="{pivot_y}" r="6" fill="#6b7280" />
        </svg>
        """

    def update_robot_vector(self, linear, side):
        """ 250x250 공간의 정중앙(125, 125) 점을 기준으로 출력 벡터 화살표 연산 """
        center_x, center_y = 125, 125
        magnitude = math.sqrt(linear**2 + side**2)
        if magnitude < 0.01:
            self.robot_vector_svg = ""
            return

        scale_length = min(90, magnitude * 50)
        angle = math.atan2(-linear, side)
        
        end_x = center_x + scale_length * math.cos(angle)
        end_y = center_y + scale_length * math.sin(angle)

        self.robot_vector_svg = f"""
        <line x1="{center_x}" y1="{center_y}" x2="{end_x}" y2="{end_y}" stroke="#ef4444" stroke-width="7" marker-end="url(#robot_arrow)" stroke-linecap="round"/>
        """

    # --- [6] 웹 UI 레이아웃 대시보드 엔진 설계 ---
    def build_ui(self):
        ui.query("body").style("background-color: #f1f5f9;")

        with ui.header().classes("bg-[#1e293b] text-white p-3 items-center shadow-md"):
            ui.label("⚙️ CANGO Robot GCS Dashboard").classes("text-lg font-bold tracking-wider")

        with ui.row().classes("w-full p-4 justify-between items-stretch gap-4"):

            # ----------------------------------------------------------------------
            # COLUMN 1: 제어 모드 버튼, 조이스틱/레버 피드백 및 로봇 벡터 레이어
            # ----------------------------------------------------------------------
            with ui.column().classes("w-full md:w-[32%] gap-4"):
                
                # [상단 제어 상태 표시 라벨 블록]
                @ui.refreshable
                def render_top_buttons():
                    # Sit(is_stand=False) 상태일 때 자율 및 수동조종 버튼(왼쪽 버튼) 구역 회색조 잠금 스타일 동적 연산
                    auto_btn_style = "" if self.is_stand else "background-color: #e2e8f0; color: #94a3b8; opacity: 0.6; border: 1px solid #cbd5e1;"
                    
                    with ui.row().classes("w-full gap-2 text-center text-sm font-bold"):
                        # 1) 자율/수동 운전 모드 디스플레이 (Sit 상태일 때 회색 블로킹 처리)
                        if self.is_auto:
                            ui.label("Auto Driving").classes("flex-1 p-3 rounded shadow-sm text-white bg-blue-500 transition-all").style(auto_btn_style)
                        else:
                            ui.label("Operation").classes("flex-1 p-3 rounded shadow-sm text-white bg-green-500 transition-all").style(auto_btn_style)

                        # 2) 기립 제어 모드 디스플레이 (Sit / Stand)
                        if self.is_stand:
                            ui.label("Stand").classes("flex-1 p-3 rounded shadow-sm text-white bg-green-500 transition-all")
                        else:
                            ui.label("Sit").classes("flex-1 p-3 rounded shadow-sm text-white bg-red-500 transition-all")

                render_top_buttons()

                # [조이스틱 및 회전 레버 피드백 패널]
                @ui.refreshable
                def render_control_panel():
                    with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg"):
                        with ui.row().classes("w-full justify-around items-center relative"):
                            
                            # 조이스틱 서브 컴포넌트 구역 (Sit 상태 시 흐리게 락)
                            js_style = "" if self.is_stand else "background-color: #e2e8f0; opacity: 0.6;"
                            with ui.column().classes("items-center p-2 border rounded relative w-[46%]").style(js_style):
                                ui.label("JOYSTICK").classes("text-[10px] font-bold text-slate-400 mb-1")
                                ui.html(self.joystick_svg)
                                with ui.element('div').classes('absolute inset-0 bg-slate-200/60 rounded flex items-center justify-center').bind_visibility_from(self, 'is_auto'):
                                    ui.label('비활성화 (원점)').classes('text-[10px] font-bold text-slate-400 bg-white px-2 py-0.5 rounded shadow-sm')

                            # 회전 레버 서브 컴포넌트 구역
                            with ui.column().classes("items-center p-2 border rounded bg-slate-50 relative w-[46%]"):
                                ui.label("ROTARY LEVER").classes("text-[10px] font-bold text-slate-400 mb-1")
                                ui.html(self.lever_svg)
                                with ui.element('div').classes('absolute inset-0 bg-slate-200/60 rounded flex items-center justify-center').bind_visibility_from(self, 'is_auto', backward=lambda x: not x):
                                    ui.label('비활성화 (원점)').classes('text-[10px] font-bold text-slate-400 bg-white px-2 py-0.5 rounded shadow-sm')

                render_control_panel()

                # [로봇 이미지 및 중앙 정렬 출력 벡터 레이어]
                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg items-center justify-center"):
                    ui.label("🤖 로봇 중심 출력 벡터").classes("text-xs font-bold text-slate-500 self-start mb-2")
                    
                    with ui.element('div').classes('relative w-[250px] h-[250px] bg-slate-100 rounded-full border border-slate-200 shadow-inner flex items-center justify-center'):
                        
                        # 로봇 이미지를 absolute 제어로 프레임 정확한 가운데 배치 (z-10)
                        ui.image('https://images.tuyatech.com/smart/robot/go2.png').classes('w-[130px] opacity-80 absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 z-10 pointer-events-none')
                        
                        # 기준 그리드 데코레이션용 배경 레이어
                        with ui.html().classes('absolute inset-0 z-0 pointer-events-none'):
                            ui.html("""
                                <svg width="250" height="250" class="w-full h-full">
                                    <defs>
                                        <marker id="robot_arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
                                            <path d="M 0 0 L 10 5 L 0 10 z" fill="#ef4444"/>
                                        </marker>
                                    </defs>
                                    <circle cx="125" cy="125" r="40" stroke="#cbd5e1" stroke-width="1" fill="none" stroke-dasharray="3"/>
                                    <circle cx="125" cy="125" r="80" stroke="#cbd5e1" stroke-width="1" fill="none" stroke-dasharray="3"/>
                                </svg>
                            """)
                        
                        # 실시간으로 변경되는 빨간색 출력 벡터 레이어 배치 (z-20)
                        ui.html().bind_content_from(self, "robot_vector_svg").classes('absolute inset-0 z-20 pointer-events-none')

            # ----------------------------------------------------------------------
            # COLUMN 2: RViz 내비게이션 시각화 영역 (Center)
            # ----------------------------------------------------------------------
            with ui.column().classes("w-full md:w-[38%] gap-4"):
                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg flex-grow flex flex-col"):
                    ui.label("🗺️ RViz 시각화").classes("text-base font-bold text-slate-700 mb-2")
                    
                    @ui.refreshable
                    def render_rviz_boxes():
                        with ui.row().classes("w-full gap-2 mb-3"):
                            start_color = "background-color: #ffedd5; border-color: #fed7aa;" if self.start_location else "background-color: #f8fafc; border-color: #e2e8f0;"
                            with ui.row().classes("flex-1 p-3 rounded border text-xs font-medium items-center justify-between").style(start_color):
                                ui.label("출발 위치:")
                                ui.label(self.start_location if self.start_location else "데이터 대기 중...")

                            goal_color = "background-color: #ffedd5; border-color: #fed7aa;" if self.goal_location else "background-color: #f8fafc; border-color: #e2e8f0;"
                            with ui.row().classes("flex-1 p-3 rounded border text-xs font-medium items-center justify-between").style(goal_color):
                                ui.label("목적지:")
                                ui.label(self.goal_location if self.goal_location else "데이터 대기 중...")

                    render_rviz_boxes()

                    with ui.element('div').classes('w-full flex-grow min-h-[380px] bg-slate-900 rounded-lg flex items-center justify-center border border-slate-800 relative overflow-hidden'):
                        ui.label("RViz 3D Map Viewport Link").classes("text-slate-500 font-mono text-sm z-10")
                        ui.icon("map", size="lg").classes("text-slate-700/40 absolute text-[120px] z-0")

            # ----------------------------------------------------------------------
            # COLUMN 3: RMD 모터 실시간 전류 트렌드 그래프 & AI 챗봇 영역 (Right)
            # ----------------------------------------------------------------------
            with ui.column().classes("w-full md:w-[27%] gap-4"):
                
                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg"):
                    ui.label("⚡ rmd 모터 값").classes("text-sm font-bold text-slate-700")
                    
                    global chart
                    chart = ui.echart(
                        {
                            "title": {"text": "실시간 출력 트렌드 (Linear)", "textStyle": {"fontSize": 11, "color": "#64748b"}},
                            "grid": {"top": 35, "bottom": 20, "left": 35, "right": 15},
                            "xAxis": {"type": "value", "show": False},
                            "yAxis": {"type": "value", "min": -5, "max": 5},
                            "series": [{"data": [], "type": "line", "smooth": True, "color": "#f97316", "areaStyle": {"opacity": 0.1}}],
                        }
                    ).classes("w-full h-40 mt-1")

                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg flex-grow flex flex-col"):
                    ui.label("💬 llm 기능").classes("text-sm font-bold text-slate-700 mb-2")
                    
                    with ui.scroll_area().classes("w-full flex-grow h-64 border border-slate-100 p-3 rounded-lg bg-slate-50 shadow-inner"):
                        @ui.refreshable
                        def render_chat():
                            for msg in self.llm_messages:
                                ui.chat_message(
                                    text=msg["text"],
                                    name="LLM Agent" if not msg["sent"] else "Operator",
                                    sent=msg["sent"],
                                    avatar="https://api.dicebear.com/7.x/bottts/svg?seed=cango" if not msg["sent"] else None,
                                )
                        render_chat()
                    
                    self.render_chat = render_chat 

                    with ui.row().classes("w-full mt-2 items-center gap-1"):
                        cmd_input = ui.input(placeholder="로봇 지시어 입력...").classes("flex-grow text-xs").props("outlined dense")
                        
                        def send_message_ui():
                            if cmd_input.value:
                                self.llm_messages.append({"text": cmd_input.value, "sent": True})
                                self.llm_messages.append({"text": f"인식 완료: '{cmd_input.value}' 작업을 분석합니다.", "sent": False})
                                cmd_input.value = ""
                                render_chat.refresh()
                        
                        ui.button(icon="send", on_click=send_message_ui).props("flat color=primary")


# --- [7] ROS 2 비동기 구동 프로세서 루프 루틴 ---
if not rclpy.utilities.ok():
    rclpy.init()

node = RobotWebUI()

async def ros_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)

app.on_startup(ros_loop)

if __name__ in {"__main__", "__mp_main__"}:
    ui.run(title="CANGO GCS Dashboard", port=8080, reload=False, show=True)