import asyncio
import math
from nicegui import app, ui
import rclpy
from rclpy.node import Node

# 커스텀 메시지 포맷 임포트
from cango_msgs.msg import RobotControl, LlmRequest, RobotStatus, SoundRequest

# 글로벌 접근을 위한 노드 및 차트 전역 플레이스홀더
node = None
chart = None

# ----------------------------------------------------------------------
# 글로벌 리프레시 컴포넌트 데이터 바인딩 구조
# ----------------------------------------------------------------------
@ui.refreshable
def render_top_buttons():
    if node is None:
        return
    with ui.row().classes("w-full gap-2 text-center text-sm font-bold"):
        ui.html().bind_content_from(node.state, "mode_html").classes("flex-1")
        ui.html().bind_content_from(node.state, "stand_html").classes("flex-1")

@ui.refreshable
def render_control_panel():
    if node is None:
        return
    with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg"):
        with ui.row().classes("w-full justify-around items-center relative"):
            
            # JOYSTICK UI BOX
            with ui.column().classes("items-center p-2 border rounded relative w-[46%]"):
                ui.label("JOYSTICK").classes("text-[10px] font-bold text-slate-400 mb-1")
                ui.html().bind_content_from(node.state, "joystick_svg")
                
                overlay_auto = ui.element('div').classes(
                    'absolute inset-0 bg-slate-200/70 flex items-center justify-center rounded z-30'
                )
                with overlay_auto:
                    ui.label('비활성화').classes('text-[10px] font-bold text-slate-500 bg-white px-2 py-0.5 rounded shadow-sm')
                overlay_auto.bind_visibility_from(node, 'is_auto')

            # ROTARY LEVER UI BOX
            with ui.column().classes("items-center p-2 border rounded bg-slate-50 relative w-[46%]"):
                ui.label("ROTARY LEVER").classes("text-[10px] font-bold text-slate-400 mb-1")
                ui.html().bind_content_from(node.state, "lever_svg")
                
                overlay_manual = ui.element('div').classes(
                    'absolute inset-0 bg-slate-200/70 flex items-center justify-center rounded z-30'
                )
                with overlay_manual:
                    ui.label('비활성화').classes('text-[10px] font-bold text-slate-500 bg-white px-2 py-0.5 rounded shadow-sm')
                overlay_manual.bind_visibility_from(node, 'is_auto', backward=lambda x: not x)

@ui.refreshable
def render_rviz_boxes():
    if node is None:
        return
    with ui.row().classes("w-full gap-2 mb-3"):
        with ui.row().classes("flex-1 p-3 rounded border text-xs font-medium items-center justify-between").style("background-color: #f8fafc; border-color: #e2e8f0;"):
            ui.label("출발 위치:")
            ui.label().bind_text_from(node, "start_location", backward=lambda x: x if x else "데이터 대기 중...")

        with ui.row().classes("flex-1 p-3 rounded border text-xs font-medium items-center justify-between").style("background-color: #f8fafc; border-color: #e2e8f0;"):
            ui.label("목적지:")
            ui.label().bind_text_from(node, "goal_location", backward=lambda x: x if x else "데이터 대기 중...")

@ui.refreshable
def render_chat():
    if node is None:
        return
    for msg in node.llm_messages:
        ui.chat_message(
            text=msg["text"],
            name="LLM Agent" if not msg["sent"] else "Operator",
            sent=msg["sent"],
            avatar="https://api.dicebear.com/7.x/bottts/svg?seed=cango" if not msg["sent"] else None,
        )


class RobotWebUI(Node):

    def __init__(self):
        super().__init__("robot_web_ui")
        self.get_logger().info("=== CANGO Robot GCS Node Initializing ===")

        # --- 상태 관리 내부 변수 ---
        self.is_auto = False   
        self.is_stand = False  

        self.start_location = ""  
        self.goal_location = ""   

        self.ui_joystick_linear = 0.0
        self.ui_joystick_side = 0.0
        self.ui_lever_linear = -1.5 

        self.state = {
            "joystick_svg": "",
            "lever_svg": "",
            "robot_vector_svg": "", 
            "mode_html": "",
            "stand_html": ""
        }

        self.llm_messages = [{"text": "로봇 명령 대기중입니다.", "sent": False}]

        self.chart_angle_buffer = []
        self.chart_torque_buffer = []
        self.chart_x_buffer = []

        self.update_graphics()
        self.update_status_html()
        self.update_robot_vector(0.0, 0.0, 0.0)

        # --- ROS 2 토픽 퍼블리셔/구독자 설정 ---
        self.ui_text_pub = self.create_publisher(SoundRequest, "/llm_ui_text", 10)

        self.llm_request_sub = self.create_subscription(LlmRequest, "/cango/master2llm", self.llm_request_callback, 10)
        self.llm2master_sub = self.create_subscription(LlmRequest, "/cango/llm2master", self.llm2master_callback, 10)
        self.control_sub = self.create_subscription(RobotControl, "/cango/master2control", self.control_callback, 10)
        self.status_sub = self.create_subscription(RobotStatus, "/cango/robot_status", self.robot_status_callback, 10)
        self.tts_sub = self.create_subscription(SoundRequest, "/cango/sound2ui", self.tts_input_callback, 10)

        @ui.page('/')
        def index():
            global chart
            chart = None  # 새로고침 시 구형 차트 세션 해제
            self.build_ui()

        self.get_logger().info("=== CANGO Robot GCS Node Initialization Complete ===")

    def trigger_ui_refresh(self):
        try:
            render_top_buttons.refresh()
            render_control_panel.refresh()
            render_rviz_boxes.refresh()
            render_chat.refresh()
        except:
            pass

    # --- ROS 2 콜백 함수부 ---
    
    def llm2master_callback(self, msg):
        try:
            if hasattr(msg, 'goalpoint') and msg.goalpoint:
                self.goal_location = msg.goalpoint
            elif hasattr(msg, 'goal_point') and msg.goal_point:  
                self.goal_location = msg.goal_point
                
            self.get_logger().info(f"📬 [llm2master] 목적지 수신 완료 -> {self.goal_location}")
            render_rviz_boxes.refresh()
        except Exception as e:
            self.get_logger().error(f"❌ llm2master_callback 처리 중 에러: {e}")

    def robot_status_callback(self, msg):
        global chart
        try:
            raw_x = getattr(msg, 'joystick_x', 512.0)
            raw_y = getattr(msg, 'joystick_y', 500.0)
            dyn_angle = getattr(msg, 'dynamixel_angle_deg', 134.0)

            if chart is not None:
                try:
                    r_angle = getattr(msg, 'robstride_angle_deg', 0.0)
                    r_torque = getattr(msg, 'robstride_torque_nm', 0.0)
                    
                    self.chart_angle_buffer.append(float(r_angle))
                    self.chart_torque_buffer.append(float(r_torque))
                    self.chart_x_buffer.append("")
                    
                    if len(self.chart_angle_buffer) > 30:
                        self.chart_angle_buffer.pop(0)
                        self.chart_torque_buffer.pop(0)
                        self.chart_x_buffer.pop(0)
                    
                    chart.run_chart_method('setOption', {
                        "xAxis": {"data": self.chart_x_buffer},
                        "series": [
                            {"data": self.chart_angle_buffer},
                            {"data": self.chart_torque_buffer}
                        ]
                    })
                except:
                    chart = None  # 세션 끊김 시 에러 무시하고 비우기

            if not self.is_auto:
                self.ui_joystick_side = -((raw_x - 512.0) / 512.0)
                self.ui_joystick_linear = -((raw_y - 500.0) / 500.0)
            else:
                self.ui_joystick_side = 0.0
                self.ui_joystick_linear = 0.0

                target_max_scale = 0.83  

                if dyn_angle <= 134.0:
                    self.ui_lever_linear = -1.5
                elif 134.0 < dyn_angle <= 160.0:
                    ratio = (dyn_angle - 134.0) / (160.0 - 134.0)
                    self.ui_lever_linear = -1.5 + ratio * (target_max_scale - (-1.5))
                elif 160.0 < dyn_angle < 190.0:
                    self.ui_lever_linear = target_max_scale
                else:  
                    self.ui_lever_linear = 1.5

            self.update_graphics()

        except Exception as e:
            if "deleted" not in str(e):
                self.get_logger().error(f"❌ robot_status_callback 연산 중 에러: {e}")

    def control_callback(self, msg):
        try:
            raw_mode = getattr(msg, 'mode', None)
            
            if raw_mode is True or raw_mode == 1 or str(raw_mode).strip().lower() in ['true', '1', '1.0']:
                current_mode = 0  
            elif raw_mode is False or raw_mode == 0 or str(raw_mode).strip().lower() in ['false', '0', '0.0']:
                current_mode = 1  
            else:
                current_mode = 0  
                
            self.is_auto = (current_mode == 0)
            
            if hasattr(msg, 'robot_up') and msg.robot_up is not None:
                self.is_stand = bool(msg.robot_up)

            # 토픽 속성 안전 파싱
            ctrl_linear = 0.0
            if hasattr(msg, 'linear_speed'): ctrl_linear = float(msg.linear_speed)
            elif hasattr(msg, 'linear'): ctrl_linear = float(msg.linear)

            ctrl_side = 0.0
            if hasattr(msg, 'side_speed'): ctrl_side = float(msg.side_speed)
            elif hasattr(msg, 'side'): ctrl_side = float(msg.side)

            ctrl_angular = 0.0
            if hasattr(msg, 'ang'): ctrl_angular = float(msg.ang)
            elif hasattr(msg, 'angular'): ctrl_angular = float(msg.angular)
            elif hasattr(msg, 'angular_speed'): ctrl_angular = float(msg.angular_speed)
            elif hasattr(msg, 'ang_speed'): ctrl_angular = float(msg.ang_speed)

            # 데이터 가공 후 로컬 바인딩 딕셔너리만 업데이트
            self.update_robot_vector(ctrl_linear, ctrl_side, ctrl_angular)
            self.update_graphics()
            self.update_status_html()

        except Exception as e:
            if "deleted" not in str(e):
                self.get_logger().error(f"❌ control_callback 파싱 중 에러: {e}")

    def tts_input_callback(self, msg):
        try:
            raw_user = getattr(msg, 'user', "").strip()
            raw_llm = getattr(msg, 'llm_text', "").strip()
            updated = False

            if raw_user:
                self.llm_messages.append({"text": raw_user, "sent": True})
                updated = True
            if raw_llm:
                self.llm_messages.append({"text": raw_llm, "sent": False})
                updated = True

            if updated:
                render_chat.refresh()
        except Exception as e:
            self.get_logger().error(f"❌ tts_input_callback 처리 오류: {e}")

    def send_ui_text_message(self, text_value):
        if not text_value.strip():
            return
        try:
            self.llm_messages.append({"text": text_value, "sent": True})
            render_chat.refresh()

            pub_msg = SoundRequest()
            pub_msg.request = True
            pub_msg.ordered_num = 4  
            pub_msg.text = str(text_value)
            pub_msg.user = str(text_value)
            pub_msg.llm = ""

            self.ui_text_pub.publish(pub_msg)
        except Exception as e:
            self.get_logger().error(f"❌ UI 텍스트 퍼블리시 실패: {e}")

    def llm_request_callback(self, msg):
        if hasattr(msg, 'stand'):
            self.is_stand = (msg.stand == 1)
        if msg.local_candi1 and msg.local_candi2:
            self.start_location = f"{msg.local_candi1} ~ {msg.local_candi2}"
        elif msg.local_candi1 or msg.local_candi2:
            self.start_location = msg.local_candi1 if msg.local_candi1 else msg.local_candi2
        else:
            self.start_location = ""
        
        if msg.goalpoint:
            self.goal_location = msg.goalpoint
            
        try:
            self.update_graphics()
            self.update_status_html()
        except:
            pass

    # --- SVG 렌더링 엔진 부 ---
    
    def update_graphics(self):
        js_center_x, js_center_y = 75, 75
        js_max_length = 35
        js_dx = self.ui_joystick_side * js_max_length
        js_dy = -self.ui_joystick_linear * js_max_length  
        js_target_x = js_center_x + js_dx
        js_target_y = js_center_y + js_dy

        self.state["joystick_svg"] = f"""
        <svg width="150" height="150" class="mx-auto">
            <rect x="10" y="10" width="130" height="130" fill="#1e1e1e" rx="15" />
            <line x1="{js_center_x}" y1="{js_center_y}" x2="{js_target_x}" y2="{js_target_y}" stroke="#a3a3a3" stroke-width="16" stroke-linecap="round" />
            <circle cx="{js_target_x}" cy="{js_target_y}" r="28" fill="#ef4444" stroke="#dc2626" stroke-width="2" />
        </svg>
        """

        pivot_x, pivot_y = 35, 35
        lever_length = 100
        clipped_linear = max(-1.5, min(1.5, self.ui_lever_linear))
        angle_deg = 45.0 - (clipped_linear / 1.5) * 45.0
        angle_rad = math.radians(angle_deg)

        lv_target_x = pivot_x + lever_length * math.cos(angle_rad)
        lv_target_y = pivot_y + lever_length * math.sin(angle_rad)

        self.state["lever_svg"] = f"""
        <svg width="150" height="150" class="mx-auto">
            <path d="M 20,20 L 140,20 L 140,35 L 35,35 L 35,140 L 20,140 Z" fill="#1e1e1e" />
            <path d="M 35,35 L 130,35 M 35,35 L 35,130" stroke="#404040" stroke-width="1.5" stroke-dasharray="3" />
            <line x1="{pivot_x}" y1="{pivot_y}" x2="{lv_target_x}" y2="{lv_target_y}" stroke="#a3a3a3" stroke-width="20" stroke-linecap="round" />
            <circle cx="{pivot_x}" cy="{pivot_y}" r="6" fill="#6b7280" />
        </svg>
        """

    def update_robot_vector(self, linear, side, angular):
        center_x, center_y = 125, 125
        try:
            val_linear = float(linear)
            val_side = float(side)
            val_ang = float(angular)
        except:
            val_linear, val_side, val_ang = 0.0, 0.0, 0.0

        # 1️⃣ [side 주행]
        if abs(val_side) >= 0.0001:
            rotate_deg = 90.0 if val_side >= 0 else -90.0
            clamped_side = max(0.0, min(1.0, abs(val_side)))
            arrow_length = 40.0 + (clamped_side * 75.0)  
            
            vector_line_html = f"""
            <g transform="translate({center_x}, {center_y}) rotate({rotate_deg})">
                <line x1="0" y1="0" x2="0" y2="-{arrow_length}" stroke="#ef4444" stroke-width="6" stroke-linecap="round"/>
                <polygon points="0,-{arrow_length+14} 5,-{arrow_length} -5,-{arrow_length}" fill="#ef4444" />
            </g>
            """

        # 2️⃣ [ang 주행 지시 벡터 계산]
        else:
            # -1.0 ~ 1.0 범위 기준 시원하게 가시성을 주기 위해 최대 좌우 90도 범위로 고정 확장
            clamped_ang = max(-1.0, min(1.0, val_ang))
            rotate_deg = clamped_ang * 90.0
            
            if val_linear < 0:
                rotate_deg += 180.0

            clamped_linear = max(0.0, min(1.0, abs(val_linear)))
            arrow_length = 40.0 + (clamped_linear * 75.0)

            vector_line_html = f"""
            <g transform="translate({center_x}, {center_y}) rotate({rotate_deg})">
                <line x1="0" y1="0" x2="0" y2="-{arrow_length}" stroke="#ef4444" stroke-width="6" stroke-linecap="round"/>
                <polygon points="0,-{arrow_length+14} 5,-{arrow_length} -5,-{arrow_length}" fill="#ef4444" />
            </g>
            """

        # 🚨 [완벽 해결 핵심] 직접 돔을 건드리지 않고, 동기화된 데이터 컨테이너 문자열만 교체합니다.
        self.state["robot_vector_svg"] = self.get_base_svg_template(vector_line_html)

    def get_base_svg_template(self, vector_line_html):
        return f"""
        <svg width="250" height="250" class="w-full h-full">
            <circle cx="125" cy="125" r="40" stroke="#cbd5e1" stroke-width="1.5" fill="none" stroke-dasharray="4"/>
            <circle cx="125" cy="125" r="80" stroke="#cbd5e1" stroke-width="1.5" fill="none" stroke-dasharray="4"/>
            <circle cx="125" cy="125" r="115" stroke="#e2e8f0" stroke-width="1" fill="none"/>
            <line x1="10" y1="125" x2="240" y2="125" stroke="#e2e8f0" stroke-width="1" stroke-dasharray="2"/>
            <line x1="125" y1="10" x2="125" y2="240" stroke="#e2e8f0" stroke-width="1" stroke-dasharray="2"/>
            
            <g transform="translate(95, 100)" opacity="0.35">
                <rect x="5" y="15" width="50" height="35" rx="8" fill="#475569" />
                <rect x="15" y="2" width="30" height="12" rx="4" fill="#334155" />
                <circle cx="23" cy="8" r="3" fill="#cbd5e1" />
                <circle cx="37" cy="8" r="3" fill="#cbd5e1" />
                <rect x="0" y="10" width="8" height="45" rx="3" fill="#1e293b" />
                <rect x="52" y="10" width="8" height="45" rx="3" fill="#1e293b" />
            </g>
            {vector_line_html}
        </svg>
        """

    def update_status_html(self):
        mode_text = "Auto Driving" if self.is_auto else "Operation"
        mode_color = "bg-blue-500" if self.is_auto else "bg-green-500"
        self.state["mode_html"] = f'<div class="p-3 rounded shadow-sm text-white {mode_color} text-center font-bold">{mode_text}</div>'

        stand_text = "Stand" if self.is_stand else "Sit"
        stand_color = "bg-green-500" if self.is_stand else "bg-red-500"
        self.state["stand_html"] = f'<div class="p-3 rounded shadow-sm text-white {stand_color} text-center font-bold">{stand_text}</div>'

    # --- UI 레이아웃 설계 빌더 ---
    
    def build_ui(self):
        global chart
        ui.query("body").style("background-color: #f1f5f9;")

        with ui.header().classes("bg-[#1e293b] text-white p-3 items-center shadow-md"):
            ui.label("⚙️ CANGO Robot GCS Dashboard").classes("text-lg font-bold tracking-wider")

        ui.timer(0.1, self.trigger_ui_refresh)

        with ui.row().classes("w-full p-4 justify-between items-stretch gap-4"):

            # COLUMN 1: 조작 판넬 및 화살표 출력 벡터
            with ui.column().classes("w-full md:w-[32%] gap-4"):
                render_top_buttons()      
                render_control_panel()    

                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg items-center justify-center"):
                    ui.label("🤖 로봇 중심 출력 벡터").classes("text-xs font-bold text-slate-500 self-start mb-2")
                    
                    with ui.element('div').classes('w-[250px] h-[250px] bg-slate-100 rounded-full border border-slate-200 shadow-inner flex items-center justify-center relative overflow-hidden'):
                        # 🚨 [구조 대개조] 직접 할당 방식 철폐하고 전역 바인딩 구조로 완전 결속!
                        ui.html().bind_content_from(self.state, "robot_vector_svg").classes('w-full h-full absolute inset-0')

            # COLUMN 2: 디지털 지도 영역
            with ui.column().classes("w-full md:w-[38%] gap-4"):
                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg flex-grow flex flex-col"):
                    ui.label("🗺️ RViz 시각화").classes("text-base font-bold text-slate-700 mb-2")
                    render_rviz_boxes()  

                    with ui.element('div').classes('w-full flex-grow min-h-[380px] bg-slate-900 rounded-lg flex items-center justify-center border border-slate-800 relative overflow-hidden'):
                        ui.label("RViz 3D Map Viewport Link").classes("text-slate-500 font-mono text-sm z-10")
                        ui.icon("map", size="lg").classes("text-slate-700/40 absolute text-[120px] z-0")

            # COLUMN 3: Robstride 2중 차트 및 대화 창
            with ui.column().classes("w-full md:w-[27%] gap-4"):
                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg"):
                    ui.label("⚡ Robstride 모터 데이터 추이").classes("text-sm font-bold text-slate-700")
                    
                    chart = ui.echart(
                        {
                            "legend": {"data": ["각도 (deg)", "토크 (Nm)"], "top": 0, "textStyle": {"fontSize": 10}},
                            "grid": {"top": 35, "bottom": 20, "left": 40, "right": 40},
                            "xAxis": {"type": "category", "data": [], "show": False}, 
                            "yAxis": [
                                {"type": "value", "name": "deg", "min": -50, "max": 50, "position": "left"},  
                                {"type": "value", "name": "Nm", "min": -2.5, "max": 2.5, "position": "right"}  
                            ],
                            "series": [
                                {
                                    "name": "각도 (deg)", 
                                    "data": [], 
                                    "type": "line", 
                                    "smooth": True, 
                                    "color": "#f97316",
                                    "yAxisIndex": 0,
                                    "showSymbol": True,
                                    "symbolSize": 4
                                },
                                {
                                    "name": "토크 (Nm)", 
                                    "data": [], 
                                    "type": "line", 
                                    "smooth": True, 
                                    "color": "#a855f7",
                                    "yAxisIndex": 1,
                                    "showSymbol": True,
                                    "symbolSize": 4
                                }
                            ],
                        }
                    ).classes("w-full h-44 mt-1")

                with ui.card().classes("w-full p-4 bg-white shadow-sm rounded-lg flex-grow flex flex-col"):
                    ui.label("💬 llm 기능").classes("text-sm font-bold text-slate-700 mb-2")
                    with ui.scroll_area().classes("w-full flex-grow h-64 border border-slate-100 p-3 rounded-lg bg-slate-50 shadow-inner"):
                        render_chat()  

                    with ui.row().classes("w-full mt-2 items-center gap-1"):
                        cmd_input = ui.input(placeholder="로봇 지시어 입력...").classes("flex-grow text-xs").props("outlined dense")
                        
                        def handle_send_event():
                            val = cmd_input.value
                            if val and val.strip():
                                self.send_ui_text_message(val)
                                cmd_input.value = "" 
                                
                        ui.button(icon="send", on_click=handle_send_event).props("flat color=primary")


# --- ROS 2 비동기 구동 진입 루프 ---
if not rclpy.utilities.ok():
    rclpy.init()

node = RobotWebUI()

async def ros_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)

app.on_startup(ros_loop)

if __name__ in {"__main__", "__mp_main__"}:
    ui.run(host="0.0.0.0", port=8080, reload=False, show=True)
