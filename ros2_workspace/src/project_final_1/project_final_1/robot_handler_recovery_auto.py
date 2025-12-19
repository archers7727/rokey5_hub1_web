import sys
import time
import threading

import rclpy
import DR_init
from rclpy.executors import MultiThreadedExecutor
from supabase import create_client, Client
from rclpy.callback_groups import ReentrantCallbackGroup # ✅ 필수 추가

SUPABASE_URL = "https://lxllllhkovbegtbcfnaw.supabase.co"
SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx4bGxsbGhrb3ZiZWd0YmNmbmF3Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjU4MDQ4MzksImV4cCI6MjA4MTM4MDgzOX0.MGL576AumztyxI7SEYUCxnPuo0euoWWFoEmTmJmfEEQ"

ROBOT_STATE_TABLE = "robot_state"
ROBOT_STATE_ROW_ID = "current"


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v3"

VELOCITY = 150
ACC = 150
l_VELOCITY = 150
l_ACC = 150

HOME_JOINT = [0, 0, 90, 0, 90, 0]
HOME_VEL = 60
HOME_ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =========================
# Doosan robot_state 숫자 매핑
# =========================
ROBOT_STATE_MAP = {
    0: "STATE_INITIALIZING",
    1: "STATE_STANDBY",
    2: "STATE_MOVING",
    3: "STATE_SAFE_OFF",
    4: "STATE_TEACHING",
    5: "STATE_SAFE_STOP",
    6: "STATE_EMERGENCY_STOP",
    7: "STATE_HOMMING",
    8: "STATE_RECOVERY",
    9: "STATE_SAFE_STOP2",
    10: "STATE_SAFE_OFF2",
    11: "STATE_RESERVED1",
    12: "STATE_RESERVED2",
    13: "STATE_RESERVED3",
    14: "STATE_RESERVED4",
    15: "STATE_NOT_READY",
}

ERROR_STATES = {3, 5, 6, 9, 10, 15}

RECOVERY_CONTROL_BY_STATE = {
    3: 3,
    10: 3,
    5: 2,
    9: 2,
    6: 1,
}


def normalize_cmd(cmd: str) -> str:
    return (cmd or "").strip().lower().replace("-", "_")


def check_and_get_pending_task(supabase: Client):
    try:
        response = (
            supabase.table("tasks")
            .select("*")
            .eq("status", "pending")
            .order("created_at", desc=False)
            .limit(1)
            .execute()
        )

        if response.data:
            task = response.data[0]
            task_id = task["id"]
            parameters = task.get("parameters", {}) or {}
            interval = parameters.get("thickness", 50)
            print(f"✅ Found pending task: {task_id} / interval={interval}")
            return task_id, interval

        return None, None

    except Exception as e:
        print(f"❌ Error checking pending tasks: {e}")
        return None, None


def update_task_status(supabase: Client, task_id: str, status: str):
    try:
        (
            supabase.table("tasks")
            .update({"status": status})
            .eq("id", task_id)
            .execute()
        )
        print(f"✅ Task {task_id} status -> {status}")
    except Exception as e:
        print(f"❌ Error updating task status: {e}")


def get_robot_desired_state(supabase: Client):
    try:
        res = (
            supabase.table(ROBOT_STATE_TABLE)
            .select("desired_state,command_timestamp")
            .eq("id", ROBOT_STATE_ROW_ID)
            .limit(1)
            .execute()
        )
        if not res.data:
            return "", None
        row = res.data[0]
        desired = normalize_cmd(row.get("desired_state"))
        ts = row.get("command_timestamp", None)
        return desired, ts
    except Exception as e:
        print(f"❌ get_robot_desired_state error: {e}")
        return "", None


def update_robot_state_row(supabase: Client, **fields):
    try:
        (
            supabase.table(ROBOT_STATE_TABLE)
            .update(fields)
            .eq("id", ROBOT_STATE_ROW_ID)
            .execute()
        )
    except Exception as e:
        print(f"❌ update_robot_state_row error: {e}")


class MotionController:
    """
    ✅ 수정됨: ReentrantCallbackGroup 적용하여 데드락 방지
    """
    def __init__(self, node, robot_id: str):
        from dsr_msgs2.srv import MovePause, MoveResume, MoveStop

        self.node = node
        self.robot_id = robot_id
        
        # [수정] 콜백 그룹 적용
        self.cb_group = ReentrantCallbackGroup()

        self.pause_cli = node.create_client(MovePause, f"/{robot_id}/motion/move_pause", callback_group=self.cb_group)
        self.resume_cli = node.create_client(MoveResume, f"/{robot_id}/motion/move_resume", callback_group=self.cb_group)
        self.stop_cli = node.create_client(MoveStop, f"/{robot_id}/motion/move_stop", callback_group=self.cb_group)

        if not self.pause_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MovePause service not available")
        if not self.resume_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MoveResume service not available")
        if not self.stop_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MoveStop service not available")

        self._paused = False
        self._lock = threading.Lock()

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused

    def _set_paused(self, v: bool):
        with self._lock:
            self._paused = v

    def _call_wait(self, cli, req, timeout=2.0):
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and (time.time() - t0) < timeout:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def move_pause(self) -> bool:
        from dsr_msgs2.srv import MovePause
        res = self._call_wait(self.pause_cli, MovePause.Request())
        ok = bool(res is not None and getattr(res, "success", False))
        if ok:
            self._set_paused(True)
            self.node.get_logger().info("✅ MovePause 성공")
        else:
            self.node.get_logger().error("❌ MovePause 실패/타임아웃")
        return ok

    def move_resume(self) -> bool:
        from dsr_msgs2.srv import MoveResume
        res = self._call_wait(self.resume_cli, MoveResume.Request())
        ok = bool(res is not None and getattr(res, "success", False))
        if ok:
            self._set_paused(False)
            self.node.get_logger().info("✅ MoveResume 성공")
        else:
            self.node.get_logger().error("❌ MoveResume 실패/타임아웃")
        return ok

    def move_stop(self, stop_mode: int = 1) -> bool:
        from dsr_msgs2.srv import MoveStop
        req = MoveStop.Request()
        if hasattr(req, "stop_mode"):
            req.stop_mode = int(stop_mode)
        elif hasattr(req, "stop_type"):
            req.stop_type = int(stop_mode)

        res = self._call_wait(self.stop_cli, req)
        ok = bool(res is not None and getattr(res, "success", False))
        if ok:
            self.node.get_logger().warn(f"🚨 MoveStop 성공 (mode={stop_mode})")
        else:
            # MoveStop은 실패해도 치명적이지 않으므로(이미 멈춰있을 수 있음) warn 처리
            self.node.get_logger().warn("⚠️ MoveStop 실패/타임아웃 (이미 멈춰있거나 에러 상태일 수 있음)")
        return ok


class RobotSystemController:
    """
    [Final Ver] 독립 노드를 사용하여 Deadlock을 회피하고,
    Safe Stop(5) -> Safe Off(3) -> Standby(1)로 이어지는 
    '2단계 연쇄 수복'을 자동으로 처리하는 컨트롤러
    """
    def __init__(self, node, robot_id: str):
        from DSR_ROBOT2 import GetRobotState, SetRobotControl
        
        self.node = node
        self.robot_id = robot_id
        self._GetRobotState = GetRobotState
        self._SetRobotControl = SetRobotControl

        # 평상시 모니터링용 (메인 Executor 사용)
        self.cb_group = ReentrantCallbackGroup()
        self.state_cli = node.create_client(
            GetRobotState, 
            f"/{robot_id}/system/get_robot_state", 
            callback_group=self.cb_group
        )
        self.ctrl_cli = node.create_client(
            SetRobotControl, 
            f"/{robot_id}/system/set_robot_control", 
            callback_group=self.cb_group
        )

    def _call_wait(self, cli, req, timeout=2.0):
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and (time.time() - t0) < timeout:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def get_robot_state(self, timeout=1.0):
        # 평상시 메인 루프에서 사용하는 조회 함수
        req = self._GetRobotState.Request()
        res = self._call_wait(self.state_cli, req, timeout=timeout)
        if res is None:
            return None
        return int(getattr(res, "robot_state", -1))

    def set_robot_control(self, robot_control: int, timeout=3.0) -> bool:
        req = self._SetRobotControl.Request()
        req.robot_control = robot_control
        res = self._call_wait(self.ctrl_cli, req, timeout=timeout)
        return bool(res is not None and getattr(res, "success", False))

    def _standalone_recovery_step(self, target_control_mode: int) -> bool:
        """독립 노드를 생성하여 단발성 복구 명령을 전송"""
        import rclpy
        
        temp_node = rclpy.create_node(f"recovery_worker_{int(time.time()*1000)}")
        try:
            cli = temp_node.create_client(self._SetRobotControl, f"/{self.robot_id}/system/set_robot_control")
            if not cli.wait_for_service(timeout_sec=2.0):
                return False
            
            req = self._SetRobotControl.Request()
            req.robot_control = target_control_mode
            
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=3.0)
            
            if future.result() is not None and future.result().success:
                return True
            return False
        except Exception:
            return False
        finally:
            temp_node.destroy_node()

    def _standalone_get_state(self) -> int:
        """독립 노드를 생성하여 가장 확실한 현재 상태를 조회 (캐싱/데드락 방지)"""
        import rclpy
        temp_node = rclpy.create_node(f"state_worker_{int(time.time()*1000)}")
        try:
            cli = temp_node.create_client(self._GetRobotState, f"/{self.robot_id}/system/get_robot_state")
            if not cli.wait_for_service(timeout_sec=2.0):
                return -1
            
            req = self._GetRobotState.Request()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=2.0)
            
            if future.result() is not None:
                return int(future.result().robot_state)
            return -1
        except Exception:
            return -1
        finally:
            temp_node.destroy_node()

    def recover_if_possible(self, initial_state: int) -> bool:
        """
        [핵심] 스마트 연쇄 수복 로직
        1. Safe Stop(5)이면 Reset(2) 전송
        2. 상태 확인 -> Safe Off(3)으로 변했으면 -> Servo On(3) 전송
        3. Standby(1) 되면 성공
        """
        
        # 1. 초기 진단 및 1차 명령 결정
        current_state = self._standalone_get_state()
        if current_state == -1: current_state = initial_state # 조회 실패시 인자값 신뢰

        target_ctrl = RECOVERY_CONTROL_BY_STATE.get(current_state)
        if target_ctrl is None:
            self.node.get_logger().error(f"⛔ 수복 불가 상태: {current_state}")
            return False

        name = ROBOT_STATE_MAP.get(current_state, str(current_state))
        self.node.get_logger().warn(f"⚡ [1차 수복] {name} -> Control({target_ctrl}) 시도 (독립 노드)")
        
        # 2. 1차 명령 전송
        if self._standalone_recovery_step(target_ctrl):
            self.node.get_logger().info("✅ 1차 명령 전송 성공. 상태 변화 관찰 중...")
        else:
            self.node.get_logger().error("❌ 1차 명령 거부됨.")
            return False

        # 3. 상태 변화 관찰 및 2차 수복 (최대 10초)
        t_start = time.time()
        while time.time() - t_start < 10.0:
            time.sleep(1.0) # 로봇 반응 대기
            
            s = self._standalone_get_state()
            s_name = ROBOT_STATE_MAP.get(s, str(s))
            self.node.get_logger().info(f"🧐 현재 상태: {s}({s_name})")

            # [성공 케이스] 에러 상태가 아니면 완료 (Standby=1, Moving=2 등)
            if s not in ERROR_STATES and s > 0:
                self.node.get_logger().info(f"🎉 수복 완료! 최종 상태: {s_name}")
                return True

            # [2차 수복 케이스] Safe Stop(5) -> Safe Off(3)으로 변한 경우
            # 에러는 풀렸으나 모터가 꺼진 상태이므로 켜줘야 함
            if s == 3: 
                self.node.get_logger().warn("⚡ [2차 수복] Safe Off 감지 -> Servo On(3) 명령 전송")
                if self._standalone_recovery_step(3): # Servo On 명령
                    self.node.get_logger().info("✅ Servo On 명령 전송됨")
                    time.sleep(1.5) # 서보 켜지는 시간 대기
                    continue # 루프 다시 돌면서 상태 확인
            
            # [재시도 케이스] 여전히 5번(Safe Stop)이면 1차 명령 재전송
            if s == 5 and (time.time() - t_start > 3.0):
                 self.node.get_logger().warn("⚠️ 상태 불변. Reset(2) 재전송...")
                 self._standalone_recovery_step(2)
                 time.sleep(1.0)

        self.node.get_logger().error("⛔ 시간 초과: 수복 실패")
        return False

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def grip_close():
    from DSR_ROBOT2 import set_digital_output, ON, OFF, wait
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(0.5)


def grip_open():
    from DSR_ROBOT2 import set_digital_output, ON, OFF, wait
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(0.5)


def go_home():
    from DSR_ROBOT2 import movej
    from DR_common2 import posj
    print(f"🏠 Going HOME (joint={HOME_JOINT})")
    home = posj(HOME_JOINT)
    movej(home, vel=HOME_VEL, acc=HOME_ACC)


class StopRequested(Exception):
    pass


class HomeRequested(Exception):
    pass


def command_watcher_thread(
    run_gate: threading.Event,
    abort_event: threading.Event,
    home_event: threading.Event,
    stop_event: threading.Event,
    motion: MotionController,
    fault_event: threading.Event,
):
    supabase = create_client(SUPABASE_URL, SUPABASE_ANON_KEY)
    last_ts = None
    last_cmd = None

    while not stop_event.is_set():
        cmd, ts = get_robot_desired_state(supabase)
        is_new = (ts is not None and ts != last_ts) or (ts is None and cmd != last_cmd)

        if is_new:
            print(f"[Watcher] cmd={cmd!r}, ts={ts!r}")
            last_ts = ts
            last_cmd = cmd

            if cmd in ("emergency_stop", "stop", "abort"):
                # [수정] 이미 에러 상태면 MoveStop 스킵 (타임아웃 방지)
                if not fault_event.is_set():
                    motion.move_stop(stop_mode=1)
                abort_event.set()
                run_gate.set()

            elif cmd == "home":
                # [수정] 이미 에러 상태면 MoveStop 스킵
                if not fault_event.is_set():
                    motion.move_stop(stop_mode=1)
                
                abort_event.set() 
                home_event.set()
                run_gate.set()

            elif cmd == "pause":
                if not (abort_event.is_set() or fault_event.is_set() or home_event.is_set()):
                    if not motion.paused:
                        motion.move_pause()
                    run_gate.clear()

            else:  # resume/default
                if not (abort_event.is_set() or fault_event.is_set() or home_event.is_set()):
                    if motion.paused:
                        motion.move_resume()
                    run_gate.set()

        time.sleep(0.2)


def robot_state_monitor_thread(
    stop_event: threading.Event,
    fault_event: threading.Event,
    motion: MotionController,
    sysctl: RobotSystemController,
):
    """
    [수정됨] 에러(fault_event)가 발생하면, 수복 작업이 원활하도록
    GetRobotState 요청을 멈추고 대기(Back-off)합니다.
    """
    supabase = create_client(SUPABASE_URL, SUPABASE_ANON_KEY)
    last_state = None
    last_db_push_t = 0.0
    last_error_db_push_t = 0.0

    while not stop_event.is_set():
        # [핵심 수정] 수리 중(fault_event Set)이면 로봇에게 말을 걸지 않고 쉽니다.
        # 이렇게 해야 handle_home_request의 SetRobotControl 명령이 씹히지 않습니다.
        if fault_event.is_set():
            time.sleep(0.5)
            continue

        # Reentrant 덕분에 이제 여기서 블로킹 덜 됨
        state = sysctl.get_robot_state(timeout=1.0)

        if state is not None and state >= 0:
            now = time.time()
            if state != last_state or (now - last_db_push_t) > 1.0:
                update_robot_state_row(supabase, doosan_robot_state=state)
                last_state = state
                last_db_push_t = now

            if state in ERROR_STATES:
                # 에러 최초 감지 시
                if not fault_event.is_set():
                    name = ROBOT_STATE_MAP.get(state, str(state))
                    sysctl.node.get_logger().error(f"🚧 ERROR detected: {state}({name}) -> Monitoring Paused for Recovery")
                    
                    # 1. 즉시 Fault 이벤트 발생 (이 순간부터 위쪽 if문에 걸려 루프가 멈춤)
                    fault_event.set() 
                    
                    # 2. DB 기록
                    now2 = time.time()
                    update_robot_state_row(supabase, status="error", recovery_needed=True)
                    last_error_db_push_t = now2

        time.sleep(0.2)


def get_db_doosan_robot_state(supabase: Client) -> int | None:
    try:
        res = (
            supabase.table(ROBOT_STATE_TABLE)
            .select("doosan_robot_state")
            .eq("id", ROBOT_STATE_ROW_ID)
            .limit(1)
            .execute()
        )
        if not res.data: return None
        return int(res.data[0].get("doosan_robot_state", -1))
    except Exception as e:
        print(f"❌ get_db_doosan_robot_state error: {e}")
        return None


def handle_home_request(
    supabase: Client,
    motion: MotionController,
    sysctl: RobotSystemController,
    run_gate: threading.Event,
    abort_event: threading.Event,
    home_event: threading.Event,
    fault_event: threading.Event,
) -> bool:
    
    node = sysctl.node
    node.get_logger().warn("🏠 HOME 요청 처리 시작 (복구 로직 포함)")

    run_gate.set()

    # 1) 현재 상태 확인
    live_state = sysctl.get_robot_state(timeout=1.0)
    db_state = get_db_doosan_robot_state(supabase)
    state = live_state if (live_state is not None and live_state >= 0) else db_state

    node.get_logger().warn(f"[HOME] Check state: live={live_state}, db={db_state} -> using {state}")

    if state is None:
        node.get_logger().error("❌ 상태 확인 불가")
        return False

    # 2) 수복 필요시 시도
    if state in ERROR_STATES:
        node.get_logger().warn("🔧 에러 상태이므로 수복(Recover) 시도합니다.")
        update_robot_state_row(supabase, status="error", recovery_needed=True)

        ok = sysctl.recover_if_possible(int(state))
        if not ok:
            node.get_logger().error("⛔ 수복 실패. 수동 조치 필요.")
            return False
        
        # 수복 성공 후 잠시 대기
        time.sleep(1.0)

    # 3) Resume (Paused 상태였다면)
    if getattr(motion, "paused", False):
        try:
            motion.move_resume()
        except Exception:
            pass

    # 4) HOME 이동
    try:
        go_home()
    except Exception as e:
        node.get_logger().error(f"❌ go_home 실패: {e}")
        update_robot_state_row(supabase, status="error", recovery_needed=True)
        return False

    # 5) 정리
    update_robot_state_row(
        supabase,
        desired_state="None",
        status="idle",
        recovery_needed=False,
    )

    home_event.clear()
    abort_event.clear()
    fault_event.clear()

    node.get_logger().info("✅ HOME / RECOVERY 처리 완료")
    return True


def perform_task(
    interval=50,
    run_gate: threading.Event = None,
    abort_event: threading.Event = None,
    fault_event: threading.Event = None,
):
    print(f"Performing task with interval: {interval}mm")

    from DSR_ROBOT2 import (
        posx, movej, movel, posj,
        task_compliance_ctrl, set_stiffnessx,
        release_compliance_ctrl,
        set_desired_force, DR_FC_MOD_ABS,
        get_tool_force, move_periodic, release_force,
        DR_MV_MOD_REL, DR_BASE
    )

    compliance_on = False

    def checkpoint():
        if fault_event is not None and fault_event.is_set():
            raise StopRequested("Fault detected")
        if abort_event is not None and abort_event.is_set():
            raise StopRequested("Abort requested")
        if run_gate is not None:
            while not run_gate.is_set():
                if fault_event is not None and fault_event.is_set():
                    raise StopRequested("Fault during pause")
                if abort_event is not None and abort_event.is_set():
                    raise StopRequested("Abort during pause")
                time.sleep(0.05)

    def movej_p(*args, **kwargs):
        checkpoint()
        return movej(*args, **kwargs)

    def movel_p(*args, **kwargs):
        checkpoint()
        return movel(*args, **kwargs)

    HOME = posj([0, 0, 90, 0, 90, -90])
    KNIFE_UP = posj([46.179, 9.846, 80.07, -5.218, 87.91, -43.519])
    KNIFE_START=posj([9.677, 3.308, 86.739, -0.065, 88.944, -80.221])

    NUM = 175 // interval

    try:
        movej_p(HOME, vel=VELOCITY, acc=ACC)
        grip_open()

        movej_p(KNIFE_UP, vel=VELOCITY, acc=ACC, r=10)
        movel_p(posx(0, 0, -108, 0, 0, 0), vel=l_VELOCITY, acc=l_ACC, mod=DR_MV_MOD_REL)
        grip_close()

        movel_p(posx(0, 0, 108, 0, 0, 0), vel=l_VELOCITY, acc=l_ACC, mod=DR_MV_MOD_REL)
        movej_p(KNIFE_START, vel=VELOCITY, acc=ACC)

        task_compliance_ctrl()
        compliance_on = True
        set_stiffnessx([3000, 3000, 5, 200, 200, 200], time=0)

        for i in range(NUM - 1):
            print(f"Slice {i+1}/{NUM}")
            movel_p(posx(0, -interval, 0, 0, 0, 0), vel=l_VELOCITY, acc=l_ACC, mod=DR_MV_MOD_REL)
            movel_p(posx(0, 0, -145, 0, 0, 0), vel=l_VELOCITY, acc=l_ACC, mod=DR_MV_MOD_REL)
            set_desired_force([0,0,-50,0,0,0],[0,0,1,0,0,0],time=0.0,mod=DR_FC_MOD_ABS)
            
            # Force Check Loop with Safety
            t_force_start = time.time()
            while True:
                checkpoint() # 루프 안에서도 체크
                force = get_tool_force()
                if force[2] > 25:
                    move_periodic(amp =[10,0,0,0,0,0], period=1.0, atime=0.2, repeat=5,ref=DR_BASE)
                    release_force(time=0) 
                    break
                # 무한루프 방지용 (옵션)
                if time.time() - t_force_start > 10.0:
                    print("Force check timeout, proceeding...")
                    release_force(time=0)
                    break
                time.sleep(0.05)

            movel_p(posx(0,0,145,0,0,0),vel=l_VELOCITY,acc=l_ACC,mod=DR_MV_MOD_REL)
        
        release_compliance_ctrl()
        compliance_on = False

        movej_p(KNIFE_UP, vel=VELOCITY, acc=ACC)
        movel_p(posx(0, 0, -125, 0, 0, 0), vel=l_VELOCITY, acc=l_ACC, mod=DR_MV_MOD_REL)
        
        task_compliance_ctrl()
        compliance_on = True
        set_stiffnessx([3000,3000,5,200,200,200],time=0)
        set_desired_force([0,0,-50,0,0,0],[0,0,1,0,0,0],time=0.0,mod=DR_FC_MOD_ABS)
        
        while True:
            checkpoint()
            force = get_tool_force()
            if force[2] > 30:
                release_force(time=0)
                break
            time.sleep(0.05)
            
        release_compliance_ctrl()
        compliance_on = False

        grip_open()
        movel_p(posx(0, 0, 110, 0, 0, 0), vel=l_VELOCITY, acc=l_ACC, mod=DR_MV_MOD_REL)
        movej_p(HOME, vel=VELOCITY, acc=ACC)

        print(f"Task completed!")

    finally:
        if compliance_on:
            try:
                release_compliance_ctrl()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("move_basic_web_only", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    print("🔗 Connecting to Supabase...")
    supabase: Client = create_client(SUPABASE_URL, SUPABASE_ANON_KEY)

    try:
        initialize_robot()
        print("🤖 Robot initialized successfully")
    except Exception as e:
        print(f"❌ Failed to initialize robot: {e}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        motion = MotionController(node, ROBOT_ID)
        print("✅ Motion services ready")
    except Exception as e:
        print(f"❌ Failed to init motion services: {e}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 3

    try:
        sysctl = RobotSystemController(node, ROBOT_ID)
        print("✅ System services ready")
    except Exception as e:
        print(f"❌ Failed to init system services: {e}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 4

    run_gate = threading.Event()
    run_gate.set()

    abort_event = threading.Event()
    home_event = threading.Event()
    fault_event = threading.Event()
    task_busy_event = threading.Event()
    threads_stop_event = threading.Event()

    watcher = threading.Thread(
        target=command_watcher_thread,
        args=(run_gate, abort_event, home_event, threads_stop_event, motion, fault_event),
        daemon=True,
    )
    watcher.start()

    monitor = threading.Thread(
        target=robot_state_monitor_thread,
        args=(threads_stop_event, fault_event, motion, sysctl),
        daemon=True,
    )
    monitor.start()

    print("👀 Waiting for tasks...")

    last_home_try = 0.0

    try:
        while rclpy.ok():
            
            # =========================================================
            # [핵심 수정] 자동 복구 트리거 로직 변경
            # 기존: if home_event.is_set(): ...
            # 변경: 에러(fault)가 있거나 홈 요청(home)이 있으면 복구 시도
            # =========================================================
            is_fault = fault_event.is_set()
            is_home_req = home_event.is_set()

            if is_home_req or is_fault:
                # 태스크가 안 바쁘거나, 이미 에러가 난 상태라면 진입
                can_run_recovery = (not task_busy_event.is_set()) or is_fault
                
                if can_run_recovery:
                    now = time.time()
                    # 너무 잦은 재시도 방지 (0.8초 쿨타임)
                    if now - last_home_try > 0.8:
                        print(f"🔄 Recovery/Home Triggered! (Fault={is_fault}, HomeReq={is_home_req})")
                        last_home_try = now
                        
                        # 복구 및 홈 이동 로직 실행
                        success = handle_home_request(
                            supabase, motion, sysctl, run_gate,
                            abort_event, home_event, fault_event
                        )
                        
                        if not success:
                            # 실패시 로그 남기고 잠시 대기 (CPU 폭주 방지)
                            print("⚠️ Recovery failed, retrying soon...")
                            time.sleep(1.0)
                            
                    time.sleep(0.1)
                    continue

            # ------------------------------------------------------------
            # 1) abort/fault/home 중이면 새 task 시작 금지
            #    (위의 if문에서 처리되지 않은 잔여 상태 대기)
            # ------------------------------------------------------------
            if abort_event.is_set() or fault_event.is_set() or home_event.is_set():
                time.sleep(0.2)
                continue

            # ------------------------------------------------------------
            # 2) pending task 확인
            # ------------------------------------------------------------
            task_id, interval = check_and_get_pending_task(supabase)

            if task_id is None:
                time.sleep(1.0)
                continue

            update_task_status(supabase, task_id, "running")
            print(f"🚀 Starting task {task_id} (interval={interval})")

            task_busy_event.set()
            try:
                perform_task(
                    interval,
                    run_gate=run_gate,
                    abort_event=abort_event,
                    fault_event=fault_event,
                )
                update_task_status(supabase, task_id, "completed")
                print(f"✅ Task {task_id} completed")

            except HomeRequested:
                update_task_status(supabase, task_id, "failed")
                print(f"🏠 Home requested -> task {task_id} aborted")

            except StopRequested as e:
                update_task_status(supabase, task_id, "failed")
                print(f"🛑 Stop/Fault requested ({e}) -> task {task_id} aborted")
                # 여기서 task_busy가 풀리고 다음 루프에서 `if is_fault:` 조건에 걸려 복구로 넘어감

            except Exception as e:
                update_task_status(supabase, task_id, "failed")
                print(f"❌ Task error: {e}")

            finally:
                task_busy_event.clear()

            print("🔄 Next task (checking system status)...")

    except KeyboardInterrupt:
        print("\n⚠️ Interrupted")

    finally:
        threads_stop_event.set()
        time.sleep(0.2)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    return 0

if __name__ == "__main__":
    sys.exit(main())