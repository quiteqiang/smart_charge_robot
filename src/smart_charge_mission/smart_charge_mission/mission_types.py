"""Pure data types for the mission state machine (no rclpy imports).

Events are everything that can arrive from the outside world; commands are
everything MissionCore asks the rclpy shell to do in response. Keeping these
as plain dataclasses (rather than importing ROS message types) is what lets
mission_core.py stay entirely free of rclpy.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class MissionState(str, Enum):
    IDLE = 'IDLE'
    EXECUTING_TASK = 'EXECUTING_TASK'
    LOW_BATTERY = 'LOW_BATTERY'
    NAVIGATING_TO_DOCK = 'NAVIGATING_TO_DOCK'
    PRE_DOCKING = 'PRE_DOCKING'
    DOCKING = 'DOCKING'
    CHARGING = 'CHARGING'
    UNDOCKING = 'UNDOCKING'
    RESUMING_TASK = 'RESUMING_TASK'
    ERROR_WAITING_HUMAN = 'ERROR_WAITING_HUMAN'


# ------------------------------------------------------------ Events (in)

@dataclass(frozen=True)
class BatteryReading:
    now: float
    percentage: float
    charging: bool  # True <=> POWER_SUPPLY_STATUS_CHARGING


@dataclass(frozen=True)
class NavGoalResponse:
    now: float
    request_id: int
    accepted: bool


@dataclass(frozen=True)
class NavServerUnavailable:
    """Nav2's wait_for_server(timeout) expired for this request.

    Distinct from NavGoalResponse(accepted=False): a rejected goal goes
    through the normal nav_retries/backoff path, but server-unavailable goes
    straight to ERROR_WAITING_HUMAN today (see charge_mission_node._send_nav_goal),
    so it can't be collapsed into the same event without changing that behavior.
    """
    now: float
    request_id: int


@dataclass(frozen=True)
class NavResult:
    now: float
    request_id: int
    success: bool


@dataclass(frozen=True)
class DockResultReceived:
    now: float
    sequence: int
    success: bool


@dataclass(frozen=True)
class DockServiceResponse:
    now: float
    kind: str  # 'start' | 'undock'
    accepted: bool
    detail: str = ''


@dataclass(frozen=True)
class GotoRequested:
    now: float
    waypoint: str


@dataclass(frozen=True)
class StartTaskRequested:
    now: float


@dataclass(frozen=True)
class ResetRequested:
    now: float


@dataclass(frozen=True)
class TimerFired:
    now: float
    timer_id: int


@dataclass(frozen=True)
class TfCheckResult:
    now: float
    ok: bool


Event = (
    BatteryReading | NavGoalResponse | NavResult | NavServerUnavailable
    | DockResultReceived | DockServiceResponse | GotoRequested
    | StartTaskRequested | ResetRequested | TimerFired | TfCheckResult
)


# ------------------------------------------------------------ Commands (out)

@dataclass(frozen=True)
class PublishState:
    state: MissionState


@dataclass(frozen=True)
class PublishChargingActive:
    active: bool


@dataclass(frozen=True)
class SendNavGoal:
    request_id: int
    waypoint_name: str
    x: float
    y: float
    yaw: float
    frame: str
    source: str  # 'task' | 'dock' | 'resume_none'


@dataclass(frozen=True)
class CancelNavGoal:
    request_id: int


@dataclass(frozen=True)
class CallDockService:
    kind: str  # 'start' | 'undock'


@dataclass(frozen=True)
class ScheduleTimer:
    timer_id: int
    delay_s: float


@dataclass(frozen=True)
class TriggerAck:
    success: bool
    message: str = ''


@dataclass(frozen=True)
class Log:
    level: str  # 'info' | 'warn' | 'error'
    message: str


Command = (
    PublishState | PublishChargingActive | SendNavGoal | CancelNavGoal
    | CallDockService | ScheduleTimer | TriggerAck | Log
)
