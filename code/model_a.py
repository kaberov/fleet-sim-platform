from pydantic import BaseModel, ConfigDict
from typing import Deque, List, Dict
from enum import Enum
from collections import deque


class BasePoint(BaseModel):
    x: float | None = None
    y: float | None = None

    def __str__(self):
        return f"({self.x}, {self.y})"


class BasePose(BaseModel):
    x: float | None = None
    y: float | None = None
    theta: float | None = None



class BaseVelocity(BaseModel):
    linear_x: float | None = 0
    angular_z: float | None = 0


class RobotStatus(Enum):
    Idle = 0
    Moving = 1
    ExecutingTask = 2
    LowBattery = 3
    Blocked = 4
    Waiting = 5
    EmergencyStop = 6
    Manual = 7


class BaseZone(BaseModel):
    minx: float | None = None
    maxx: float | None = None
    miny: float | None = None
    maxy: float | None = None


class BaseTask(BaseModel):
    id: int
    position: BasePoint
    robot_name: str
    active: bool


class BaseEvent(BaseModel):
    id: int
    time: str = f'{0}.{0:09d}'
    robot_name: str
    status: RobotStatus


class RobotTelemetry(BaseModel):
    model_config = ConfigDict(ser_json_bytes='utf8')
    time: str | None = f'{0}.{0:09d}'
    idle_timer: int | None = None
    status: RobotStatus = RobotStatus.Idle
    pose: BasePose | None = None
    noisy_pose: BasePose | None = None
    velocity: BaseVelocity
    goal: BasePoint | None = None
    path: Deque[BasePoint] = deque()
    img: str | None = None


class ManagerConfig(BaseModel):
    work_zone: BaseZone
    update_task_time: float = 1.0
    listen_names: List[str] = []


class ManagerTelemetry(BaseModel):
    time: str = f'{0}.{0:09d}'
    robot_followers: Dict[str, RobotTelemetry] = {}
    tasks: List[BaseTask] = []
    events: List[BaseEvent] = []
    path_graph: str = None
    config: ManagerConfig


class ApiConfig(BaseModel):
    img_wight: int = 0
    img_height: int = 0


class ApiTelemetry(BaseModel):
    current_follower_name: str | None = None
    follower_control: str | None = None
    config: ApiConfig
    manager_telemetry: ManagerTelemetry