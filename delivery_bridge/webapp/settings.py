import os
import json
import logging
import requests
from delivery_bridge.utils import EnumWithDescription as Enum

logger = logging.getLogger(__name__)

# to get a string like this run: $ openssl rand -hex 32
SECRET_KEY = "6558c21a0068b9a84411f5037dcb76a422c0e73aad5a96bba92ded0d9bd5583f"
APP_DATA_DIR = os.path.join(os.path.expanduser("~"), ".delivery_bridge")


class Map:
    def __init__(
        self,
        TOPIC_NAME: str = "/map",
        TOPIC_TYPE: str = "nav_msgs/OccupancyGrid",
    ):
        self.TOPIC_NAME: str = TOPIC_NAME
        self.TOPIC_TYPE: str = TOPIC_TYPE

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()

class PoseTopic:
    def __init__(
        self,
        TOPIC_NAME: str,
        TOPIC_TYPE: str,
        MAX_EMIT_RATE: int = 0,
        COVARIANCE: list[float] = [],
    ):
        self.TOPIC_NAME: str = TOPIC_NAME
        self.TOPIC_TYPE: str = TOPIC_TYPE
        self.MAX_EMIT_RATE: int = MAX_EMIT_RATE
        self.COVARIANCE: list[float] = COVARIANCE

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()

class Battery:
    def __init__(
        self,
        TOPIC_NAME: str = "/battery",
        TOPIC_TYPE: str = "std_msgs/Float64",
        MAX_EMIT_RATE: int = 1,
        PERCENTAGE_ZERO_SAFE: float = 10.0,
        PERCENTAGE_FULL_SAFE: float = 95.0,
        PERCENTAGE_LOW: float = 20.0,
        VOLTAGE_ZERO: float = 20.0,
        VOLTAGE_FULL: float = 25.0,
    ):
        self.TOPIC_NAME: str = TOPIC_NAME
        self.TOPIC_TYPE: str = TOPIC_TYPE  # supported: std_msgs/Float64
        self.MAX_EMIT_RATE: int = MAX_EMIT_RATE  # Hz, 0 for unlimited
        # low threshold to show 0% battery
        self.PERCENTAGE_ZERO_SAFE: float = PERCENTAGE_ZERO_SAFE
        # high threshold to show 100% battery
        self.PERCENTAGE_FULL_SAFE: float = PERCENTAGE_FULL_SAFE
        self.PERCENTAGE_LOW: float = PERCENTAGE_LOW  # low threshold to show low battery
        self.VOLTAGE_ZERO: float = VOLTAGE_ZERO  #
        self.VOLTAGE_FULL: float = VOLTAGE_FULL  #

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()

class CmdVel:
    def __init__(
        self,
        TOPIC_NAME: str = "/cmd_vel",
        TOPIC_TYPE: str = "geometry_msgs/Twist",
    ):
        self.TOPIC_NAME: str = TOPIC_NAME
        self.TOPIC_TYPE: str = TOPIC_TYPE  # supported: geometry_msgs/Twist

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()


class FunctionMode(Enum):   # Functionality Modes
    STOP = "stopped", "Detenido"
    STATIC = "static", "Estático"
    DELIVERY = "delivery", "Entrega"
    CRUISER = "cruiser", "Crucero"


class Process:
    def __init__(
        self,
        ENVIRON_VARS: dict = {},
        COMMANDS: list[str] = [],
        # ERROR_CODES: list[str] = [],
    ):
        # The environment variables that the below commands need to run,
        # in the format "ENV_VAR_NAME": "ENV_VAR_VALUE"
        self.ENVIRON_VARS: dict = ENVIRON_VARS
        # The commands to execute to start the process, each command in a new string
        # separated by comma
        self.COMMANDS: list[str] = COMMANDS
        # The error codes that block the process, each error code in a new string
        # separated by comma
        # self.ERROR_CODES: list[str] = ERROR_CODES # ------------

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()


class FunctionManager:
    def __init__(
        self,
        FUNCTION_TO_LOAD_ON_STARTUP: FunctionMode = None,
        PROCESS_FOR_STATIC: Process = Process(),
        PROCESS_FOR_DELIVERY: Process = Process(
            COMMANDS=[
                # (
                #     "ros2 run robot_core amcl_robot_pose"
                # )
            ]
        ),
        PROCESS_FOR_CRUISER: Process = Process(
            COMMANDS=[] #[ ("ros2 run robot_core amcl_robot_pose") ]
        ),
    ):
        self.FUNCTION_TO_LOAD_ON_STARTUP: FunctionMode = (
            FUNCTION_TO_LOAD_ON_STARTUP  # mapping, manual
        )
        self.PROCESS_FOR_STATIC: Process = PROCESS_FOR_STATIC
        self.PROCESS_FOR_DELIVERY: Process = PROCESS_FOR_DELIVERY
        self.PROCESS_FOR_CRUISER: Process = PROCESS_FOR_CRUISER

    @classmethod
    def from_dict(cls, data: dict):
        data["PROCESS_FOR_STATIC"] = (
            Process.from_dict(data["PROCESS_FOR_STATIC"])
            if "PROCESS_FOR_STATIC" in data
            else Process()
        )
        data["PROCESS_FOR_DELIVERY"] = (
            Process.from_dict(data["PROCESS_FOR_DELIVERY"])
            if "PROCESS_FOR_DELIVERY" in data
            else Process()
        )
        data["PROCESS_FOR_CRUISER"] = (
            Process.from_dict(data["PROCESS_FOR_CRUISER"])
            if "PROCESS_FOR_CRUISER" in data
            else Process()
        )
        return cls(**data)

    def to_dict(self):
        data = self.__dict__.copy()
        data["PROCESS_FOR_STATIC"] = self.PROCESS_FOR_STATIC.to_dict()
        data["PROCESS_FOR_DELIVERY"] = self.PROCESS_FOR_DELIVERY.to_dict()
        data["PROCESS_FOR_CRUISER"] = self.PROCESS_FOR_CRUISER.to_dict()
        return data

    def get_process(self, mode: FunctionMode) -> Process:
        if mode == FunctionMode.STATIC:
            return self.PROCESS_FOR_STATIC
        elif mode == FunctionMode.DELIVERY:
            return self.PROCESS_FOR_DELIVERY
        elif mode == FunctionMode.CRUISER:
            return self.PROCESS_FOR_CRUISER
        else:
            return Process()


class Settings:
    def __init__(self):
        self.SECRET_KEY: str = SECRET_KEY #"yxio$l!%v7#mgh9736%2bsw$)+owmdt(_2q5_pvhaltz*s+p)#"
        # ROS TOPICS TO LISTEN
        self.MAP: Map = Map()
        self.POSE_TO_SET: str = "last_mode"  # "last_mode", "same", ""
        # Topic to set the robot pose
        self.INITIAL_POSE: PoseTopic = PoseTopic(
            "/initialpose",
            "geometry_msgs/PoseWithCovarianceStamped",
            # default covariance for the pose
            COVARIANCE=[
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.06853892326654787,
            ],
        )
        # Topic to get the robot pose 
        self.POSE: PoseTopic = PoseTopic(
            "/amcl_robot_pose",
            "geometry_msgs/Pose",
            MAX_EMIT_RATE=5,  # Hz, 0 for unlimited
        )
        self.BATTERY: Battery = Battery()
        self.CMD_VEL: CmdVel = CmdVel()

        # functions managers
        self.FUNCTION_MANAGER: FunctionManager = FunctionManager()

        self.load()

    def from_dict(self, data: dict):
        self.SECRET_KEY = (
            data["SECRET_KEY"]
            if "SECRET_KEY"
            else "yxio$l!%v7#mgh9736%2bsw$)+owmdt(_2q5_pvhaltz*s+p)#" #default value
        )
        self.POSE_TO_SET = data["POSE_TO_SET"] if "POSE_TO_SET" else "same"
        self.INITIAL_POSE = (
            PoseTopic.from_dict(data["INITIAL_POSE"])
            if "INITIAL_POSE" in data
            else PoseTopic()
        )
        self.POSE = PoseTopic.from_dict(data["POSE"]) if "POSE" in data else PoseTopic()
        self.BATTERY = (
            Battery.from_dict(data["BATTERY"]) if "BATTERY" in data else Battery()
        )
        self.CMD_VEL = (
            CmdVel.from_dict(data["CMD_VEL"]) if "CMD_VEL" in data else CmdVel()
        )
        self.FUNCTION_MANAGER = (
            FunctionManager.from_dict(data["FUNCTION_MANAGER"])
            if "FUNCTION_MANAGER" in data
            else FunctionManager()
        )

    def to_dict(self):
        data = self.__dict__.copy()
        data["MAP"] = self.MAP.to_dict()
        data["INITIAL_POSE"] = self.INITIAL_POSE.to_dict()
        data["POSE"] = self.POSE.to_dict()
        data["BATTERY"] = self.BATTERY.to_dict()
        data["CMD_VEL"] = self.CMD_VEL.to_dict()
        data["FUNCTION_MANAGER"] = self.FUNCTION_MANAGER.to_dict()
        return data

    def save(self):
        # create app data dir if not exists
        if not os.path.exists(APP_DATA_DIR):
            os.makedirs(APP_DATA_DIR)
        # save the settings to the file
        with open(APP_DATA_DIR + "/settings.json", "w") as file:
            file.write(json.dumps(self.to_dict(), indent=4))

    def load(self):
        # check if the file exists
        if os.path.exists(APP_DATA_DIR + "/settings.json"):
            # load the settings from the file
            with open(APP_DATA_DIR + "/settings.json", "r") as file:
                data = file.read()
                data = json.loads(data)
                self.from_dict(data)
        else:
            # save the settings to the file
            print("No settings file found, creating a new one.")
            self.save()

"""
class Endpoint:
    def __init__(
        self, URL: str = "", METHOD: str = "GET", PAYLOAD: dict = {}, TOKEN: str = ""
    ):
        self.URL: str = URL
        self.METHOD: str = METHOD
        self.PAYLOAD: dict = PAYLOAD
        self.TOKEN: str = TOKEN

    @classmethod
    def from_dict(cls, data: dict):
        return cls(**data)

    def to_dict(self):
        return self.__dict__.copy()

    def request(
        self, url=None, method=None, headers=None, payload=None, debug=False
    ) -> dict:
        try:
            if headers is None:
                headers = {}
            headers.update({"Content-Type": "application/json"})
            if url is None:
                url = self.URL
            if method is None:
                method = self.METHOD
            if payload is None:
                payload = {}
            if self.PAYLOAD:
                payload.update(self.PAYLOAD)

            if debug:
                logger.info(f"method: {method}")
                logger.info(f"url: {url}")
                logger.info(f"headers: {headers}")
                logger.info(f"payload[{type(payload)}]: {payload}")
            response1 = requests.request(
                method, url, headers=headers, data=json.dumps(payload)  # payload #
            )
            # check if response have json format
            try:
                response = response1.json()
            except Exception:
                response = {}
                response["text"] = response1.text
            response["status_code"] = response1.status_code
        except ConnectionError as e:
            logger.warning(f"NO HAY RESPUESTA:{e}")
            response = {"status_code": 500, "error": str(e)}
        except Exception as e:
            logger.error(f"ERROR: {e}")
            response = {"error": str(e)}

        if debug:
            logger.info(f"response: {response}")

        return response
"""


settings = Settings()

if __name__ == "__main__":
    settings.save()
    print(settings.to_dict())
