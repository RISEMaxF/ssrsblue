from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    blueos_host: str = "192.168.2.2"
    system_id: int = 254
    target_system: int = 1
    target_component: int = 1
    watchdog_timeout: float = 2.0
    log_level: str = "INFO"

    model_config = {"env_prefix": "CONNECTOR_"}

    @property
    def mavlink_rest_url(self) -> str:
        return f"http://{self.blueos_host}/mavlink2rest/mavlink"

    @property
    def mavlink_ws_url(self) -> str:
        return f"ws://{self.blueos_host}/mavlink2rest/ws/mavlink"
