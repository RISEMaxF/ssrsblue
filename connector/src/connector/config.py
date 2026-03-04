from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Default BlueOS address. Override with CONNECTOR_BLUEOS_HOST in production.
    # The vessel uses 192.168.1.248 (Teltonika router network), not the
    # BlueOS default 192.168.2.2 DHCP range.
    blueos_host: str = "192.168.2.2"
    system_id: int = 254
    target_system: int = 1
    target_component: int = 1
    watchdog_timeout: float = 2.0
    log_level: str = "INFO"

    # GPS serial reader (empty port = disabled)
    gps_serial_port: str = ""
    gps_serial_baud: int = 9600
    # NOTE: When re-enabling GPS, also set CONNECTOR_GPS_UDP_HOST to the
    # actual BlueOS address (e.g. 192.168.1.248), not this default.
    gps_udp_host: str = "192.168.2.2"
    gps_udp_port: int = 27000

    model_config = {"env_prefix": "CONNECTOR_"}

    @property
    def mavlink_rest_url(self) -> str:
        return f"http://{self.blueos_host}/mavlink2rest/v1/mavlink"

    @property
    def mavlink_ws_url(self) -> str:
        return f"ws://{self.blueos_host}/mavlink2rest/v1/ws/mavlink"

    @property
    def gps_enabled(self) -> bool:
        return bool(self.gps_serial_port)
