from unittest.mock import MagicMock, patch

from connector.gps_reader import GPSReader
from connector.gps_state import GPSState
from connector.config import Settings


def _make_reader() -> tuple[GPSReader, GPSState]:
    config = Settings(gps_serial_port="/dev/ttyUSB0")
    state = GPSState()
    reader = GPSReader(config, state)
    return reader, state


class TestParseSentence:
    def test_gga(self):
        reader, state = _make_reader()
        line = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F"
        reader._parse_sentence(line)

        assert state.sentences_received == 1
        assert state.fix_quality == 1
        assert state.satellites == 8
        assert abs(state.hdop - 0.9) < 0.01
        assert abs(state.altitude - 545.4) < 0.1
        assert abs(state.lat - 48.1173) < 0.001
        assert abs(state.lon - 11.51667) < 0.001
        assert "12:35:19" in state.utc_time

    def test_rmc(self):
        reader, state = _make_reader()
        line = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
        reader._parse_sentence(line)

        assert state.sentences_received == 1
        assert abs(state.speed_knots - 22.4) < 0.1
        assert abs(state.course - 84.4) < 0.1
        assert abs(state.lat - 48.1173) < 0.001
        assert abs(state.lon - 11.51667) < 0.001

    def test_invalid_sentence(self):
        reader, state = _make_reader()
        reader._parse_sentence("not an nmea sentence")
        assert state.parse_errors == 1
        assert state.sentences_received == 0

    def test_empty_fields_gga(self):
        reader, state = _make_reader()
        # GGA with no fix (empty lat/lon/alt)
        line = "$GPGGA,123519,,,,,0,00,,,,,,,*66"
        reader._parse_sentence(line)
        assert state.fix_quality == 0
        assert state.satellites == 0
        assert state.lat == 0.0
        assert state.lon == 0.0


class TestForwardUDP:
    def test_sends_to_socket(self):
        reader, _ = _make_reader()
        mock_sock = MagicMock()
        reader._udp_sock = mock_sock

        reader._forward_udp("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47")

        mock_sock.sendto.assert_called_once()
        data, addr = mock_sock.sendto.call_args[0]
        assert data.endswith(b"\r\n")
        assert addr == ("192.168.2.2", 27000)

    def test_no_socket(self):
        reader, _ = _make_reader()
        reader._udp_sock = None
        # Should not raise
        reader._forward_udp("$GPGGA,test")

    def test_socket_error(self):
        reader, _ = _make_reader()
        mock_sock = MagicMock()
        mock_sock.sendto.side_effect = OSError("Network unreachable")
        reader._udp_sock = mock_sock
        # Should not raise
        reader._forward_udp("$GPGGA,test")
