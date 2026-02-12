import pytest

from connector.validators import (
    clamp,
    clamp_steering,
    clamp_throttle,
    is_controllable_mode,
    validate_mode,
)


class TestClamp:
    def test_within_range(self):
        assert clamp(500, 0, 1000) == 500

    def test_below_min(self):
        assert clamp(-2000, -1000, 1000) == -1000

    def test_above_max(self):
        assert clamp(9999, 0, 1000) == 1000

    def test_float_truncated(self):
        assert clamp(500.9, 0, 1000) == 500


class TestClampSteering:
    def test_normal(self):
        assert clamp_steering(200) == 200

    def test_clamps_high(self):
        assert clamp_steering(5000) == 1000

    def test_clamps_low(self):
        assert clamp_steering(-5000) == -1000


class TestClampThrottle:
    def test_neutral(self):
        assert clamp_throttle(0) == 0

    def test_forward(self):
        assert clamp_throttle(500) == 500

    def test_reverse(self):
        assert clamp_throttle(-500) == -500

    def test_clamps_negative(self):
        assert clamp_throttle(-2000) == -1000

    def test_clamps_high(self):
        assert clamp_throttle(2000) == 1000


class TestValidateMode:
    def test_by_name(self):
        assert validate_mode("GUIDED") == 15

    def test_by_name_case_insensitive(self):
        assert validate_mode("manual") == 0

    def test_by_number(self):
        assert validate_mode(3) == 3

    def test_invalid_name(self):
        with pytest.raises(ValueError, match="Unknown mode"):
            validate_mode("HOVER")

    def test_invalid_number(self):
        with pytest.raises(ValueError, match="Unknown mode number"):
            validate_mode(99)


class TestIsControllableMode:
    def test_guided(self):
        assert is_controllable_mode(15) is True

    def test_steering(self):
        assert is_controllable_mode(3) is True

    def test_manual(self):
        assert is_controllable_mode(0) is False

    def test_hold(self):
        assert is_controllable_mode(4) is False
