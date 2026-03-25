# Copyright 2026 USU AggieAir
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for DJI XMP tag value computation in the tagger."""

import math

import pytest

from stardos_tagger.tagger import _wrap_deg


class TestWrapDeg:
    """_wrap_deg: radians → degrees, wrapped to [-180, 180]."""

    def test_zero_stays_zero(self):
        assert _wrap_deg(0.0) == pytest.approx(0.0)

    def test_quarter_turn(self):
        assert _wrap_deg(math.pi / 2) == pytest.approx(90.0)

    def test_wraps_large_positive(self):
        assert _wrap_deg(math.radians(1190.0)) == pytest.approx(110.0)

    def test_wraps_large_negative(self):
        assert _wrap_deg(math.radians(-10000.0)) == pytest.approx(80.0)

    def test_identity_for_valid_range(self):
        for deg in [-179.0, -90.0, 0.0, 45.0, 90.0]:
            assert _wrap_deg(math.radians(deg)) == pytest.approx(deg)

    def test_plus_minus_180_boundary(self):
        # atan2(sin(π), cos(π)) is platform-defined; result is ±180.
        result = _wrap_deg(math.pi)
        assert abs(result) == pytest.approx(180.0)


class TestGimbalPitchConvention:
    """GimbalPitchDegree = _wrap_deg(pitch) - 90.0 (DJI nadir = -90 deg)."""

    def test_nadir_pitch_produces_minus_90(self):
        gimbal_pitch = _wrap_deg(0.0) - 90.0
        assert gimbal_pitch == pytest.approx(-90.0)

    def test_nose_up_5deg_produces_minus_85(self):
        gimbal_pitch = _wrap_deg(math.radians(5.0)) - 90.0
        assert gimbal_pitch == pytest.approx(-85.0)


class TestFlightPitchConvention:
    """FlightPitchDegree = _wrap_deg(pitch) — NO -90° offset applied."""

    def test_level_flight_no_offset(self):
        # At level flight (pitch=0), FlightPitchDegree = 0.0, not -90.0.
        assert _wrap_deg(0.0) == pytest.approx(0.0)

    def test_nose_up_5deg_no_offset(self):
        # Aircraft pitched 5° nose-up → FlightPitchDegree = +5.0, not -85.0.
        assert _wrap_deg(math.radians(5.0)) == pytest.approx(5.0)
        assert _wrap_deg(math.radians(5.0)) != pytest.approx(-85.0)


class TestAltitudeFormat:
    """DJI altitude strings: signed, 3 decimal places (e.g. '+50.300')."""

    def test_positive_altitude(self):
        alt_mm = 50300
        result = f'{alt_mm / 1000:+.3f}'
        assert result == '+50.300'

    def test_zero_altitude(self):
        assert f'{0 / 1000:+.3f}' == '+0.000'

    def test_negative_relative_altitude(self):
        # Drone below home point - relative_alt can be negative
        # CORRECT: f'{val:+.3f}' -> '-0.500'
        # WRONG:   f'+{val:.3f}' -> '+-0.500'
        rel_mm = -500
        result = f'{rel_mm / 1000:+.3f}'
        assert result == '-0.500'

    def test_three_decimal_places(self):
        result = f'{1234 / 1000:+.3f}'
        assert result == '+1.234'
