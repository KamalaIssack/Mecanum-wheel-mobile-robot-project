"""Unit tests for the hardware-free logic in nucleo_bridge."""

from mecanum_control.nucleo_bridge import (
    fresh_goal,
    parse_four,
    parse_status,
    ramp_toward,
)
import pytest

ZEROS = [0.0, 0.0, 0.0, 0.0]
TARGET = [4.0, -4.0, 4.0, -4.0]


def test_parse_four_reads_padded_v_line():
    line = 'V,  +4.02,  -3.69,  +0.00,  +4.36'
    assert parse_four(line, 'V') == (4.02, -3.69, 0.0, 4.36)


def test_parse_four_reads_velocity_ack():
    assert parse_four('A,  +4.00,  +4.00,  -4.00,  +0.00', 'A') == (4.0, 4.0, -4.0, 0.0)


@pytest.mark.parametrize('line', [
    'V,1.0,2.0,3.0',
    'V,1.0,2.0,3.0,4.0,5.0',
    'V,1.0,2.0,x,4.0',
    'V,1.0,,3.0,4.0',
    'V,nan,0.0,0.0,0.0',
    'V,inf,0.0,0.0,0.0',
    '',
    'D,1.0,2.0,3.0,4.0',
])
def test_parse_four_rejects_bad_lines(line):
    assert parse_four(line, 'V') is None


def test_parse_four_ignores_gain_acks():
    assert parse_four('A,K,10.00', 'A') is None
    assert parse_four('A,I,100.00', 'A') is None


@pytest.mark.parametrize('line, state', [
    ('S,ARMED', 'ARMED'),
    ('S,DISARMED', 'DISARMED'),
    ('S,TIMEOUT', 'TIMEOUT'),
    ('S,armed', None),
    ('S,', None),
    ('V,1,2,3,4', None),
])
def test_parse_status(line, state):
    assert parse_status(line) == state


def test_no_command_yet_means_zeros():
    assert fresh_goal(TARGET, None, now=100.0, timeout=0.5) == ZEROS


def test_fresh_command_is_used():
    assert fresh_goal(TARGET, 100.0, now=100.4, timeout=0.5) == TARGET


def test_command_at_timeout_is_stale():
    assert fresh_goal(TARGET, 100.0, now=100.5, timeout=0.5) == ZEROS


def test_fresh_goal_returns_a_copy():
    goal = fresh_goal(TARGET, 100.0, now=100.1, timeout=0.5)
    goal[0] = 99.0
    assert TARGET[0] == 4.0


def test_ramp_reaches_4_in_two_steps():
    goal = [4.0] * 4
    first = ramp_toward(ZEROS, goal, max_step=2.0, max_speed=41.9)
    assert first == [2.0] * 4
    assert ramp_toward(first, goal, max_step=2.0, max_speed=41.9) == goal


def test_ramp_handles_each_wheel_and_direction_separately():
    out = ramp_toward(ZEROS, [-4.0, 4.0, 4.0, -4.0], max_step=2.0, max_speed=41.9)
    assert out == [-2.0, 2.0, 2.0, -2.0]


def test_ramp_small_change_lands_exactly():
    assert ramp_toward([4.0] * 4, [4.5] * 4, max_step=2.0, max_speed=41.9) == [4.5] * 4


def test_ramp_clamps_to_max_speed():
    out = ramp_toward([41.0] * 4, [100.0] * 4, max_step=2.0, max_speed=41.9)
    assert out == pytest.approx([41.9] * 4)
