# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


PDM_PROFILE_CHOICES = ("normal", "mad-max", "beginner", "idiot")


@dataclass
class PdmLiteMoraiConfig:
    """CARLA-free subset of pdm_lite.team_code.config.GlobalConfig."""

    points_per_meter: int = 10
    route_points: int = 10
    fps: float = 20.0
    time_step: float = 1.0 / 20.0

    minimum_speed_to_prevent_rolling_back: float = 0.5

    route_speed_limit_ratio: float = 0.72
    curve_speed_minimum: float = 3.0
    curve_lateral_accel_limit: float = 1.8
    curve_lookahead_points: int = 320
    curve_sample_step_points: int = 10

    target_speed_increase_rate: float = 1.8
    target_speed_decrease_rate: float = 4.0
    leading_vehicle_time_gap: float = 1.4
    leading_vehicle_min_gap: float = 7.0
    leading_vehicle_speed_drop_gain: float = 0.45
    leading_vehicle_lateral_window: float = 2.2
    leading_vehicle_max_distance: float = 65.0
    leading_vehicle_heading_window_deg: float = 70.0
    leading_vehicle_idm_acceleration: float = 1.4
    leading_vehicle_idm_deceleration: float = 3.0
    leading_vehicle_idm_delta: float = 4.0
    leading_vehicle_idm_horizon: float = 0.9
    leading_vehicle_emergency_ttc: float = 0.9
    ego_length: float = 4.635
    speed_hold_deadband: float = 0.20
    overspeed_brake_margin: float = 0.15
    overspeed_brake_gain: float = 0.30

    max_steer_high_speed: float = 0.28
    steer_limit_start_speed: float = 6.0
    steer_limit_full_speed: float = 20.0
    steer_rate_limit: float = 1.0
    throttle_increase_rate: float = 1.6
    throttle_decrease_rate: float = 3.0
    brake_increase_rate: float = 3.5
    brake_decrease_rate: float = 5.0

    longitudinal_linear_regression_minimum_target_speed: float = 0.278
    longitudinal_linear_regression_params: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                1.1990342347353184,
                -0.8057602384167799,
                1.710818710950062,
                0.921890257450335,
                1.556497522998393,
                -0.7013479734904027,
                1.031266635497984,
            ]
        )
    )
    longitudinal_linear_regression_maximum_acceleration: float = 1.89
    longitudinal_linear_regression_maximum_deceleration: float = -4.82

    lateral_pid_kp: float = 1.4
    lateral_pid_kd: float = 0.35
    lateral_pid_ki: float = 0.05
    lateral_pid_speed_scale: float = 2.0
    lateral_pid_speed_offset: float = 5.0
    lateral_pid_window_size: int = 8

    front_wheel_base: float = -0.090769015
    rear_wheel_base: float = 1.4178275
    steering_gain: float = 0.36848336
    brake_acceleration: float = -4.952399
    throttle_acceleration: float = 0.5633837
    throttle_values: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                9.63873001e-01,
                4.37535692e-04,
                -3.80192912e-01,
                1.74950069e00,
                9.16787414e-02,
                -7.05461530e-02,
                -1.05996152e-03,
                6.71079346e-04,
            ]
        )
    )
    brake_values: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                9.31711370e-03,
                8.20967431e-02,
                -2.83832427e-03,
                5.06587474e-05,
                -4.90357228e-07,
                2.44419284e-09,
                -4.91381935e-12,
            ]
        )
    )
    throttle_threshold_during_forecasting: float = 0.3

    @property
    def lateral_pid_default_lookahead(self) -> float:
        return 6.0 * self.points_per_meter

    @property
    def lateral_pid_speed_threshold(self) -> float:
        return 6.0 * self.points_per_meter

    @property
    def lateral_pid_minimum_lookahead_distance(self) -> float:
        return 5.0 * self.points_per_meter

    @property
    def lateral_pid_maximum_lookahead_distance(self) -> float:
        return 18.0 * self.points_per_meter


def apply_pdm_profile(config: PdmLiteMoraiConfig, profile: str) -> None:
    if profile == "normal":
        return
    if profile == "mad-max":
        config.route_speed_limit_ratio = 1.10
        config.curve_lateral_accel_limit = 4.0
        config.target_speed_increase_rate = 5.0
        config.target_speed_decrease_rate = 1.5
        config.leading_vehicle_time_gap = 0.35
        config.leading_vehicle_min_gap = 1.5
        config.leading_vehicle_lateral_window = 1.1
        config.leading_vehicle_max_distance = 25.0
        config.leading_vehicle_idm_acceleration = 3.5
        config.leading_vehicle_idm_deceleration = 1.2
        config.leading_vehicle_idm_horizon = 0.4
        config.leading_vehicle_emergency_ttc = 0.25
        config.overspeed_brake_margin = 2.0
        config.overspeed_brake_gain = 0.10
        config.max_steer_high_speed = 0.45
        config.steer_rate_limit = 4.0
        config.throttle_increase_rate = 5.0
        config.throttle_decrease_rate = 5.0
        config.brake_increase_rate = 1.2
        config.brake_decrease_rate = 6.0
        return
    if profile == "beginner":
        config.route_speed_limit_ratio = 0.45
        config.curve_lateral_accel_limit = 0.9
        config.target_speed_increase_rate = 0.6
        config.target_speed_decrease_rate = 1.6
        config.leading_vehicle_time_gap = 2.8
        config.leading_vehicle_min_gap = 15.0
        config.leading_vehicle_lateral_window = 3.0
        config.leading_vehicle_max_distance = 90.0
        config.leading_vehicle_idm_acceleration = 0.7
        config.leading_vehicle_idm_deceleration = 2.0
        config.leading_vehicle_idm_horizon = 1.8
        config.leading_vehicle_emergency_ttc = 2.0
        config.overspeed_brake_margin = 0.05
        config.overspeed_brake_gain = 0.55
        config.max_steer_high_speed = 0.18
        config.steer_rate_limit = 0.35
        config.throttle_increase_rate = 0.5
        config.throttle_decrease_rate = 1.0
        config.brake_increase_rate = 1.2
        config.brake_decrease_rate = 1.8
        return
    if profile == "idiot":
        config.route_speed_limit_ratio = 0.95
        config.curve_lateral_accel_limit = 2.8
        config.target_speed_increase_rate = 2.8
        config.target_speed_decrease_rate = 0.5
        config.leading_vehicle_time_gap = 0.1
        config.leading_vehicle_min_gap = 0.5
        config.leading_vehicle_lateral_window = 0.65
        config.leading_vehicle_max_distance = 12.0
        config.leading_vehicle_idm_acceleration = 2.0
        config.leading_vehicle_idm_deceleration = 0.8
        config.leading_vehicle_idm_horizon = 0.25
        config.leading_vehicle_emergency_ttc = 0.12
        config.overspeed_brake_margin = 3.0
        config.overspeed_brake_gain = 0.04
        config.max_steer_high_speed = 0.22
        config.steer_rate_limit = 0.25
        config.throttle_increase_rate = 2.5
        config.throttle_decrease_rate = 0.6
        config.brake_increase_rate = 0.35
        config.brake_decrease_rate = 1.0
        return
    raise ValueError(f"Unknown PDM profile: {profile}")


def pdm_profile_target_speed_scale(profile: str) -> float:
    if profile == "normal":
        return 1.0
    if profile == "mad-max":
        return 1.35
    if profile == "beginner":
        return 0.60
    if profile == "idiot":
        return 1.10
    raise ValueError(f"Unknown PDM profile: {profile}")
