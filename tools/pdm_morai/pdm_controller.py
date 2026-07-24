# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from pdm_lite.team_code.ego_manual_control import EgoManualControl

from .morai_udp import MoraiVehicleState
from .pdm_config import PdmLiteMoraiConfig


ROOT_DIR = Path(__file__).resolve().parents[2]
TEAM_CODE_DIR = ROOT_DIR / "pdm_lite" / "team_code"
if str(TEAM_CODE_DIR) not in sys.path:
    sys.path.insert(0, str(TEAM_CODE_DIR))

from kinematic_bicycle_model import KinematicBicycleModel
from lateral_controller import LateralPIDController
from longitudinal_controller import LongitudinalLinearRegressionController


@dataclass(frozen=True)
class PdmLeadingVehicle:
    entity_id: str
    distance: float
    lateral_offset: float
    speed: float
    relative_speed: float | None = None
    center_distance: float | None = None


@dataclass(frozen=True)
class PdmMoraiStep:
    control: EgoManualControl
    target_speed: float
    raw_target_speed: float
    map_speed_limit: float | None
    curve_speed_cap: float | None
    current_speed: float
    route_points: int
    forecast_x: float
    forecast_y: float
    forecast_speed: float
    leading_vehicle_id: str | None = None
    leading_distance: float | None = None
    leading_speed: float | None = None


class PdmMoraiController:
    def __init__(self, target_speed: float, config: PdmLiteMoraiConfig | None = None):
        self.config = config or PdmLiteMoraiConfig()
        self.target_speed = float(target_speed)
        self.lateral = LateralPIDController(self.config)
        self.longitudinal = LongitudinalLinearRegressionController(self.config)
        self.ego_model = KinematicBicycleModel(self.config)
        self._filtered_target_speed: float | None = None
        self._last_control = EgoManualControl()

    def run_step(
        self,
        ego: MoraiVehicleState,
        route_points: np.ndarray,
        map_speed_limit: float | None = None,
        leading_vehicle: PdmLeadingVehicle | None = None,
    ) -> PdmMoraiStep:
        if len(route_points) < 2:
            control = EgoManualControl(steer=0.0, throttle=0.0, brake=1.0)
            self._last_control = control
            return PdmMoraiStep(
                control,
                0.0,
                self.target_speed,
                map_speed_limit,
                None,
                ego.speed,
                len(route_points),
                ego.x,
                ego.y,
                ego.speed,
                None,
                None,
                None,
            )

        vehicle_position = np.array([ego.x, ego.y], dtype=float)
        target_speed, raw_target_speed, curve_speed_cap = self._compute_target_speed(
            ego.speed,
            route_points,
            map_speed_limit,
            leading_vehicle,
        )
        steering = self.lateral.step(
            route_points[:, :2],
            ego.speed,
            vehicle_position,
            ego.heading_rad,
        )
        steering = self._limit_steering(steering, ego.speed)
        throttle, control_brake = self.longitudinal.get_throttle_and_brake(
            False,
            target_speed,
            ego.speed,
        )
        brake = self._brake_value(bool(control_brake), target_speed, ego.speed)
        if abs(target_speed - ego.speed) <= self.config.speed_hold_deadband and not control_brake:
            throttle = 0.0
            brake = 0.0
        if target_speed < 0.05 and ego.speed < self.config.minimum_speed_to_prevent_rolling_back:
            brake = 1.0

        control = self._smooth_control(EgoManualControl(
            steer=float(steering),
            throttle=float(throttle),
            brake=float(brake),
        )).clamped()
        forecast_location, _forecast_heading, forecast_speed = self.ego_model.forecast_ego_vehicle(
            np.array([ego.x, ego.y, ego.z], dtype=float),
            ego.heading_rad,
            np.array([ego.speed], dtype=float),
            np.array([control.steer, control.throttle, control.brake], dtype=float),
        )

        return PdmMoraiStep(
            control=control,
            target_speed=target_speed,
            raw_target_speed=raw_target_speed,
            map_speed_limit=map_speed_limit,
            curve_speed_cap=curve_speed_cap,
            current_speed=ego.speed,
            route_points=len(route_points),
            forecast_x=float(forecast_location[0]),
            forecast_y=float(forecast_location[1]),
            forecast_speed=float(np.asarray(forecast_speed).item()),
            leading_vehicle_id=leading_vehicle.entity_id if leading_vehicle else None,
            leading_distance=leading_vehicle.distance if leading_vehicle else None,
            leading_speed=leading_vehicle.speed if leading_vehicle else None,
        )

    def _compute_target_speed(
        self,
        current_speed: float,
        route_points: np.ndarray,
        map_speed_limit: float | None,
        leading_vehicle: PdmLeadingVehicle | None,
    ) -> tuple[float, float, float | None]:
        raw_target_speed = self.target_speed
        if map_speed_limit is not None and map_speed_limit > 0.0:
            raw_target_speed = min(
                raw_target_speed,
                map_speed_limit * self.config.route_speed_limit_ratio,
            )

        curve_speed_cap = self._curve_speed_cap(route_points)
        if curve_speed_cap is not None:
            raw_target_speed = min(raw_target_speed, curve_speed_cap)

        if leading_vehicle is not None:
            raw_target_speed = min(
                raw_target_speed,
                self._target_speed_wrt_leading_vehicle(
                    current_speed,
                    raw_target_speed,
                    leading_vehicle,
                ),
            )

        raw_target_speed = max(0.0, raw_target_speed)
        if self._filtered_target_speed is None:
            self._filtered_target_speed = min(
                raw_target_speed,
                current_speed + self.config.target_speed_increase_rate * self.config.time_step,
            )
        else:
            self._filtered_target_speed = self._rate_limit(
                self._filtered_target_speed,
                raw_target_speed,
                self.config.target_speed_increase_rate,
                self.config.target_speed_decrease_rate,
            )
        return self._filtered_target_speed, raw_target_speed, curve_speed_cap

    def _target_speed_wrt_leading_vehicle(
        self,
        current_speed: float,
        free_target_speed: float,
        leading_vehicle: PdmLeadingVehicle,
    ) -> float:
        gap = max(0.1, leading_vehicle.distance)
        lead_speed = max(0.0, leading_vehicle.speed)
        closing_speed = max(0.0, current_speed - lead_speed)
        if closing_speed > 0.1:
            ttc = gap / closing_speed
            if ttc < self.config.leading_vehicle_emergency_ttc:
                return 0.0

        accel = max(0.1, self.config.leading_vehicle_idm_acceleration)
        decel = max(0.1, self.config.leading_vehicle_idm_deceleration)
        desired_gap = (
            self.config.leading_vehicle_min_gap
            + current_speed * self.config.leading_vehicle_time_gap
            + current_speed * closing_speed / (2.0 * np.sqrt(accel * decel))
        )
        speed_ratio = current_speed / max(0.1, free_target_speed)
        idm_accel = accel * (
            1.0
            - speed_ratio ** self.config.leading_vehicle_idm_delta
            - (desired_gap / gap) ** 2
        )
        idm_cap = max(
            0.0,
            current_speed + idm_accel * self.config.leading_vehicle_idm_horizon,
        )

        if gap > desired_gap and lead_speed < free_target_speed:
            distance_margin = gap - desired_gap
            follow_cap = lead_speed + 0.25 * distance_margin
            return max(0.0, min(free_target_speed, max(idm_cap, follow_cap)))
        return max(0.0, min(free_target_speed, idm_cap))

    def _curve_speed_cap(self, route_points: np.ndarray) -> float | None:
        points = route_points[: self.config.curve_lookahead_points, :2]
        step = max(1, self.config.curve_sample_step_points)
        points = points[::step]
        if len(points) < 3:
            return None

        curvatures: list[float] = []
        for index in range(1, len(points) - 1):
            prev_vec = points[index] - points[index - 1]
            next_vec = points[index + 1] - points[index]
            prev_len = float(np.linalg.norm(prev_vec))
            next_len = float(np.linalg.norm(next_vec))
            if prev_len < 1e-3 or next_len < 1e-3:
                continue
            prev_heading = np.arctan2(prev_vec[1], prev_vec[0])
            next_heading = np.arctan2(next_vec[1], next_vec[0])
            heading_delta = abs((next_heading - prev_heading + np.pi) % (2.0 * np.pi) - np.pi)
            curvatures.append(float(heading_delta / max(0.5 * (prev_len + next_len), 1e-3)))

        if not curvatures:
            return None
        curvature = float(np.percentile(curvatures, 90))
        if curvature < 1e-4:
            return None
        speed_cap = float(np.sqrt(self.config.curve_lateral_accel_limit / curvature))
        return max(self.config.curve_speed_minimum, speed_cap)

    def _limit_steering(self, steering: float, current_speed: float) -> float:
        if current_speed <= self.config.steer_limit_start_speed:
            steer_limit = 1.0
        elif current_speed >= self.config.steer_limit_full_speed:
            steer_limit = self.config.max_steer_high_speed
        else:
            ratio = (
                (current_speed - self.config.steer_limit_start_speed)
                / (self.config.steer_limit_full_speed - self.config.steer_limit_start_speed)
            )
            steer_limit = 1.0 + ratio * (self.config.max_steer_high_speed - 1.0)
        return float(np.clip(steering, -steer_limit, steer_limit))

    def _brake_value(self, control_brake: bool, target_speed: float, current_speed: float) -> float:
        if not control_brake:
            return 0.0
        if target_speed < 0.05:
            return 1.0
        overspeed = current_speed - target_speed
        if overspeed <= self.config.overspeed_brake_margin:
            return 0.0
        return float(np.clip(0.2 + overspeed * self.config.overspeed_brake_gain, 0.2, 1.0))

    def _smooth_control(self, desired: EgoManualControl) -> EgoManualControl:
        steer = self._rate_limit_symmetric(
            self._last_control.steer,
            desired.steer,
            self.config.steer_rate_limit,
        )
        throttle = self._rate_limit(
            self._last_control.throttle,
            desired.throttle,
            self.config.throttle_increase_rate,
            self.config.throttle_decrease_rate,
        )
        brake = self._rate_limit(
            self._last_control.brake,
            desired.brake,
            self.config.brake_increase_rate,
            self.config.brake_decrease_rate,
        )
        if brake > 0.01:
            throttle = 0.0
        control = EgoManualControl(steer=steer, throttle=throttle, brake=brake).clamped()
        self._last_control = control
        return control

    def _rate_limit(
        self,
        previous: float,
        desired: float,
        increase_rate: float,
        decrease_rate: float,
    ) -> float:
        rate = increase_rate if desired >= previous else decrease_rate
        max_delta = rate * self.config.time_step
        return float(previous + np.clip(desired - previous, -max_delta, max_delta))

    def _rate_limit_symmetric(self, previous: float, desired: float, rate: float) -> float:
        max_delta = rate * self.config.time_step
        return float(previous + np.clip(desired - previous, -max_delta, max_delta))
