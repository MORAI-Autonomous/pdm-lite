from __future__ import annotations

from dataclasses import dataclass


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


@dataclass(frozen=True)
class EgoManualControl:
    """Small CARLA/PDM-Lite compatible ego control container."""

    steer: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0

    def clamped(self) -> "EgoManualControl":
        return EgoManualControl(
            steer=_clamp(self.steer, -1.0, 1.0),
            throttle=_clamp(self.throttle, 0.0, 1.0),
            brake=_clamp(self.brake, 0.0, 1.0),
        )

    def as_morai_udp_tuple(self) -> tuple[float, float, float]:
        control = self.clamped()
        return control.throttle, control.brake, control.steer

    @classmethod
    def from_morai_udp_tuple(
        cls, throttle: float, brake: float, steering_wheel_angle: float
    ) -> "EgoManualControl":
        return cls(
            steer=steering_wheel_angle,
            throttle=throttle,
            brake=brake,
        ).clamped()
