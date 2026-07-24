# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class LinkRef:
    idx: str
    from_node_idx: str
    to_node_idx: str
    points: np.ndarray
    road_id: str
    ego_lane: int | None
    max_speed_kph: float | None
    can_move_left_lane: bool
    can_move_right_lane: bool


@dataclass(frozen=True)
class RouteProjection:
    s: float
    lateral_offset: float
    distance: float
    heading: float
    segment_index: int


def _as_lane(value: Any) -> int | None:
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _heading_of_points(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    delta = points[-1, :2] - points[0, :2]
    return math.atan2(float(delta[1]), float(delta[0]))


def _angle_diff(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def resample_polyline(points: np.ndarray, spacing: float) -> np.ndarray:
    if len(points) < 2:
        return points.copy()

    result = [points[0]]
    carry = 0.0
    current = points[0].copy()
    for target in points[1:]:
        segment = target - current
        length = float(np.linalg.norm(segment[:2]))
        if length < 1e-6:
            continue
        direction = segment / length
        distance = spacing - carry
        while distance <= length:
            current = current + direction * distance
            result.append(current.copy())
            segment = target - current
            length = float(np.linalg.norm(segment[:2]))
            distance = spacing
            if length < 1e-6:
                break
        carry = float(np.linalg.norm((target - current)[:2]))
        current = target.copy()
    if np.linalg.norm((result[-1] - points[-1])[:2]) > 1e-6:
        result.append(points[-1])
    return np.asarray(result, dtype=float)


class MGeoRoutePlanner:
    def __init__(self, map_dir: Path, spacing: float = 0.1):
        self.map_dir = map_dir
        self.spacing = spacing
        self.links: dict[str, LinkRef] = {}
        self.links_from_node: dict[str, list[LinkRef]] = {}
        self.current_link_ids: list[str] = []
        self.current_links: list[LinkRef] = []
        self.route_points = np.empty((0, 3), dtype=float)
        self.route_s = np.empty((0,), dtype=float)
        self._load_links()

    def _load_links(self) -> None:
        with (self.map_dir / "link_set.json").open("r", encoding="utf-8-sig") as file:
            raw_links = json.load(file)
        for raw in raw_links:
            points = np.asarray(raw.get("points") or [], dtype=float)
            if len(points) < 2:
                continue
            link = LinkRef(
                idx=str(raw["idx"]),
                from_node_idx=str(raw.get("from_node_idx") or ""),
                to_node_idx=str(raw.get("to_node_idx") or ""),
                points=points,
                road_id=str(raw.get("road_id") or ""),
                ego_lane=_as_lane(raw.get("ego_lane")),
                max_speed_kph=_as_float(raw.get("max_speed")),
                can_move_left_lane=bool(raw.get("can_move_left_lane")),
                can_move_right_lane=bool(raw.get("can_move_right_lane")),
            )
            self.links[link.idx] = link
            self.links_from_node.setdefault(link.from_node_idx, []).append(link)

    def rebuild_from_pose(
        self,
        x: float,
        y: float,
        heading_rad: float,
        preferred_lane: int | None = None,
        min_route_length: float = 300.0,
    ) -> None:
        start = self._nearest_link(x, y, heading_rad, preferred_lane)
        links = [start]
        visited = {start.idx}
        while self._route_length(links) < min_route_length:
            next_link = self._next_link(links[-1], preferred_lane)
            if next_link is None or next_link.idx in visited:
                break
            links.append(next_link)
            visited.add(next_link.idx)

        points = self._merge_links(links)
        self.current_links = links
        self.current_link_ids = [link.idx for link in links]
        self.route_points = resample_polyline(points, self.spacing)
        self.route_s = self._cumulative_distance(self.route_points)

    def set_route_points(self, points: np.ndarray, route_id: str) -> None:
        if len(points) < 2:
            raise RuntimeError(f"AdvancedRouting '{route_id}' has fewer than two points.")
        self.current_links = []
        self.current_link_ids = [route_id]
        self.route_points = resample_polyline(np.asarray(points, dtype=float), self.spacing)
        self.route_s = self._cumulative_distance(self.route_points)

    def segment_ahead(self, x: float, y: float, count: int = 260) -> np.ndarray:
        if len(self.route_points) < 2:
            return self.route_points
        deltas = self.route_points[:, :2] - np.array([x, y])
        start = int(np.argmin(np.einsum("ij,ij->i", deltas, deltas)))
        end = min(len(self.route_points), start + count)
        segment = self.route_points[start:end]
        if len(segment) >= 2:
            return segment
        return self.route_points[max(0, len(self.route_points) - count) :]

    def distance_to_route(self, x: float, y: float) -> float:
        if len(self.route_points) == 0:
            return float("inf")
        deltas = self.route_points[:, :2] - np.array([x, y])
        return float(np.sqrt(np.min(np.einsum("ij,ij->i", deltas, deltas))))

    def speed_limit_at(self, x: float, y: float) -> float | None:
        link = self._nearest_current_link(x, y)
        if link is None or link.max_speed_kph is None or link.max_speed_kph <= 0:
            return None
        return link.max_speed_kph / 3.6

    def lane_change_available_at(self, x: float, y: float) -> tuple[bool, bool]:
        link = self._nearest_current_link(x, y)
        if link is None:
            return False, False
        return link.can_move_left_lane, link.can_move_right_lane

    def project(self, x: float, y: float) -> RouteProjection | None:
        if len(self.route_points) < 2:
            return None
        position = np.array([x, y], dtype=float)
        starts = self.route_points[:-1, :2]
        ends = self.route_points[1:, :2]
        segments = ends - starts
        lengths_sq = np.einsum("ij,ij->i", segments, segments)
        valid = lengths_sq > 1e-9
        if not np.any(valid):
            return None

        rel = position - starts
        t = np.zeros(len(segments), dtype=float)
        t[valid] = np.clip(
            np.einsum("ij,ij->i", rel[valid], segments[valid]) / lengths_sq[valid],
            0.0,
            1.0,
        )
        projected = starts + segments * t[:, None]
        deltas = position - projected
        distances_sq = np.einsum("ij,ij->i", deltas, deltas)
        index = int(np.argmin(distances_sq))
        segment = segments[index]
        segment_length = float(math.sqrt(max(lengths_sq[index], 1e-9)))
        heading = math.atan2(float(segment[1]), float(segment[0]))
        # Positive lateral offset is left of the current route tangent.
        lateral = float((segment[0] * deltas[index, 1] - segment[1] * deltas[index, 0]) / segment_length)
        s = float(self.route_s[index] + t[index] * segment_length)
        return RouteProjection(
            s=s,
            lateral_offset=lateral,
            distance=float(math.sqrt(distances_sq[index])),
            heading=heading,
            segment_index=index,
        )

    def _nearest_current_link(self, x: float, y: float) -> LinkRef | None:
        links = self.current_links or list(self.links.values())
        if not links:
            return None
        position = np.array([x, y], dtype=float)
        return min(
            links,
            key=lambda link: float(
                np.min(np.einsum("ij,ij->i", link.points[:, :2] - position, link.points[:, :2] - position))
            ),
        )

    def _nearest_link(
        self, x: float, y: float, heading_rad: float, preferred_lane: int | None
    ) -> LinkRef:
        position = np.array([x, y], dtype=float)
        candidates: list[tuple[float, LinkRef]] = []
        for link in self.links.values():
            if not link.road_id:
                continue
            deltas = link.points[:, :2] - position
            distance = float(np.sqrt(np.min(np.einsum("ij,ij->i", deltas, deltas))))
            heading_penalty = _angle_diff(_heading_of_points(link.points), heading_rad)
            lane_penalty = 0.0
            if preferred_lane is not None and link.ego_lane != preferred_lane:
                lane_penalty = 2.0
            length_bonus = min(5.0, len(link.points) / 100.0)
            score = distance + 2.0 * heading_penalty + lane_penalty - 0.01 * length_bonus
            candidates.append((score, link))
        if not candidates:
            raise RuntimeError(f"No drivable MGeo links in {self.map_dir}")
        return min(candidates, key=lambda item: item[0])[1]

    def _next_link(self, link: LinkRef, preferred_lane: int | None) -> LinkRef | None:
        candidates = [
            candidate
            for candidate in self.links_from_node.get(link.to_node_idx, [])
            if candidate.road_id
        ]
        if not candidates:
            return None
        if preferred_lane is None:
            preferred_lane = link.ego_lane
        if preferred_lane is not None:
            same_lane = [candidate for candidate in candidates if candidate.ego_lane == preferred_lane]
            if same_lane:
                candidates = same_lane
        heading = _heading_of_points(link.points)
        return min(
            candidates,
            key=lambda candidate: _angle_diff(_heading_of_points(candidate.points), heading),
        )

    def _route_length(self, links: list[LinkRef]) -> float:
        total = 0.0
        for link in links:
            diffs = np.diff(link.points[:, :2], axis=0)
            total += float(np.sum(np.linalg.norm(diffs, axis=1)))
        return total

    def _cumulative_distance(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return np.empty((0,), dtype=float)
        if len(points) == 1:
            return np.zeros((1,), dtype=float)
        diffs = np.diff(points[:, :2], axis=0)
        lengths = np.linalg.norm(diffs, axis=1)
        return np.concatenate(([0.0], np.cumsum(lengths)))

    def _merge_links(self, links: list[LinkRef]) -> np.ndarray:
        pieces = []
        for index, link in enumerate(links):
            points = link.points
            if index > 0 and len(points) > 1:
                points = points[1:]
            pieces.append(points)
        return np.vstack(pieces)
