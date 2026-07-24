# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_SUITE = (
    ROOT_DIR
    / "data"
    / "example-suite"
    # / "PDMLite UDP Car Following 2026-05-26"
    # / "PDMLite UDP Car Following 2026-05-26.msuite"
)

LOCAL_MGEO_ALIASES = {
    "KATRI": "KATRI",
    "K-City": "KATRI",
    "K-City (Old)": "KATRI",
    "Hwaseong-si": "KATRI",
    "R_KR_PR_Hwaseong-si": "KATRI",
    "R_KR_PG_K-City": "KATRI",
}


class ConfigError(RuntimeError):
    pass


@dataclass(frozen=True)
class XoscMetadata:
    entity_ids: list[str] = field(default_factory=list)
    external_control_entities: list[str] = field(default_factory=list)
    map_name: str | None = None
    logic_file: str | None = None


@dataclass(frozen=True)
class BindingConfig:
    source_path: Path
    suite_path: Path | None
    project_path: Path | None
    scenario_path: Path | None
    ego_id: str
    ctrl_ip: str
    ctrl_port: int
    ego_info_ip: str
    ego_info_port: int
    control_hz: int
    feedback_hz: int
    entity_ids: list[str]
    map_name: str | None = None
    logic_file: str | None = None
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class StartPose:
    x: float
    y: float
    z: float
    heading: float


@dataclass(frozen=True)
class ActorMetadata:
    entity_id: str
    object_type: str
    bbox_center: tuple[float, float, float] | None
    dimensions: tuple[float, float, float] | None
    properties: dict[str, str]


@dataclass(frozen=True)
class AdvancedRoute:
    entity_id: str
    name: str
    points: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class PdmMoraiScenario:
    binding: BindingConfig
    map_dir: Path
    target_speed: float
    start_pose: StartPose | None
    actor_metadata: dict[str, ActorMetadata]
    advanced_routes: dict[str, AdvancedRoute] = field(default_factory=dict)


DEFAULT_TARGET_SPEED_MPS = 10.0
MIN_VALID_TARGET_SPEED_MPS = 0.5
MAX_AUTODETECTED_TARGET_SPEED_MPS = 13.0


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as file:
        data = json.load(file)
    if not isinstance(data, dict):
        raise ConfigError(f"JSON root is not an object: {path}")
    return data


def _resolve_path(path: str | Path, base: Path | None = None) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute() and base is not None:
        candidate = base / candidate
    return candidate.expanduser().resolve(strict=False)


def _as_int(value: Any, default: int | None = None) -> int | None:
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_ip(value: Any, default: str | None = None) -> str | None:
    text = str(value or "").strip()
    return text or default


def _scenario_matches(entry: dict[str, Any], scenario_name: str) -> bool:
    name = str(entry.get("name") or "").strip()
    path = str(entry.get("path") or "").strip()
    return name == scenario_name or Path(path).stem == scenario_name


def _project_scenario_path(
    project_path: Path,
    project: dict[str, Any],
    scenario_name: str | None = None,
) -> Path | None:
    scenario_list = project.get("scenarioList") or []
    if not scenario_list:
        return None
    if scenario_name:
        matches = [
            scenario
            for scenario in scenario_list
            if isinstance(scenario, dict) and _scenario_matches(scenario, scenario_name)
        ]
        if not matches:
            available = ", ".join(
                str(scenario.get("name") or scenario.get("path") or "-")
                for scenario in scenario_list
                if isinstance(scenario, dict)
            )
            raise ConfigError(
                f"Scenario name was not found in project: {scenario_name}. "
                f"Available scenarios: {available}"
            )
        if len(matches) > 1:
            raise ConfigError(f"Scenario name is ambiguous in project: {scenario_name}")
        scenario_rel = matches[0].get("path")
    else:
        scenario_rel = scenario_list[0].get("path")
    return _resolve_path(scenario_rel, project_path.parent) if scenario_rel else None


def _load_suite(
    source_path: Path,
    scenario_name: str | None = None,
) -> tuple[Path, Path | None, dict[str, Any]]:
    suite = _read_json(source_path)
    project_list = suite.get("projectList") or []
    if not project_list:
        raise ConfigError(f"No projectList in suite: {source_path}")
    project_rel = project_list[0].get("filepath")
    if not project_rel:
        raise ConfigError(f"No project filepath in suite: {source_path}")
    project_path = _resolve_path(project_rel, source_path.parent)
    if not project_path.exists():
        raise ConfigError(f"Project file does not exist: {project_path}")
    project = _read_json(project_path)
    return project_path, _project_scenario_path(project_path, project, scenario_name), project


def _load_project(project_path: Path | None) -> dict[str, Any] | None:
    if project_path is None:
        return None
    if not project_path.exists():
        raise ConfigError(f"Project file does not exist: {project_path}")
    return _read_json(project_path)


def _find_project_for_xosc(xosc_path: Path) -> Path | None:
    for project_path in xosc_path.parent.glob("*.mproj"):
        try:
            project = _read_json(project_path)
        except (OSError, json.JSONDecodeError, ConfigError):
            continue
        for scenario in project.get("scenarioList") or []:
            rel = scenario.get("path") if isinstance(scenario, dict) else None
            if rel and _resolve_path(rel, project_path.parent) == xosc_path:
                return project_path
    projects = list(xosc_path.parent.glob("*.mproj"))
    return projects[0] if projects else None


def _find_suite_for_project(project_path: Path | None) -> Path | None:
    if project_path is None:
        return None
    for root in (project_path.parent.parent, project_path.parent):
        for suite_path in root.glob("*.msuite"):
            try:
                suite = _read_json(suite_path)
            except (OSError, json.JSONDecodeError, ConfigError):
                continue
            for project in suite.get("projectList") or []:
                rel = project.get("filepath") if isinstance(project, dict) else None
                if rel and _resolve_path(rel, suite_path.parent) == project_path:
                    return suite_path
    return None


def _load_interface_files(
    project_path: Path | None, project: dict[str, Any] | None
) -> dict[str, dict[str, Any]]:
    if project_path is None or project is None:
        return {}
    interfaces: dict[str, dict[str, Any]] = {}
    for entry in project.get("interfaceList") or []:
        rel_path = entry.get("path") if isinstance(entry, dict) else None
        if not rel_path:
            continue
        interface_path = _resolve_path(rel_path, project_path.parent)
        if not interface_path.exists():
            continue
        data = _read_json(interface_path)
        iface = data.get("interface") or {}
        name = str(iface.get("name") or entry.get("name") or interface_path.stem).lower()
        direction = str(iface.get("direction") or "").upper()
        template = str(iface.get("messageTemplateFileName") or "").lower()
        if "manual" in name or direction == "RECEIVE" or "manual" in template:
            interfaces["manual"] = data
        if "vehicle info" in name or direction == "SEND" or "vehicle info" in template:
            interfaces["feedback"] = data
    return interfaces


def _parse_xosc_metadata(path: Path | None) -> XoscMetadata:
    if path is None or not path.exists():
        return XoscMetadata()
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return XoscMetadata()

    entity_ids: list[str] = []
    external_control_entities: list[str] = []
    map_name: str | None = None
    logic_file: str | None = None
    for element in root.iter():
        tag = element.tag.split("}")[-1]
        if tag == "VariableDeclaration" and element.get("name") == "moraiMap":
            map_name = element.get("value")
        if tag == "LogicFile":
            logic_file = element.get("filepath")
        if tag != "ScenarioObject":
            continue
        name = element.get("name")
        if name:
            entity_ids.append(name)
        for prop in element.iter():
            if prop.tag.split("}")[-1] == "Property" and prop.get("name") == "external_control":
                if name and str(prop.get("value")).lower() == "true":
                    external_control_entities.append(name)
    return XoscMetadata(
        entity_ids=entity_ids,
        external_control_entities=external_control_entities,
        map_name=map_name,
        logic_file=logic_file,
    )


def _manual_endpoint(data: dict[str, Any] | None) -> tuple[str | None, int | None, int | None]:
    if not data:
        return None, None, None
    iface = data.get("interface") or {}
    host = iface.get("hostAddr") or {}
    return _as_ip(host.get("ip")), _as_int(host.get("port")), _as_int(iface.get("hertz"))


def _feedback_endpoint(data: dict[str, Any] | None) -> tuple[str | None, int | None, int | None]:
    if not data:
        return None, None, None
    iface = data.get("interface") or {}
    destination = iface.get("destinationAddr") or {}
    return _as_ip(destination.get("ip")), _as_int(destination.get("port")), _as_int(iface.get("hertz"))


def _detect_ego_id(metadata: XoscMetadata) -> str:
    external = metadata.external_control_entities
    if len(external) == 1:
        return external[0]
    if "Ego" in metadata.entity_ids:
        return "Ego"
    for entity_id in metadata.entity_ids:
        if entity_id.lower() == "ego":
            return entity_id
    return metadata.entity_ids[0] if metadata.entity_ids else "Ego"


def resolve_binding_config(source: str | Path, scenario_name: str | None = None) -> BindingConfig:
    source_path = _resolve_path(source)
    if not source_path.exists():
        raise ConfigError(f"Source file does not exist: {source_path}")

    suffix = source_path.suffix.lower()
    suite_path: Path | None = None
    project_path: Path | None = None
    scenario_path: Path | None = None
    project: dict[str, Any] | None = None
    warnings: list[str] = []

    if suffix == ".msuite":
        suite_path = source_path
        project_path, scenario_path, project = _load_suite(source_path, scenario_name)
    elif suffix == ".xosc":
        scenario_path = source_path
        project_path = _find_project_for_xosc(source_path)
        suite_path = _find_suite_for_project(project_path)
        project = _load_project(project_path)
    else:
        raise ConfigError("Open a .msuite or .xosc file.")

    interfaces = _load_interface_files(project_path, project)
    metadata = _parse_xosc_metadata(scenario_path)

    manual_ip, manual_port, manual_hz = _manual_endpoint(interfaces.get("manual"))
    feedback_ip, feedback_port, feedback_hz = _feedback_endpoint(interfaces.get("feedback"))

    ctrl_ip = manual_ip or "127.0.0.1"
    ctrl_port = manual_port
    ego_info_ip = feedback_ip or "127.0.0.1"
    ego_info_port = feedback_port
    control_hz = manual_hz or 60
    feedback_hz = feedback_hz or 30
    ego_id = _detect_ego_id(metadata)

    if ctrl_port is None or ctrl_port <= 0:
        raise ConfigError("Control UDP port was not found in Manual Control interface.")
    if ego_info_port is None or ego_info_port <= 0:
        raise ConfigError("Ego info UDP port was not found in Vehicle Info interface.")

    entity_ids = metadata.entity_ids
    if ego_id not in entity_ids and entity_ids:
        warnings.append(f"ego_id '{ego_id}' is not in loaded entity list")
    if project_path is None:
        warnings.append("No .mproj was found beside the xosc; interface defaults are used")

    return BindingConfig(
        source_path=source_path,
        suite_path=suite_path,
        project_path=project_path,
        scenario_path=scenario_path,
        ego_id=ego_id,
        ctrl_ip=ctrl_ip,
        ctrl_port=ctrl_port,
        ego_info_ip=ego_info_ip,
        ego_info_port=ego_info_port,
        control_hz=control_hz,
        feedback_hz=feedback_hz,
        entity_ids=entity_ids,
        map_name=metadata.map_name,
        logic_file=metadata.logic_file,
        warnings=tuple(warnings),
    )


def _project_json(project_path: Path | None) -> dict[str, Any]:
    if project_path is None or not project_path.exists():
        return {}
    with project_path.open("r", encoding="utf-8-sig") as file:
        data = json.load(file)
    return data if isinstance(data, dict) else {}


def _float_attr(element: ET.Element | None, name: str) -> float | None:
    if element is None:
        return None
    value = element.get(name)
    return float(value) if value is not None else None


def _parse_xosc(
    path: Path | None,
    ego_id: str,
    map_dir: Path,
) -> tuple[float | None, StartPose | None, dict[str, ActorMetadata], dict[str, AdvancedRoute]]:
    if path is None or not path.exists():
        return None, None, {}, {}

    root = ET.parse(path).getroot()
    target_speed: float | None = None
    start_pose: StartPose | None = None
    actor_metadata: dict[str, ActorMetadata] = {}
    advanced_routes = _parse_advanced_routes(root)

    for element in root.iter():
        tag = element.tag.split("}")[-1]
        if tag == "ScenarioObject":
            entity_id = element.get("name")
            vehicle = None
            for child in element:
                if child.tag.split("}")[-1] == "Vehicle":
                    vehicle = child
                    break
            if entity_id and vehicle is not None:
                center = None
                dimensions = None
                properties: dict[str, str] = {}
                for child in vehicle.iter():
                    child_tag = child.tag.split("}")[-1]
                    if child_tag == "Center":
                        x = _float_attr(child, "x")
                        y = _float_attr(child, "y")
                        z = _float_attr(child, "z")
                        if x is not None and y is not None and z is not None:
                            center = (x, y, z)
                    elif child_tag == "Dimensions":
                        length = _float_attr(child, "length")
                        width = _float_attr(child, "width")
                        height = _float_attr(child, "height")
                        if length is not None and width is not None and height is not None:
                            dimensions = (length, width, height)
                    elif child_tag == "Property":
                        name = child.get("name")
                        value = child.get("value")
                        if name and value is not None:
                            properties[name] = value
                actor_metadata[entity_id] = ActorMetadata(
                    entity_id=entity_id,
                    object_type=vehicle.get("vehicleCategory") or "vehicle",
                    bbox_center=center,
                    dimensions=dimensions,
                    properties=properties,
                )
        if tag == "ParameterDeclaration" and element.get("name") == "v_ego":
            value = element.get("value")
            if value is not None:
                target_speed = float(value)
        if tag != "Private" or element.get("entityRef") != ego_id:
            continue
        world_position = None
        lane_position = None
        for child in element.iter():
            child_tag = child.tag.split("}")[-1]
            if child_tag == "WorldPosition":
                world_position = child
                break
            if child_tag == "LanePosition":
                lane_position = child
        if lane_position is not None:
            start_pose = _start_pose_from_lane_position(lane_position, map_dir)
            if start_pose is not None:
                break
        if world_position is not None:
            start_pose = StartPose(
                x=float(world_position.get("x", "0")),
                y=float(world_position.get("y", "0")),
                z=float(world_position.get("z", "0")),
                heading=float(world_position.get("h", "0")),
            )
            break

    return target_speed, start_pose, actor_metadata, advanced_routes


def _start_pose_from_lane_position(
    lane_position: ET.Element,
    map_dir: Path,
) -> StartPose | None:
    lane_id = lane_position.get("laneId")
    if not lane_id:
        return None
    with (map_dir / "link_set.json").open("r", encoding="utf-8-sig") as file:
        link = next(
            (item for item in json.load(file) if str(item.get("idx")) == lane_id),
            None,
        )
    points = link.get("points") if link else None
    if not isinstance(points, list) or len(points) < 2:
        return None

    remaining = max(0.0, float(lane_position.get("s", "0")))
    offset = float(lane_position.get("offset", "0"))
    start = points[0]
    end = points[1]
    ratio = 0.0
    for segment_start, segment_end in zip(points, points[1:]):
        dx = float(segment_end[0]) - float(segment_start[0])
        dy = float(segment_end[1]) - float(segment_start[1])
        dz = float(segment_end[2]) - float(segment_start[2])
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        start, end = segment_start, segment_end
        if length > 0.0 and remaining <= length:
            ratio = remaining / length
            break
        remaining -= length
        ratio = 1.0

    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    xy_length = math.hypot(dx, dy)
    x = float(start[0]) + dx * ratio
    y = float(start[1]) + dy * ratio
    z = float(start[2]) + (float(end[2]) - float(start[2])) * ratio
    if xy_length > 0.0:
        x += -dy / xy_length * offset
        y += dx / xy_length * offset
    orientation = next(
        (
            child
            for child in lane_position
            if child.tag.split("}")[-1] == "Orientation"
        ),
        None,
    )
    heading = (
        float(orientation.get("h", "0"))
        if orientation is not None
        else math.atan2(dy, dx)
    )
    return StartPose(x=x, y=y, z=z, heading=heading)


def _position_from_route_item(item: Any) -> tuple[float, float, float] | None:
    if not isinstance(item, dict):
        return None
    position = item.get("position")
    if not isinstance(position, dict):
        return None
    try:
        return (
            float(position["x"]),
            float(position["y"]),
            float(position.get("z", 0.0)),
        )
    except (KeyError, TypeError, ValueError):
        return None


def _parse_advanced_routes(root: ET.Element) -> dict[str, AdvancedRoute]:
    routes: dict[str, AdvancedRoute] = {}
    for element in root.iter():
        if element.tag.split("}")[-1] != "CustomCommandAction":
            continue
        if element.get("type") != "AdvancedRouting":
            continue
        if not element.text:
            continue
        try:
            payload = json.loads(element.text)
        except json.JSONDecodeError:
            continue
        entity_id = str(payload.get("entityRef") or "")
        if not entity_id:
            continue
        route_payload = payload.get("route") if isinstance(payload.get("route"), dict) else payload
        route_name = str(route_payload.get("name") or payload.get("name") or entity_id)
        points: list[tuple[float, float, float]] = []
        initial = _position_from_route_item(route_payload.get("initialWaypoint"))
        if initial is not None:
            points.append(initial)
        for waypoint in route_payload.get("waypoints") or []:
            point = _position_from_route_item(waypoint)
            if point is not None:
                points.append(point)
        if len(points) >= 2:
            routes[entity_id] = AdvancedRoute(
                entity_id=entity_id,
                name=route_name,
                points=tuple(points),
            )
    return routes


def _target_speed_from_project(project: dict[str, Any], ego_id: str) -> float | None:
    agent_model_groups = [project.get("AgentModels") or []]
    for scenario in project.get("scenarioList") or []:
        config = scenario.get("config") if isinstance(scenario, dict) else None
        if isinstance(config, dict):
            agent_model_groups.append(config.get("AgentModels") or [])
    for agent_models in agent_model_groups:
        for agent_model in agent_models:
            if agent_model.get("agentName") == ego_id and "desiredSpeed" in agent_model:
                return float(agent_model["desiredSpeed"])
    return None


def _local_mgeo_name(map_name: str) -> str:
    return LOCAL_MGEO_ALIASES.get(map_name, map_name)


def _valid_target_speed(value: float | None) -> float | None:
    if value is None:
        return None
    if value < MIN_VALID_TARGET_SPEED_MPS:
        return None
    if value > MAX_AUTODETECTED_TARGET_SPEED_MPS:
        return None
    return value


def _resolve_mgeo_dir(binding: BindingConfig, project: dict[str, Any]) -> Path:
    if binding.scenario_path is not None and binding.logic_file:
        candidate = _resolve_path(binding.logic_file, binding.scenario_path.parent)
        if candidate.is_dir() and (candidate / "link_set.json").exists():
            return candidate
        if candidate.is_dir() and (candidate / "MGeo" / "link_set.json").exists():
            return candidate / "MGeo"

    map_name = binding.map_name or project.get("map", {}).get("name")
    if not map_name:
        raise RuntimeError("MORAI map name was not found in xosc/project.")
    map_dir = ROOT_DIR / "data" / "morai" / "mgeo" / _local_mgeo_name(str(map_name))
    if not map_dir.exists():
        raise RuntimeError(f"MGeo map directory does not exist: {map_dir}")
    nested = map_dir / "MGeo"
    if nested.exists() and (nested / "link_set.json").exists():
        return nested
    return map_dir


def load_scenario(
    source: str | Path = DEFAULT_SUITE,
    scenario_name: str | None = None,
) -> PdmMoraiScenario:
    binding = resolve_binding_config(source, scenario_name)
    project = _project_json(binding.project_path)
    map_dir = _resolve_mgeo_dir(binding, project)
    xosc_speed, start_pose, actor_metadata, advanced_routes = _parse_xosc(
        binding.scenario_path,
        binding.ego_id,
        map_dir,
    )
    project_speed = _target_speed_from_project(project, binding.ego_id)
    target_speed = _valid_target_speed(xosc_speed) or _valid_target_speed(project_speed)
    if target_speed is None:
        target_speed = DEFAULT_TARGET_SPEED_MPS
    return PdmMoraiScenario(
        binding=binding,
        map_dir=map_dir,
        target_speed=target_speed,
        start_pose=start_pose,
        actor_metadata=actor_metadata,
        advanced_routes=advanced_routes,
    )
