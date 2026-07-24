# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = ROOT_DIR / "data" / "generated-suite"
GENERATED_PROJECT_NAME = "PDM MORAI Project"
DEFAULT_LOCAL_MAP_NAME = "KATRI"
SIM_MAP_NAME_BY_ID = {
    100001: "Sangam-dong",
    100002: "KATRI (Old)",
    100003: "Hwaseong-si",
    100004: "Pangyo",
    100005: "KIAPI",
    100006: "Gyeongbu Expressway",
    100007: "Sangam Track",
    100008: "KATECH",
    100009: "Yong-In Circuit",
    100010: "TYMICT",
    100011: "KATRI",
    110001: "Virtual TestBed 01",
    110002: "Hwaseong-si_E8",
    110003: "Woowa24",
    110004: "Testfield_Dynamics",
    110005: "Testfield_Sensor",
    110006: "Nato AVT 408",
    110007: "Off-Road Showcase",
    110008: "ROKA Training Center A",
    110009: "ROKA Training Center B",
    110010: "Hanwha Turret",
    110011: "Region 06",
    110012: "Morai City",
}
SIM_MAP_ID_BY_NAME = {
    name: map_id for map_id, name in SIM_MAP_NAME_BY_ID.items()
}
SIM_MAP_ID_BY_NAME.update(
    {
        "seoul": 100001,
        "kcity_katri": 100002,
        "K-City (Old)": 100002,
        "hwaseong": 100003,
        "pangyo": 100004,
        "kiapi": 100005,
        "gyeongbu_expressway": 100006,
        "sangam_racing_track": 100007,
        "dt_su_katech": 100008,
        "dt_su_yongin_circuit": 100009,
        "dt_gr_tymict": 100010,
        "dt_su_kcity": 100011,
        "K-City": 100011,
        "testbed_01": 110001,
        "hwaseong_e8": 110002,
        "lv_woowa24": 110003,
        "testfield_dynamics": 110004,
        "testfield_sensor": 110005,
        "nato_avt_408": 110006,
        "vm_gr_off_road": 110007,
        "vm_su_roka_trainingcenter_a": 110008,
        "vm_su_roka_trainingcenter_b": 110009,
        "vm_su_hanwha_turret": 110010,
        "dt_region_06": 110011,
        "vm_ub_morai_city": 110012,
    }
)
LOCAL_MGEO_BY_SIM_MAP_ID = {
    100002: DEFAULT_LOCAL_MAP_NAME,
    100003: DEFAULT_LOCAL_MAP_NAME,
    100011: DEFAULT_LOCAL_MAP_NAME,
}
LOCAL_MGEO_BY_MAP_NAME = {
    "KATRI (Old)": DEFAULT_LOCAL_MAP_NAME,
    "K-City (Old)": DEFAULT_LOCAL_MAP_NAME,
    "Hwaseong-si": DEFAULT_LOCAL_MAP_NAME,
    "R_KR_PR_Hwaseong-si": DEFAULT_LOCAL_MAP_NAME,
    "R_KR_PG_K-City": DEFAULT_LOCAL_MAP_NAME,
    "KATRI": DEFAULT_LOCAL_MAP_NAME,
    "K-City": DEFAULT_LOCAL_MAP_NAME,
}
MODEL3D_TO_VEHICLE_NAME = {
    "HD Ioniq5": "gv_car_hdioniq5_2023",
    "HD Ioniq6": "gv_car_hdioniq6_2023",
}


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    h: float


@dataclass(frozen=True)
class VehicleInfo:
    entity_id: str
    vehicle_name: str
    vehicle_category: str
    role: str | None
    bbox_center: tuple[float, float, float] | None
    dimensions: tuple[float, float, float] | None
    initial_pose: Pose | None
    initial_speed_mps: float | None


@dataclass(frozen=True)
class XoscInfo:
    path: Path
    parameters: dict[str, str]
    morai_map: str | None
    logic_file: str | None
    vehicles: dict[str, VehicleInfo]
    ego_entity: str | None
    npc_entities: list[str]
    runner_map_name: str | None
    target_suite_path: Path
    target_project_path: Path
    target_xosc_path: Path
    in_place: bool
    source_project_map: dict[str, Any] | None = None
    source_scenario_config: dict[str, Any] | None = None
    warnings: list[str] = field(default_factory=list)
    blocking_errors: list[str] = field(default_factory=list)


class PrepareError(RuntimeError):
    pass


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as file:
        data = json.load(file)
    if not isinstance(data, dict):
        raise PrepareError(f"JSON root is not an object: {path}")
    return data


def local_name(element: ET.Element) -> str:
    return element.tag.split("}")[-1]


def as_float(value: str | None, parameters: dict[str, str]) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if text.startswith("$"):
        text = parameters.get(text[1:], "")
    if text == "":
        return None
    return float(text)


def safe_name(value: str) -> str:
    name = re.sub(r'[<>:"/\\|?*]+', " ", value)
    name = re.sub(r"\s+", " ", name).strip(" .")
    return name or "prepared_xosc"


def format_float(value: float | None) -> str:
    return "-" if value is None else f"{value:.6g}"


def format_pose(pose: Pose | None) -> str:
    if pose is None:
        return "-"
    return f"x={pose.x:.6g}, y={pose.y:.6g}, z={pose.z:.6g}, h={pose.h:.6g}"


def find_first(element: ET.Element, name: str) -> ET.Element | None:
    for child in element.iter():
        if local_name(child) == name:
            return child
    return None


LEGACY_SCENARIO_CONFIG_KEYS = (
    "weatherPreset",
    "sensorList",
    "kineticsModelList",
    "DynamicsCustomData",
    "vehicleAgentTypeList",
)


def source_project_context(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    for project_path in sorted(path.parent.glob("*.mproj")):
        try:
            project = read_json(project_path)
        except (OSError, PrepareError, json.JSONDecodeError):
            continue
        project_map = project.get("map") if isinstance(project.get("map"), dict) else None
        for scenario in project.get("scenarioList") or []:
            if not isinstance(scenario, dict):
                continue
            scenario_path = scenario.get("path")
            if not scenario_path:
                continue
            resolved = (project_path.parent / str(scenario_path)).resolve(strict=False)
            if resolved == path.resolve(strict=False):
                config = scenario.get("config") if isinstance(scenario.get("config"), dict) else None
                if config is None:
                    legacy_config = {
                        key: copy.deepcopy(project[key])
                        for key in LEGACY_SCENARIO_CONFIG_KEYS
                        if key in project
                    }
                    config = legacy_config or None
                return copy.deepcopy(project_map), copy.deepcopy(config)
    return None, None


def source_project_path(path: Path) -> Path | None:
    projects = sorted(path.parent.glob("*.mproj"))
    return projects[0] if projects else None


def project_map_id(project_map: dict[str, Any] | None) -> int | None:
    if not project_map:
        return None
    for key in ("id", "iD", "ID"):
        value = project_map.get(key)
        if value is None:
            continue
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    return None


def ensure_properties(vehicle: ET.Element) -> ET.Element:
    for child in vehicle:
        if local_name(child) == "Properties":
            return child
    return ET.SubElement(vehicle, "Properties")


def upsert_property(vehicle: ET.Element, name: str, value: str) -> None:
    properties = ensure_properties(vehicle)
    for child in properties:
        if local_name(child) == "Property" and child.get("name") == name:
            child.set("value", value)
            return
    ET.SubElement(properties, "Property", {"name": name, "value": value})


def remove_property(vehicle: ET.Element, name: str) -> None:
    properties = ensure_properties(vehicle)
    for child in list(properties):
        if local_name(child) == "Property" and child.get("name") == name:
            properties.remove(child)


def normalize_vehicle_model_name(vehicle: ET.Element, entity_id: str) -> None:
    model_name = MODEL3D_TO_VEHICLE_NAME.get(vehicle.get("model3d") or "")
    if model_name and vehicle.get("name") in (None, "", entity_id):
        vehicle.set("name", model_name)


def child_by_local_name(element: ET.Element | None, name: str) -> ET.Element | None:
    if element is None:
        return None
    for child in element:
        if local_name(child) == name:
            return child
    return None


def sanitize_vehicle_geometry(vehicle: ET.Element, parameters: dict[str, str]) -> None:
    bbox = child_by_local_name(vehicle, "BoundingBox")
    center = child_by_local_name(bbox, "Center")
    dimensions = child_by_local_name(bbox, "Dimensions")
    axles = child_by_local_name(vehicle, "Axles")
    front_axle = child_by_local_name(axles, "FrontAxle")
    rear_axle = child_by_local_name(axles, "RearAxle")
    if center is None or dimensions is None or front_axle is None or rear_axle is None:
        return

    center_x = as_float(center.get("x"), parameters)
    length = as_float(dimensions.get("length"), parameters)
    front_x = as_float(front_axle.get("positionX"), parameters)
    rear_x = as_float(rear_axle.get("positionX"), parameters)
    if center_x is None or length is None or front_x is None or rear_x is None:
        return
    if length <= 0.0 or front_x <= rear_x:
        return

    front_overhang = center_x + length * 0.5 - front_x
    rear_overhang = rear_x - (center_x - length * 0.5)
    if front_overhang > 0.0 and rear_overhang > 0.0:
        return

    wheelbase = front_x - rear_x
    if length <= wheelbase:
        length = wheelbase + 0.2
        dimensions.set("length", f"{length:.6g}")
    center.set("x", f"{(front_x + rear_x) * 0.5:.6g}")


def ensure_object_controller(scenario_object: ET.Element) -> None:
    if any(local_name(child) == "ObjectController" for child in scenario_object):
        return
    for controller_type, name, properties in (
        ("all", "Preset", {"name": "Standard"}),
        ("longitudinal", "Longitudinal", {}),
        ("lateral", "Lateral", {}),
    ):
        object_controller = ET.SubElement(scenario_object, "ObjectController")
        controller = ET.SubElement(
            object_controller,
            "Controller",
            {"controllerType": controller_type, "name": name},
        )
        controller_properties = ET.SubElement(controller, "Properties")
        for prop_name, prop_value in properties.items():
            ET.SubElement(
                controller_properties,
                "Property",
                {"name": prop_name, "value": prop_value},
            )


def scrub_xml_text_nodes(root: ET.Element) -> None:
    for element in root.iter():
        if local_name(element) != "CustomCommandAction" and element.text and element.text.strip():
            element.text = None
        if element.tail and element.tail.strip():
            element.tail = None


def sanitize_sun_angles(root: ET.Element, parameters: dict[str, str]) -> None:
    for element in root.iter():
        if local_name(element) != "Sun":
            continue
        azimuth = as_float(element.get("azimuth"), parameters)
        if azimuth is not None:
            if azimuth < 0.0 or azimuth > 2.0 * math.pi:
                azimuth = math.radians(azimuth) % (2.0 * math.pi)
            element.set("azimuth", f"{azimuth:.6g}")
        elevation = as_float(element.get("elevation"), parameters)
        if elevation is not None:
            if abs(elevation) > math.pi:
                elevation = math.radians(elevation)
            element.set("elevation", f"{elevation:.6g}")


def is_speed_private_action(element: ET.Element) -> bool:
    if local_name(element) != "PrivateAction":
        return False
    return any(local_name(child) == "SpeedAction" for child in element.iter())


def is_teleport_private_action(element: ET.Element) -> bool:
    if local_name(element) != "PrivateAction":
        return False
    return any(local_name(child) == "TeleportAction" for child in element.iter())


def move_initial_speed_before_teleport(root: ET.Element, entity_id: str) -> None:
    for private in root.iter():
        if local_name(private) != "Private" or private.get("entityRef") != entity_id:
            continue
        actions = [child for child in list(private) if local_name(child) == "PrivateAction"]
        speed_action = next((child for child in actions if is_speed_private_action(child)), None)
        teleport_action = next((child for child in actions if is_teleport_private_action(child)), None)
        if speed_action is None or teleport_action is None:
            return
        if list(private).index(speed_action) < list(private).index(teleport_action):
            return
        private.remove(speed_action)
        private.insert(list(private).index(teleport_action), speed_action)
        return


def force_initial_speed(root: ET.Element, entity_id: str, speed_mps: float) -> None:
    for private in root.iter():
        if local_name(private) != "Private" or private.get("entityRef") != entity_id:
            continue
        for speed in private.iter():
            if local_name(speed) == "AbsoluteTargetSpeed":
                speed.set("value", f"{speed_mps:g}")
                return


def parse_xosc(
    path: Path,
    ego_override: str | None,
    output_root: Path,
    suite_path: Path | None = None,
    in_place: bool = False,
) -> XoscInfo:
    if not path.exists():
        raise PrepareError(f"XOSC file does not exist: {path}")

    root = ET.parse(path).getroot()
    parameters: dict[str, str] = {}
    for element in root.iter():
        if local_name(element) == "ParameterDeclaration":
            name = element.get("name")
            value = element.get("value")
            if name and value is not None:
                parameters[name] = value

    morai_map: str | None = None
    for element in root.iter():
        if local_name(element) == "VariableDeclaration" and element.get("name") == "moraiMap":
            morai_map = element.get("value")
            break

    logic_file: str | None = None
    logic = find_first(root, "LogicFile")
    if logic is not None:
        logic_file = logic.get("filepath")
    source_map, source_config = source_project_context(path)

    vehicle_elements: dict[str, ET.Element] = {}
    for scenario_object in root.iter():
        if local_name(scenario_object) != "ScenarioObject":
            continue
        entity_id = scenario_object.get("name")
        if not entity_id:
            continue
        for child in scenario_object:
            if local_name(child) == "Vehicle":
                vehicle_elements[entity_id] = child
                break

    ego_entity = detect_ego(list(vehicle_elements), ego_override)
    warnings: list[str] = []
    blocking_errors: list[str] = []
    if ego_entity is None:
        blocking_errors.append("Cannot safely determine Ego from this XOSC. Use --ego-entity.")
    elif ego_entity not in vehicle_elements:
        blocking_errors.append(f"Requested --ego-entity '{ego_entity}' is not a vehicle ScenarioObject.")

    runner_map_name = resolve_runner_map(
        logic_file,
        morai_map,
        warnings,
        source_project_map=source_map,
    )
    if runner_map_name is None:
        blocking_errors.append("Cannot map XOSC road network to a local MGeo map directory.")
    mgeo_dir = (
        ROOT_DIR / "data" / "morai" / "mgeo" / runner_map_name
        if runner_map_name
        else None
    )
    poses, speeds = parse_initial_state(root, parameters, mgeo_dir)
    if ego_entity:
        speeds[ego_entity] = 0.0
    vehicles: dict[str, VehicleInfo] = {}
    for entity_id, vehicle in vehicle_elements.items():
        center = parse_xyz(find_first(vehicle, "Center"), ("x", "y", "z"), parameters)
        dimensions = parse_xyz(find_first(vehicle, "Dimensions"), ("length", "width", "height"), parameters)
        vehicles[entity_id] = VehicleInfo(
            entity_id=entity_id,
            vehicle_name=vehicle.get("name") or "",
            vehicle_category=vehicle.get("vehicleCategory") or "vehicle",
            role=vehicle.get("role"),
            bbox_center=center,
            dimensions=dimensions,
            initial_pose=poses.get(entity_id),
            initial_speed_mps=speeds.get(entity_id),
        )

    if ego_entity and vehicles.get(ego_entity) and vehicles[ego_entity].initial_pose is None:
        blocking_errors.append(
            f"Ego '{ego_entity}' has no supported initial WorldPosition or LanePosition."
        )

    target_suite_path, project_path, target_xosc_path = resolve_target_paths(
        path,
        output_root,
        suite_path,
        in_place,
        warnings,
        blocking_errors,
    )
    npc_entities = [name for name in vehicles if name != ego_entity]

    return XoscInfo(
        path=path,
        parameters=parameters,
        morai_map=morai_map,
        logic_file=logic_file,
        vehicles=vehicles,
        ego_entity=ego_entity,
        npc_entities=npc_entities,
        runner_map_name=runner_map_name,
        target_suite_path=target_suite_path,
        target_project_path=project_path,
        target_xosc_path=target_xosc_path,
        in_place=in_place,
        source_project_map=source_map,
        source_scenario_config=source_config,
        warnings=warnings,
        blocking_errors=blocking_errors,
    )


def resolve_target_paths(
    xosc_path: Path,
    output_root: Path,
    suite_path: Path | None,
    in_place: bool,
    warnings: list[str],
    blocking_errors: list[str],
) -> tuple[Path, Path, Path]:
    if in_place:
        if suite_path is None:
            blocking_errors.append("--in-place requires --suite.")
            suite_path = output_root / "PDM_MORAI_ACTIVE" / "PDM_MORAI_ACTIVE.msuite"
            project_path = suite_path.parent / GENERATED_PROJECT_NAME / f"{GENERATED_PROJECT_NAME}.mproj"
            return suite_path, project_path, project_path.parent / "current.xosc"
        suite_path = suite_path.resolve(strict=False)
        if not suite_path.exists():
            blocking_errors.append(f"Target suite does not exist for --in-place: {suite_path}")
            project_path = suite_path.parent / GENERATED_PROJECT_NAME / f"{GENERATED_PROJECT_NAME}.mproj"
            return suite_path, project_path, project_path.parent / "current.xosc"
        try:
            suite = read_json(suite_path)
        except PrepareError as exc:
            blocking_errors.append(str(exc))
            project_path = suite_path.parent / GENERATED_PROJECT_NAME / f"{GENERATED_PROJECT_NAME}.mproj"
            return suite_path, project_path, project_path.parent / "current.xosc"
        project_list = suite.get("projectList") or []
        if not project_list or not project_list[0].get("filepath"):
            blocking_errors.append(f"Target suite has no projectList filepath: {suite_path}")
            project_path = suite_path.parent / GENERATED_PROJECT_NAME / f"{GENERATED_PROJECT_NAME}.mproj"
            return suite_path, project_path, project_path.parent / "current.xosc"
        project_path = (suite_path.parent / str(project_list[0]["filepath"])).resolve(strict=False)
        if not project_path.exists():
            warnings.append(f"Project file will be created: {project_path}")
        return suite_path, project_path, project_path.parent / "current.xosc"

    if suite_path is not None:
        blocking_errors.append("--suite is only supported with --in-place.")
    stem = safe_name(xosc_path.stem)
    suite_dir = output_root / stem
    source_project = source_project_path(xosc_path)
    project_name = source_project.stem if source_project else GENERATED_PROJECT_NAME
    project_dir = suite_dir / project_name
    target_suite_path = suite_dir / f"{stem}.msuite"
    project_path = project_dir / f"{project_name}.mproj"
    target_xosc_path = project_dir / xosc_path.name
    return target_suite_path, project_path, target_xosc_path


def detect_ego(entity_ids: list[str], override: str | None) -> str | None:
    if override:
        return override
    if "Ego" in entity_ids:
        return "Ego"
    for entity_id in entity_ids:
        if entity_id.lower() == "ego":
            return entity_id
    return None


def parse_xyz(
    element: ET.Element | None,
    names: tuple[str, str, str],
    parameters: dict[str, str],
) -> tuple[float, float, float] | None:
    if element is None:
        return None
    values = [as_float(element.get(name), parameters) for name in names]
    if any(value is None for value in values):
        return None
    return (float(values[0]), float(values[1]), float(values[2]))


def parse_initial_state(
    root: ET.Element,
    parameters: dict[str, str],
    mgeo_dir: Path | None = None,
) -> tuple[dict[str, Pose], dict[str, float]]:
    poses: dict[str, Pose] = {}
    speeds: dict[str, float] = {}
    links: dict[str, dict[str, Any]] | None = None
    for private in root.iter():
        if local_name(private) != "Private":
            continue
        entity = private.get("entityRef")
        if not entity:
            continue
        if mgeo_dir is not None:
            for lane_position in private.iter():
                if local_name(lane_position) != "LanePosition":
                    continue
                if links is None:
                    with (mgeo_dir / "link_set.json").open("r", encoding="utf-8-sig") as file:
                        links = {str(item.get("idx")): item for item in json.load(file)}
                pose = pose_from_lane_position(lane_position, parameters, links)
                if pose is not None:
                    poses[entity] = pose
                    break
        if entity not in poses:
            for world_position in private.iter():
                if local_name(world_position) != "WorldPosition":
                    continue
                x = as_float(world_position.get("x"), parameters)
                y = as_float(world_position.get("y"), parameters)
                z = as_float(world_position.get("z"), parameters)
                h = as_float(world_position.get("h"), parameters)
                if x is not None and y is not None and z is not None and h is not None:
                    poses[entity] = Pose(x=x, y=y, z=z, h=h)
                    break
        for speed in private.iter():
            if local_name(speed) != "AbsoluteTargetSpeed":
                continue
            value = as_float(speed.get("value"), parameters)
            if value is not None:
                speeds[entity] = value
                break
    return poses, speeds


def pose_from_lane_position(
    lane_position: ET.Element,
    parameters: dict[str, str],
    links: dict[str, dict[str, Any]],
) -> Pose | None:
    lane_id = lane_position.get("laneId")
    s = as_float(lane_position.get("s"), parameters)
    offset = as_float(lane_position.get("offset"), parameters) or 0.0
    link = links.get(str(lane_id or ""))
    points = link.get("points") if link else None
    if s is None or not isinstance(points, list) or len(points) < 2:
        return None

    remaining = max(0.0, s)
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
    segment_xy_length = math.hypot(dx, dy)
    tangent_heading = math.atan2(dy, dx)
    orientation = find_first(lane_position, "Orientation")
    heading = (
        as_float(orientation.get("h"), parameters)
        if orientation is not None
        else None
    )
    x = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
    y = float(start[1]) + (float(end[1]) - float(start[1])) * ratio
    z = float(start[2]) + (float(end[2]) - float(start[2])) * ratio
    if segment_xy_length > 0.0:
        x += -dy / segment_xy_length * offset
        y += dx / segment_xy_length * offset
    return Pose(x=x, y=y, z=z, h=heading if heading is not None else tangent_heading)


def resolve_runner_map(
    logic_file: str | None,
    morai_map: str | None,
    warnings: list[str],
    source_project_map: dict[str, Any] | None = None,
) -> str | None:
    map_root = ROOT_DIR / "data" / "morai" / "mgeo"
    source_map_id = project_map_id(source_project_map)
    if source_map_id is not None:
        local_name = LOCAL_MGEO_BY_SIM_MAP_ID.get(source_map_id)
        if local_name and (map_root / local_name).exists():
            warnings.append(
                f"SIM map id {source_map_id} is mapped to local MGeo '{local_name}'."
            )
            return local_name

    source_map_name = None
    if source_project_map:
        value = source_project_map.get("name")
        source_map_name = str(value) if value else None
    candidates = [value for value in (logic_file, source_map_name, morai_map) if value]
    for candidate in candidates:
        if (map_root / candidate).exists():
            return candidate
        alias = LOCAL_MGEO_BY_MAP_NAME.get(candidate)
        if alias and (map_root / alias).exists():
            if alias != candidate:
                warnings.append(f"Map '{candidate}' is normalized to local MGeo '{alias}'.")
            return alias
    return None


def sim_map_name(info: XoscInfo) -> str | None:
    if info.source_project_map:
        source_id = project_map_id(info.source_project_map)
        canonical_name = SIM_MAP_NAME_BY_ID.get(source_id)
        if canonical_name:
            return canonical_name
        source_name = info.source_project_map.get("name")
        if source_name:
            return str(source_name)
    return info.morai_map or info.runner_map_name


def sim_map_id(info_or_name: XoscInfo | str | None) -> int:
    if not isinstance(info_or_name, str) and info_or_name is not None:
        source_id = project_map_id(info_or_name.source_project_map)
        if source_id is not None:
            return source_id
        map_name = sim_map_name(info_or_name)
    else:
        map_name = info_or_name
    return SIM_MAP_ID_BY_NAME.get(str(map_name or ""), 0)


def project_logic_file(info: XoscInfo) -> str | None:
    if info.logic_file:
        return info.logic_file
    if info.runner_map_name:
        return "MGeo"
    return None


def output_logic_file(info: XoscInfo, project_dir: Path, mgeo_mode: str) -> str | None:
    if mgeo_mode != "reference":
        return sidecar_logic_file(info, project_dir)
    source = resolve_mgeo_source(info)
    if source is None:
        return project_logic_file(info)
    return Path(os.path.relpath(source.resolve(strict=False), project_dir.resolve(strict=False))).as_posix()


def sidecar_logic_file(info: XoscInfo, project_dir: Path) -> str | None:
    logic_file = project_logic_file(info)
    if not logic_file:
        return None
    target = (project_dir / logic_file).resolve(strict=False)
    if target.is_relative_to(project_dir.resolve(strict=False)):
        return logic_file
    return "MGeo"


def project_logic_path(info: XoscInfo, project_dir: Path) -> Path | None:
    logic_file = sidecar_logic_file(info, project_dir)
    if not logic_file:
        return None
    parts = [part for part in logic_file.replace("\\", "/").split("/") if part]
    return project_dir.joinpath(*parts) if parts else None


def mgeo_source_candidates(info: XoscInfo) -> list[Path]:
    candidates: list[Path] = []
    logic_file = project_logic_file(info)
    if logic_file:
        logic = Path(*[part for part in logic_file.replace("\\", "/").split("/") if part])
        candidates.extend(
            [
                ROOT_DIR / "data" / "morai" / "mgeo" / logic,
                ROOT_DIR / "data" / "morai" / "mgeo" / logic.parent,
                info.path.parent / logic,
                info.path.parent / logic.parent,
            ]
        )
        home = Path.home()
        if logic_file == "R_KR_PR_Hwaseong-si/MGeo":
            candidates.extend(
                [
                    home
                    / "Downloads"
                    / "RUN_SCENARIO_V4-1"
                    / "suites"
                    / "DCAS_Sample"
                    / "DCAS_Sample"
                    / "R_KR_PR_Hwaseong-si"
                    / "MGeo",
                    home
                    / "Downloads"
                    / "RUN_SCENARIO_V4-1"
                    / "suites"
                    / "dynamics"
                    / "4.2.5.1.1. Positioning in the Lane of Travel Project"
                    / "R_KR_PR_Hwaseong-si"
                    / "MGeo",
                ]
            )
    if info.runner_map_name:
        candidates.append(ROOT_DIR / "data" / "morai" / "mgeo" / info.runner_map_name)
    return candidates


def resolve_mgeo_source(info: XoscInfo) -> Path | None:
    for candidate in mgeo_source_candidates(info):
        if candidate.exists() and (candidate / "link_set.json").exists():
            return candidate
        nested = candidate / "MGeo"
        if nested.exists() and (nested / "link_set.json").exists():
            return nested
    return None


def build_entity_entry(entity_id: str) -> dict[str, str]:
    return {"id": entity_id, "name": entity_id, "descriptorName": ""}


def build_kinetics(entity_id: str, dynamic_type: str) -> dict[str, Any]:
    return {
        "modelName": "None",
        "type": "Standard",
        "optionId": 0,
        "parentName": entity_id,
        "dynamicType": dynamic_type,
        "body": {
            "mass": 0,
            "dragCoeff": 0,
            "ineritaTensorScale": 0,
            "centerOfMass": {"bEnable": False, "position": {"x": 0, "y": 0, "z": 0}},
            "iD": 0,
        },
        "steer": {"maxAngle": 0},
        "brake": {"maxTorque": 0},
    }


def build_dynamics_custom(entity_id: str, vehicle: VehicleInfo) -> dict[str, Any]:
    mass = 2100
    return {
        "vehicleSetup": {
            "mass": mass,
            "centerOfMassOverride": {"x": 150, "y": 0, "z": 50},
            "dragCoefficient": 0.30000001192092896,
            "inertiaTensorScale": {"x": 1.47, "y": 2.57, "z": 2.3},
            "maxBrakeTorque": 3000,
        },
        "axle1": {
            "maxSteerAngle": 40,
            "wheelPosition": 299.9999084472588,
            "trackWidth": 162.54296875,
            "wheelDiameter": 74,
            "wheelMass": 20,
        },
        "axle2": {
            "maxSteerAngle": 0,
            "wheelPosition": 0.11405086516659321,
            "trackWidth": 163.97041320800781,
            "wheelDiameter": 74,
            "wheelMass": 20,
        },
        "objectName": entity_id,
    }


def build_project(info: XoscInfo) -> dict[str, Any]:
    assert info.ego_entity is not None
    entity_ids = [info.ego_entity, *info.npc_entities]
    map_name = sim_map_name(info)
    project_map = (
        copy.deepcopy(info.source_project_map)
        if info.source_project_map
        else {
            "type": "Standard",
            "name": map_name,
            "id": sim_map_id(info),
            "originFrame": {"position": {"x": 0, "y": 0, "z": 0}},
        }
    )
    if map_name:
        project_map["name"] = map_name
    scenario_config = copy.deepcopy(info.source_scenario_config)
    if scenario_config is None:
        scenario_config = {}
    scenario_config.setdefault("sensorList", [])
    scenario_config.setdefault(
        "kineticsModelList",
        [
            build_kinetics(entity_id, "Dynamics" if entity_id == info.ego_entity else "Kinematic")
            for entity_id in entity_ids
        ],
    )
    scenario_config.setdefault(
        "DynamicsCustomData",
        [
            build_dynamics_custom(entity_id, info.vehicles[entity_id])
            for entity_id in entity_ids
        ],
    )
    scenario_config.setdefault(
        "vehicleAgentTypeList",
        [
            {
                "ObjectName": entity_id,
                "AgentType": 1 if entity_id == info.ego_entity else 0,
            }
            for entity_id in entity_ids
        ],
    )
    project = {
        "map": project_map,
        "scenarioList": [
            {
                "name": info.path.stem,
                "path": info.target_xosc_path.name,
                "config": scenario_config,
            }
        ],
    }
    project["interfaceList"] = [
        {"name": "Manual Control", "path": "Manual Control.json"},
        {"name": "Vehicle Info Ego", "path": "Vehicle Info Ego.json"},
    ]
    return project


def build_msuite(info: XoscInfo) -> dict[str, Any]:
    project_name = info.target_project_path.stem
    project_filepath = info.target_project_path.relative_to(info.target_suite_path.parent).as_posix()
    return {
        "name": info.path.stem,
        "versionFormat": [1, 0],
        "versionSimulator": [],
        "projectList": [
            {
                "name": project_name,
                "filepath": project_filepath,
            }
        ],
    }


def build_vehicle_info(info: XoscInfo, rx_port: int, feedback_hz: int) -> dict[str, Any]:
    assert info.ego_entity is not None
    entity_ids = [info.ego_entity, *info.npc_entities]
    return {
        "version": "1.0.0-alpha",
        "createDate": "2026-05-26 00:00",
        "interface": {
            "activate": True,
            "isExternal": False,
            "recordEnabled": False,
            "outputFormat": "CSV",
            "name": "Vehicle Info Ego",
            "projectName": info.target_project_path.stem,
            "messageTemplateFileName": "Vehicle Info",
            "networkType": "UDP",
            "hertz": feedback_hz,
            "direction": "SEND",
            "hostAddr": {"ip": "127.0.0.1", "port": 0},
            "destinationAddr": {"ip": "127.0.0.1", "port": rx_port},
            "entityList": [build_entity_entry(entity_id) for entity_id in entity_ids],
            "recordRelationDataModelList": [],
            "rosInfo": {"topicName": "", "messageType": ""},
            "canInfo": {"id": "", "name": ""},
        },
    }


def build_manual_control(
    info: XoscInfo,
    tx_port: int,
    control_hz: int,
    manual_control_entity: str | None,
) -> dict[str, Any]:
    assert info.ego_entity is not None
    target_entity = manual_control_entity or info.ego_entity
    return {
        "version": "1.0.0-alpha",
        "createDate": "2026-05-26 00:00",
        "interface": {
            "activate": True,
            "isExternal": False,
            "recordEnabled": False,
            "outputFormat": "CSV",
            "name": "Manual Control",
            "projectName": info.target_project_path.stem,
            "messageTemplateFileName": "Vehicle Manual Control",
            "networkType": "UDP",
            "hertz": control_hz,
            "direction": "RECEIVE",
            "hostAddr": {"ip": "127.0.0.1", "port": tx_port},
            "destinationAddr": {"ip": "", "port": 0},
            "entityList": [build_entity_entry(target_entity)],
            "recordRelationDataModelList": [],
            "rosInfo": {"topicName": "", "messageType": ""},
            "canInfo": {"id": "", "name": ""},
        },
    }


def write_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


def write_prepared_xosc(info: XoscInfo, headless_control: bool, mgeo_mode: str) -> None:
    assert info.ego_entity is not None
    tree = ET.parse(info.path)
    root = tree.getroot()
    map_name = sim_map_name(info)
    for element in root.iter():
        if local_name(element) == "VariableDeclaration" and element.get("name") == "moraiMap":
            if map_name:
                element.set("value", map_name)
        if local_name(element) == "LogicFile":
            logic_file = output_logic_file(info, info.target_project_path.parent, mgeo_mode)
            if logic_file:
                element.set("filepath", logic_file)
    for scenario_object in root.iter():
        if local_name(scenario_object) != "ScenarioObject":
            continue
        entity_id = scenario_object.get("name")
        vehicle = None
        for child in scenario_object:
            if local_name(child) == "Vehicle":
                vehicle = child
                break
        if vehicle is None or entity_id is None:
            continue
        if entity_id != info.ego_entity:
            continue
        normalize_vehicle_model_name(vehicle, entity_id)
        sanitize_vehicle_geometry(vehicle, info.parameters)
        vehicle.set("role", "none")
        upsert_property(vehicle, "check_tl", "false")
        upsert_property(vehicle, "apply_acc", "true")
        upsert_property(vehicle, "MotionModel", "Dynamics")
        if headless_control:
            vehicle.set("name", entity_id)
            ensure_object_controller(scenario_object)
            remove_property(vehicle, "external_control")
            remove_property(vehicle, "AgentType")
        else:
            upsert_property(vehicle, "external_control", "true")
            upsert_property(vehicle, "AgentType", "Ego")
    force_initial_speed(root, info.ego_entity, 0.0)
    if headless_control:
        move_initial_speed_before_teleport(root, info.ego_entity)
    sanitize_sun_angles(root, info.parameters)
    scrub_xml_text_nodes(root)
    ET.indent(tree, space="  ")
    tree.write(info.target_xosc_path, encoding="utf-8", xml_declaration=True)


def build_vehicle_info_template() -> dict[str, Any]:
    fields: list[dict[str, Any]] = []
    for name, variable_name, variable_type in (
        ("seconds", "time_stamp.seconds", "INT64"),
        ("nanos", "time_stamp.nanos", "INT32"),
        ("id", "id", "STRING"),
        ("x", "ground_vehicle_attributes.chassis_attributes.transform.location.x", "FLOAT"),
        ("y", "ground_vehicle_attributes.chassis_attributes.transform.location.y", "FLOAT"),
        ("z", "ground_vehicle_attributes.chassis_attributes.transform.location.z", "FLOAT"),
        ("x", "ground_vehicle_attributes.chassis_attributes.transform.rotation.x", "FLOAT"),
        ("y", "ground_vehicle_attributes.chassis_attributes.transform.rotation.y", "FLOAT"),
        ("z", "ground_vehicle_attributes.chassis_attributes.transform.rotation.z", "FLOAT"),
        ("x", "ground_vehicle_attributes.chassis_attributes.local_velocity.x", "FLOAT"),
        ("y", "ground_vehicle_attributes.chassis_attributes.local_velocity.y", "FLOAT"),
        ("z", "ground_vehicle_attributes.chassis_attributes.local_velocity.z", "FLOAT"),
        ("x", "ground_vehicle_attributes.chassis_attributes.local_acceleration.x", "FLOAT"),
        ("y", "ground_vehicle_attributes.chassis_attributes.local_acceleration.y", "FLOAT"),
        ("z", "ground_vehicle_attributes.chassis_attributes.local_acceleration.z", "FLOAT"),
        ("x", "ground_vehicle_attributes.chassis_attributes.angular_velocity.x", "FLOAT"),
        ("y", "ground_vehicle_attributes.chassis_attributes.angular_velocity.y", "FLOAT"),
        ("z", "ground_vehicle_attributes.chassis_attributes.angular_velocity.z", "FLOAT"),
        ("throttle", "ground_vehicle_attributes.vehicle_control_attributes.throttle", "FLOAT"),
        ("brake", "ground_vehicle_attributes.vehicle_control_attributes.brake", "FLOAT"),
        ("steer_angle", "ground_vehicle_attributes.vehicle_control_attributes.steer_angle", "FLOAT"),
    ):
        field: dict[str, Any] = {
            "name": name,
            "relationName": "GroundVehicleEntity",
            "variableName": variable_name,
            "variableType": variable_type,
        }
        if variable_name == "id":
            field["length"] = 24
        fields.append(field)
    return {
        "version": "1.0.0-alpha",
        "messageTemplate": {
            "name": "Vehicle Info",
            "isControl": False,
            "isSensor": False,
            "description": "",
            "segmentList": [{"type": "FIELDS", "relationName": "GroundVehicleEntity", "fieldList": fields}],
        },
    }


def build_vehicle_manual_control_template() -> dict[str, Any]:
    fields = [
        ("throttle", "throttle"),
        ("brake", "brake"),
        ("steering wheel angle", "steering_wheel_angle"),
    ]
    return {
        "version": "1.0.0-alpha",
        "messageTemplate": {
            "name": "Vehicle Manual Control",
            "MessageType": "Control",
            "isControl": True,
            "isSensor": False,
            "description": "",
            "segmentList": [
                {
                    "type": "FIELDS",
                    "relationName": "ManualControl",
                    "fieldList": [
                        {
                            "name": name,
                            "relationName": "ManualControl",
                            "variableName": variable_name,
                            "variableType": "DOUBLE",
                        }
                        for name, variable_name in fields
                    ],
                }
            ],
        },
    }


def write_templates(project_dir: Path) -> None:
    write_json(project_dir / "Vehicle Info.tmpl", build_vehicle_info_template())
    write_json(project_dir / "Vehicle Manual Control.tmpl", build_vehicle_manual_control_template())


def copy_mgeo_sidecar(info: XoscInfo, project_dir: Path, mgeo_mode: str) -> None:
    source = resolve_mgeo_source(info)
    if source is None:
        return
    target = project_logic_path(info, project_dir)
    if target is None:
        return
    stale_target = project_dir / info.runner_map_name
    if not info.in_place and stale_target.exists() and stale_target not in (target, target.parent):
        remove_project_tree(stale_target, project_dir)
    if mgeo_mode == "reference":
        if target.exists():
            remove_project_tree(target, project_dir)
        return
    if target.exists():
        remove_project_tree(target, project_dir)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, target)


def remove_project_tree(path: Path, project_dir: Path) -> None:
    resolved_path = path.resolve(strict=False)
    resolved_project_dir = project_dir.resolve(strict=False)
    if resolved_path == resolved_project_dir or not resolved_path.is_relative_to(resolved_project_dir):
        raise PrepareError(f"Refusing to remove path outside generated project: {resolved_path}")
    shutil.rmtree(resolved_path)


def generate_suite(
    info: XoscInfo,
    rx_port: int,
    tx_port: int,
    feedback_hz: int,
    control_hz: int,
    headless_control: bool,
    manual_control_entity: str | None,
    mgeo_mode: str,
) -> None:
    if info.blocking_errors:
        raise PrepareError("; ".join(info.blocking_errors))
    project_dir = info.target_project_path.parent
    project_dir.mkdir(parents=True, exist_ok=True)
    if not info.in_place:
        write_json(info.target_suite_path, build_msuite(info))
    write_json(info.target_project_path, build_project(info))
    write_json(project_dir / "Vehicle Info Ego.json", build_vehicle_info(info, rx_port, feedback_hz))
    write_json(
        project_dir / "Manual Control.json",
        build_manual_control(info, tx_port, control_hz, manual_control_entity),
    )
    write_prepared_xosc(info, headless_control, mgeo_mode)
    write_templates(project_dir)
    copy_mgeo_sidecar(info, project_dir, mgeo_mode)


def print_report(
    info: XoscInfo,
    rx_port: int,
    tx_port: int,
    feedback_hz: int,
    control_hz: int,
    dry_run: bool,
    manual_control_entity: str | None,
) -> None:
    action = "in-place" if info.in_place else "generate"
    print(f"mode: {action} {'dry-run' if dry_run else 'write'}")
    print(f"input_xosc: {info.path}")
    print(f"ego_entity: {info.ego_entity or '-'}")
    print(f"npc_entities: {', '.join(info.npc_entities) if info.npc_entities else '-'}")
    print(f"road_network_logic_file: {info.logic_file or '-'}")
    print(f"moraiMap: {info.morai_map or '-'}")
    print(f"runner_map_name: {info.runner_map_name or '-'}")
    print("initial_state:")
    for entity_id, vehicle in info.vehicles.items():
        print(
            f"  {entity_id}: speed_mps={format_float(vehicle.initial_speed_mps)}, "
            f"pose=({format_pose(vehicle.initial_pose)}), "
            f"bbox={vehicle.dimensions or '-'}, role={vehicle.role or '-'}"
        )
    print(f"target_msuite: {info.target_suite_path}")
    print(f"target_project: {info.target_project_path}")
    print(f"target_xosc: {info.target_xosc_path}")
    print(f"planned_udp_vehicle_info: 127.0.0.1:{rx_port} @ {feedback_hz}Hz")
    control_target = manual_control_entity or info.ego_entity or "-"
    print(f"planned_manual_control: 127.0.0.1:{tx_port} @ {control_hz}Hz target={control_target}")
    mapping = [info.ego_entity, *info.npc_entities] if info.ego_entity else info.npc_entities
    print(f"planned_logical_id_mapping: {', '.join(f'{item}->{item}' for item in mapping) if mapping else '-'}")
    print(f"warnings: {'; '.join(info.warnings) if info.warnings else '-'}")
    print(f"blocking_errors: {'; '.join(info.blocking_errors) if info.blocking_errors else '-'}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare a runnable PDM-MORAI suite from a variation XOSC.")
    parser.add_argument("--xosc", required=True, help="Input OpenSCENARIO file")
    parser.add_argument("--suite", default=None, help="Existing msuite to update when --in-place is used")
    parser.add_argument("--in-place", action="store_true", help="Update the given msuite shell instead of creating a new suite")
    parser.add_argument("--ego-entity", default=None, help="Explicit Ego ScenarioObject name")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Generated suite root directory")
    parser.add_argument("--rx-port", type=int, default=None, help="UDP Vehicle Info receive port for the runner")
    parser.add_argument("--tx-port", type=int, default=None, help="UDP Manual Control port")
    parser.add_argument("--feedback-hz", type=int, default=None)
    parser.add_argument("--control-hz", type=int, default=None)
    parser.add_argument(
        "--headless-control",
        action="store_true",
        help="Reserved for later headless flow; currently disabled",
    )
    parser.add_argument(
        "--manual-control-entity",
        default=None,
        help="MORAI runtime entity selected by the Manual Control interface; defaults to detected Ego",
    )
    parser.add_argument(
        "--mgeo-mode",
        choices=["reference", "copy"],
        default="reference",
        help="reference keeps generated suites light by pointing LogicFile at data/morai/mgeo; copy embeds MGeo in each suite",
    )
    parser.add_argument("--dry-run", action="store_true", help="Parse and report without writing files")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.headless_control:
        print("ERROR: headless-control is deferred. Use the current UI-only flow.")
        return 2
    try:
        suite_path = Path(args.suite).resolve(strict=False) if args.suite else None
        rx_port = args.rx_port if args.rx_port is not None else 9097
        tx_port = args.tx_port if args.tx_port is not None else 9091
        feedback_hz = args.feedback_hz if args.feedback_hz is not None else 30
        control_hz = args.control_hz if args.control_hz is not None else 60
        info = parse_xosc(
            Path(args.xosc).resolve(strict=False),
            args.ego_entity,
            Path(args.output_root).resolve(strict=False),
            suite_path=suite_path,
            in_place=args.in_place,
        )
        print_report(
            info,
            rx_port,
            tx_port,
            feedback_hz,
            control_hz,
            args.dry_run,
            args.manual_control_entity,
        )
        if info.blocking_errors:
            return 2
        if not args.dry_run:
            generate_suite(
                info,
                rx_port,
                tx_port,
                feedback_hz,
                control_hz,
                args.headless_control,
                args.manual_control_entity,
                args.mgeo_mode,
            )
            print(f"{'updated_msuite' if info.in_place else 'generated_msuite'}: {info.target_suite_path}")
    except (OSError, ET.ParseError, ValueError, PrepareError) as exc:
        print(f"ERROR: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
