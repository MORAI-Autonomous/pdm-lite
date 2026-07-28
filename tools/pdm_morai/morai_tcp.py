# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pdm_lite.team_code.ego_manual_control import EgoManualControl


MAGIC = 0x4D
MSG_CLASS_REQUEST = 0x01
MSG_CLASS_RESPONSE = 0x02
HEADER_FMT = "<BBIIIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

MSG_GET_SIMULATOR_STATUS: int | None = None  # Msg type/schema not present in this repo.
MSG_GET_SIMULATION_TIME_STATUS = 0x1101
MSG_SET_SIMULATION_TIME_MODE = 0x1102
MSG_FIXED_STEP = 0x1201
MSG_SAVE_DATA = 0x1202
MSG_CREATE_OBJECT = 0x1301
MSG_MANUAL_CONTROL_BY_ID = 0x1302
MSG_TRANSFORM_CONTROL_BY_ID = 0x1303
MSG_SET_TRAJECTORY = 0x1304
MSG_ACTIVE_SUITE_STATUS = 0x1401
MSG_LOAD_SUITE = 0x1402
MSG_SCENARIO_STATUS = 0x1504
MSG_SCENARIO_CONTROL = 0x1505

TIME_MODE_VARIABLE = 1
TIME_MODE_FIXED = 2

SCENARIO_PLAY = 1
SCENARIO_PAUSE = 2
SCENARIO_STOP = 3
SCENARIO_PREV = 4
SCENARIO_NEXT = 5


@dataclass(frozen=True)
class TcpResponse:
    msg_type: int
    request_id: int
    payload: bytes
    flag: int = 0


class MoraiTcpError(RuntimeError):
    pass


def _pack_string(value: str | Path) -> bytes:
    raw = str(value).encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def _read_string(payload: bytes, offset: int) -> tuple[str, int]:
    if offset + 4 > len(payload):
        raise MoraiTcpError("string length is truncated")
    (size,) = struct.unpack_from("<I", payload, offset)
    offset += 4
    end = offset + size
    if end > len(payload):
        raise MoraiTcpError("string payload is truncated")
    return payload[offset:end].decode("utf-8", errors="replace"), end


def parse_result_code(payload: bytes) -> dict[str, int]:
    if len(payload) < 8:
        raise MoraiTcpError(f"result payload too short: {len(payload)}")
    result_code, detail_code = struct.unpack_from("<II", payload, 0)
    return {"result_code": result_code, "detail_code": detail_code}


def parse_simulation_time_status(payload: bytes) -> dict[str, Any]:
    if len(payload) == 8:
        result = parse_result_code(payload)
        result["schema_note"] = "server returned result_code only"
        return result
    offset = 0
    result: dict[str, Any] = {}
    if len(payload) >= 48:
        result_code, detail_code, possible_mode = struct.unpack_from("<III", payload, 0)
        if possible_mode in (TIME_MODE_VARIABLE, TIME_MODE_FIXED):
            result.update({"result_code": result_code, "detail_code": detail_code})
            offset = 8
    if len(payload) - offset < 40:
        raise MoraiTcpError(f"simulation time status payload too short: {len(payload)}")
    mode, target_fps, physics_delta_time, rtf, user_control, step_index, seconds, nanos = struct.unpack_from(
        "<IiiiiQqi", payload, offset
    )
    result.update(
        {
            "mode": mode,
            "target_fps": target_fps,
            "physics_delta_time": physics_delta_time,
            "rtf": rtf,
            "user_control": user_control,
            "step_index": step_index,
            "seconds": seconds,
            "nanos": nanos,
        }
    )
    return result


def parse_set_simulation_time_mode(payload: bytes) -> dict[str, Any]:
    if len(payload) == 8:
        result = parse_result_code(payload)
        result["schema_note"] = "server returned result_code only"
        return result
    if len(payload) < 20:
        raise MoraiTcpError(f"set time mode payload too short: {len(payload)}")
    result_code, detail_code, mode, fixed_delta, simulation_speed = struct.unpack("<IIIff", payload[:20])
    return {
        "result_code": result_code,
        "detail_code": detail_code,
        "mode": mode,
        "fixed_delta": fixed_delta,
        "simulation_speed": simulation_speed,
    }


def parse_create_object(payload: bytes) -> dict[str, Any]:
    result = parse_result_code(payload)
    object_id = ""
    if len(payload) > 8:
        object_id, _offset = _read_string(payload, 8)
    result["object_id"] = object_id
    return result


def parse_active_suite_status(payload: bytes) -> dict[str, Any]:
    result = parse_result_code(payload)
    if len(payload) == 8:
        result.update(
            {
                "active_suite_name": "",
                "active_scenario_name": "",
                "scenario_list": [],
                "schema_note": "server returned result_code only",
            }
        )
        return result
    offset = 8
    active_suite_name, offset = _read_string(payload, offset)
    active_scenario_name, offset = _read_string(payload, offset)
    if offset + 4 > len(payload):
        raise MoraiTcpError("active suite scenario_list_size is truncated")
    (scenario_count,) = struct.unpack_from("<I", payload, offset)
    offset += 4
    scenarios = []
    for _index in range(scenario_count):
        scenario_name, offset = _read_string(payload, offset)
        scenarios.append(scenario_name)
    result.update(
        {
            "active_suite_name": active_suite_name,
            "active_scenario_name": active_scenario_name,
            "scenario_list": scenarios,
        }
    )
    return result


def parse_scenario_status(payload: bytes) -> dict[str, int]:
    if len(payload) == 8:
        result = parse_result_code(payload)
        result["state"] = 0
        return result
    if len(payload) < 12:
        raise MoraiTcpError(f"scenario status payload too short: {len(payload)}")
    result_code, detail_code, state = struct.unpack("<III", payload[:12])
    return {"result_code": result_code, "detail_code": detail_code, "state": state}


class MoraiTcpClient:
    """Low-rate MORAI management TCP client.

    The PDM runtime control loop must stay on UDP. TCP is used here for suite,
    scenario, object, trajectory, and fixed-step management where latency jitter
    is acceptable and request/response correlation matters.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 0, timeout: float = 2.0):
        self.host = host
        self.port = int(port)
        self.timeout = timeout
        self._socket: socket.socket | None = None
        self._request_id = 1
        self.fixed_step_request_count = 0

    def open(self) -> None:
        if self.port <= 0:
            raise MoraiTcpError("--tcp-port must be provided for MORAI TCP API use")
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        self._socket = sock

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None

    def __enter__(self) -> "MoraiTcpClient":
        self.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def request(self, msg_type: int, payload: bytes = b"") -> TcpResponse:
        if self._socket is None:
            raise MoraiTcpError("TCP client is not open")
        request_id = self._request_id
        self._request_id += 1
        header = struct.pack(
            HEADER_FMT,
            MAGIC,
            MSG_CLASS_REQUEST,
            int(msg_type),
            len(payload),
            request_id,
            0,
        )
        self._socket.sendall(header + payload)
        return self._receive_response(request_id)

    def _receive_response(self, request_id: int) -> TcpResponse:
        while True:
            header = self._recv_exact(HEADER_SIZE)
            magic, msg_class, msg_type, payload_size, response_request_id, flag = struct.unpack(
                HEADER_FMT, header
            )
            if magic != MAGIC:
                raise MoraiTcpError(f"invalid TCP magic: 0x{magic:02X}")
            payload = self._recv_exact(payload_size)
            if msg_class != MSG_CLASS_RESPONSE:
                continue
            if response_request_id == request_id:
                return TcpResponse(msg_type=msg_type, request_id=response_request_id, payload=payload, flag=flag)

    def _recv_exact(self, size: int) -> bytes:
        if self._socket is None:
            raise MoraiTcpError("TCP client is not open")
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self._socket.recv(size - len(chunks))
            if not chunk:
                raise MoraiTcpError("TCP connection closed")
            chunks.extend(chunk)
        return bytes(chunks)

    def get_simulator_status(self, msg_type: int | None = MSG_GET_SIMULATOR_STATUS) -> dict[str, Any]:
        if msg_type is None:
            raise MoraiTcpError("GetSimulatorStatus msg_type/schema is not available in this repository")
        response = self.request(msg_type)
        # The schema is not present in this repository. Return raw payload plus
        # result code if the build happens to use a result-code prefix.
        result: dict[str, Any] = {"raw_payload": response.payload}
        if len(response.payload) >= 8:
            result.update(parse_result_code(response.payload[:8]))
        return result

    def get_simulation_time_status(self) -> dict[str, Any]:
        return parse_simulation_time_status(self.request(MSG_GET_SIMULATION_TIME_STATUS).payload)

    def set_simulation_time_mode(
        self,
        mode: int,
        target_fps: int,
        physics_delta_time: int,
        rtf: int,
        user_control: int,
    ) -> dict[str, Any]:
        payload = struct.pack("<iiiii", mode, target_fps, physics_delta_time, rtf, user_control)
        return parse_set_simulation_time_mode(self.request(MSG_SET_SIMULATION_TIME_MODE, payload).payload)

    def fixed_step(self, step_count: int = 1) -> dict[str, int]:
        self.fixed_step_request_count += 1
        payload = struct.pack("<I", int(step_count))
        return parse_result_code(self.request(MSG_FIXED_STEP, payload).payload)

    def save_data(self) -> dict[str, int]:
        return parse_result_code(self.request(MSG_SAVE_DATA).payload)

    def active_suite_status(self) -> dict[str, Any]:
        return parse_active_suite_status(self.request(MSG_ACTIVE_SUITE_STATUS).payload)

    def load_suite(self, suite_path: str | Path) -> dict[str, int]:
        return parse_result_code(self.request(MSG_LOAD_SUITE, _pack_string(suite_path)).payload)

    def scenario_status(self) -> dict[str, int]:
        return parse_scenario_status(self.request(MSG_SCENARIO_STATUS).payload)

    def scenario_control(self, command: int, scenario_name: str = "") -> dict[str, int]:
        payload = struct.pack("<I", int(command)) + _pack_string(scenario_name)
        return parse_result_code(self.request(MSG_SCENARIO_CONTROL, payload).payload)

    def create_object(
        self,
        entity_type: int,
        pos_x: float,
        pos_y: float,
        pos_z: float,
        rot_x: float,
        rot_y: float,
        rot_z: float,
        driving_mode: int,
        ground_vehicle_model: int,
    ) -> dict[str, Any]:
        payload = struct.pack(
            "<iffffffii",
            int(entity_type),
            float(pos_x),
            float(pos_y),
            float(pos_z),
            float(rot_x),
            float(rot_y),
            float(rot_z),
            int(driving_mode),
            int(ground_vehicle_model),
        )
        return parse_create_object(self.request(MSG_CREATE_OBJECT, payload).payload)

    def set_trajectory(
        self,
        entity_id: str,
        follow_mode: int,
        trajectory_name: str,
        points: list[tuple[float, float, float, float]],
    ) -> dict[str, int]:
        payload = bytearray()
        payload += _pack_string(entity_id)
        payload += struct.pack("<i", int(follow_mode))
        payload += _pack_string(trajectory_name)
        payload += struct.pack("<I", len(points))
        for x, y, z, point_time in points:
            payload += struct.pack("<dddd", float(x), float(y), float(z), float(point_time))
        return parse_result_code(self.request(MSG_SET_TRAJECTORY, bytes(payload)).payload)

    def manual_control_by_id(self, entity_id: str, control: EgoManualControl) -> dict[str, int]:
        payload = _pack_string(entity_id) + struct.pack("<ddd", *control.as_morai_udp_tuple())
        return parse_result_code(self.request(MSG_MANUAL_CONTROL_BY_ID, payload).payload)

    def transform_control_by_id(
        self,
        entity_id: str,
        pos_x: float,
        pos_y: float,
        pos_z: float,
        rot_x: float,
        rot_y: float,
        rot_z: float,
        steer_angle: float,
        speed: float,
    ) -> dict[str, int]:
        payload = _pack_string(entity_id) + struct.pack(
            "<fffffffd",
            float(pos_x),
            float(pos_y),
            float(pos_z),
            float(rot_x),
            float(rot_y),
            float(rot_z),
            float(steer_angle),
            float(speed),
        )
        return parse_result_code(self.request(MSG_TRANSFORM_CONTROL_BY_ID, payload).payload)
