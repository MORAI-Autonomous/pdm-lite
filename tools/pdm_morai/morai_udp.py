# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import math
import socket
import struct
import time
from dataclasses import dataclass
from typing import Any

from pdm_lite.team_code.ego_manual_control import EgoManualControl


CTRL_CMD_FMT = "<ddd"
EGO_INFO_FMT = "<qi24s" + ("f" * 18)
CHUNK_HEADER_FMT = "<IHH"
CTRL_CMD_PACKET_SIZE = struct.calcsize(CTRL_CMD_FMT)
EGO_INFO_PACKET_SIZE = struct.calcsize(EGO_INFO_FMT)
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FMT)


@dataclass(frozen=True)
class MoraiVehicleState:
    id: str
    raw_id: str
    record_index: int
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    velocity_x: float
    velocity_y: float
    velocity_z: float
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    throttle: float
    brake: float
    steer: float
    stamp_seconds: int
    stamp_nanos: int

    @property
    def heading_rad(self) -> float:
        return math.radians(self.yaw_deg)

    @property
    def speed(self) -> float:
        # The generated Vehicle Info template uses chassis local_velocity, which
        # is m/s in the active PDMLite suite. Keep this as the scalar speed used
        # by the PDM Lite controllers.
        return max(0.0, math.sqrt(
            self.velocity_x * self.velocity_x
            + self.velocity_y * self.velocity_y
            + self.velocity_z * self.velocity_z
        ))


def pack_control(control: EgoManualControl) -> bytes:
    return struct.pack(CTRL_CMD_FMT, *control.as_morai_udp_tuple())


def parse_ego_info_payload(payload: bytes) -> list[dict[str, Any]]:
    if len(payload) % EGO_INFO_PACKET_SIZE != 0:
        raise ValueError(
            f"unexpected ego info payload size {len(payload)}; record size {EGO_INFO_PACKET_SIZE}"
        )

    states: list[dict[str, Any]] = []
    for index, offset in enumerate(range(0, len(payload), EGO_INFO_PACKET_SIZE)):
        unpacked = struct.unpack(EGO_INFO_FMT, payload[offset : offset + EGO_INFO_PACKET_SIZE])
        raw_id = unpacked[2].decode("utf-8", errors="ignore").rstrip("\x00")
        states.append(
            {
                "record_index": index,
                "seconds": unpacked[0],
                "nanos": unpacked[1],
                "id": raw_id,
                "raw_id": raw_id,
                "x_loc": unpacked[3],
                "y_loc": unpacked[4],
                "z_loc": unpacked[5],
                "x_rot": unpacked[6],
                "y_rot": unpacked[7],
                "z_rot": unpacked[8],
                "x_vel": unpacked[9],
                "y_vel": unpacked[10],
                "z_vel": unpacked[11],
                "x_acc": unpacked[12],
                "y_acc": unpacked[13],
                "z_acc": unpacked[14],
                "x_ang_vel": unpacked[15],
                "y_ang_vel": unpacked[16],
                "z_ang_vel": unpacked[17],
                "throttle": unpacked[18],
                "brake": unpacked[19],
                "steer_angle": unpacked[20],
            }
        )
    return states


def to_vehicle_state(state: dict[str, Any]) -> MoraiVehicleState:
    return MoraiVehicleState(
        id=str(state["id"]),
        raw_id=str(state.get("raw_id") or state["id"]),
        record_index=int(state["record_index"]),
        x=float(state["x_loc"]),
        y=float(state["y_loc"]),
        z=float(state["z_loc"]),
        roll_deg=float(state["x_rot"]),
        pitch_deg=float(state["y_rot"]),
        yaw_deg=float(state["z_rot"]),
        velocity_x=float(state["x_vel"]),
        velocity_y=float(state["y_vel"]),
        velocity_z=float(state["z_vel"]),
        acceleration_x=float(state["x_acc"]),
        acceleration_y=float(state["y_acc"]),
        acceleration_z=float(state["z_acc"]),
        angular_velocity_x=float(state["x_ang_vel"]),
        angular_velocity_y=float(state["y_ang_vel"]),
        angular_velocity_z=float(state["z_ang_vel"]),
        throttle=float(state["throttle"]),
        brake=float(state["brake"]),
        steer=float(state["steer_angle"]),
        stamp_seconds=int(state["seconds"]),
        stamp_nanos=int(state["nanos"]),
    )


class MoraiUdpClient:
    def __init__(
        self,
        ego_info_ip: str,
        ego_info_port: int,
        ctrl_ip: str,
        ctrl_port: int,
        entity_ids: list[str],
        ego_id: str,
    ):
        self.ego_info_ip = ego_info_ip
        self.ego_info_port = ego_info_port
        self.ctrl_ip = ctrl_ip
        self.ctrl_port = ctrl_port
        self.entity_ids = entity_ids
        self.ego_id = ego_id
        self._receiver: socket.socket | None = None
        self._sender: socket.socket | None = None
        self._chunk_buffers: dict[int, dict[str, Any]] = {}
        self.states: dict[str, MoraiVehicleState] = {}
        self.rx_records = 0
        self.rx_chunks = 0
        self.invalid_packets = 0
        self.tx_packets = 0
        self.rx_datagrams = 0
        self.packet_sizes: list[int] = []
        self.last_error: str | None = None
        self.last_receive_time: float | None = None
        self.inferred_entity_ids = False

    def open(self) -> None:
        self.close()
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            receiver.bind((self.ego_info_ip, self.ego_info_port))
        except OSError as exc:
            receiver.close()
            raise RuntimeError(
                f"RX bind failed on {self.ego_info_ip}:{self.ego_info_port}. "
                "Close any existing UDP GUI/control process that is already "
                "listening on the same Vehicle Info port."
            ) from exc
        receiver.setblocking(False)
        self._receiver = receiver
        self._sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self) -> None:
        for sock in (self._receiver, self._sender):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._receiver = None
        self._sender = None

    def poll(self) -> int:
        if self._receiver is None:
            raise RuntimeError("UDP receiver is not open.")

        updated = 0
        while True:
            try:
                raw_data, _addr = self._receiver.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError as exc:
                self.invalid_packets += 1
                self.last_error = str(exc)
                break

            self.rx_datagrams += 1
            self.packet_sizes.append(len(raw_data))
            if len(self.packet_sizes) > 16:
                del self.packet_sizes[:-16]

            payload = self._payload_from_datagram(raw_data)
            if payload is None:
                continue

            try:
                parsed_states = parse_ego_info_payload(payload)
            except ValueError as exc:
                self.invalid_packets += 1
                self.last_error = str(exc)
                continue

            self._apply_logical_ids(parsed_states)
            next_states: dict[str, MoraiVehicleState] = {}
            for parsed in parsed_states:
                vehicle_state = to_vehicle_state(parsed)
                next_states[vehicle_state.id] = vehicle_state
            # Vehicle Info is a complete snapshot. Replacing the mapping prevents
            # actors from the previous scenario from remaining selectable after
            # a suite transition changes the vehicle count or logical IDs.
            self.states = next_states
            self.rx_records += len(parsed_states)
            self.last_receive_time = time.monotonic()
            self.last_error = None
            updated += len(parsed_states)

        self._drop_stale_chunks()
        return updated

    def ego_state(self) -> MoraiVehicleState | None:
        return self.states.get(self.ego_id)

    def send_control(self, control: EgoManualControl) -> None:
        if self._sender is None:
            raise RuntimeError("UDP sender is not open.")
        self._sender.sendto(pack_control(control), (self.ctrl_ip, self.ctrl_port))
        self.tx_packets += 1

    def _payload_from_datagram(self, raw_data: bytes) -> bytes | None:
        if not self._looks_like_chunk(raw_data):
            return raw_data

        frame, chunk_index, chunk_total = struct.unpack(
            CHUNK_HEADER_FMT, raw_data[:CHUNK_HEADER_SIZE]
        )
        self.rx_chunks += 1
        buffer = self._chunk_buffers.setdefault(
            frame, {"total": chunk_total, "parts": {}, "updated": time.monotonic()}
        )
        buffer["total"] = chunk_total
        buffer["updated"] = time.monotonic()
        buffer["parts"][chunk_index] = raw_data[CHUNK_HEADER_SIZE:]

        parts = buffer["parts"]
        if len(parts) != chunk_total:
            return None

        payload = b"".join(parts[index] for index in range(chunk_total))
        del self._chunk_buffers[frame]
        return payload

    def _looks_like_chunk(self, raw_data: bytes) -> bool:
        if len(raw_data) < CHUNK_HEADER_SIZE:
            return False
        if len(raw_data) % EGO_INFO_PACKET_SIZE == 0:
            return False
        _frame, chunk_index, chunk_total = struct.unpack(
            CHUNK_HEADER_FMT, raw_data[:CHUNK_HEADER_SIZE]
        )
        return 1 <= chunk_total <= 32 and chunk_index < chunk_total

    def _apply_logical_ids(self, parsed_states: list[dict[str, Any]]) -> None:
        raw_ids = [str(state.get("id") or "") for state in parsed_states]
        duplicate_ids = len(set(raw_ids)) < len(raw_ids)
        missing_ego = self.ego_id not in raw_ids
        if not duplicate_ids and not missing_ego:
            return

        if not self.entity_ids:
            if len(parsed_states) <= 1 or len(set(raw_ids)) != 1:
                return
            inferred_ids = [
                self.ego_id,
                *(f"NPC_{index}" for index in range(1, len(parsed_states))),
            ]
            self.inferred_entity_ids = True
        else:
            if len(parsed_states) > len(self.entity_ids):
                return
            inferred_ids = self.entity_ids

        for index, state in enumerate(parsed_states):
            state["raw_id"] = raw_ids[index]
            state["id"] = inferred_ids[index]

    def _drop_stale_chunks(self) -> None:
        now = time.monotonic()
        stale = [
            frame
            for frame, buffer in self._chunk_buffers.items()
            if now - float(buffer["updated"]) > 2.0
        ]
        for frame in stale:
            del self._chunk_buffers[frame]
