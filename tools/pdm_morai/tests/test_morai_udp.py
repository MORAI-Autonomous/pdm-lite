# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import socket
import struct
import unittest
from unittest.mock import Mock

from pdm_morai.morai_udp import (
    EGO_INFO_FMT,
    EGO_INFO_PACKET_SIZE,
    MoraiUdpClient,
    parse_ego_info_payload,
)


class MoraiUdpClientTest(unittest.TestCase):
    def make_client(self, entity_ids: list[str] | None = None) -> MoraiUdpClient:
        return MoraiUdpClient(
            ego_info_ip="127.0.0.1",
            ego_info_port=0,
            ctrl_ip="127.0.0.1",
            ctrl_port=9091,
            entity_ids=entity_ids or [],
            ego_id="Ego",
        )

    def test_duplicate_raw_ids_are_preserved_in_attach_only_mode(self) -> None:
        client = self.make_client()
        states = [{"id": "Car_1"}, {"id": "Car_1"}, {"id": "Car_1"}]

        client._apply_logical_ids(states)

        self.assertEqual([state["id"] for state in states], ["Ego", "NPC_1", "NPC_2"])
        self.assertEqual([state["raw_id"] for state in states], ["Car_1"] * 3)
        self.assertTrue(client.inferred_entity_ids)

    def test_explicit_entity_ids_take_precedence(self) -> None:
        client = self.make_client(["Host", "Lead"])
        states = [{"id": "Car_1"}, {"id": "Car_1"}]

        client._apply_logical_ids(states)

        self.assertEqual([state["id"] for state in states], ["Host", "Lead"])
        self.assertFalse(client.inferred_entity_ids)

    def test_single_car_1_keeps_existing_fallback_behavior(self) -> None:
        client = self.make_client()
        states = [{"id": "Car_1"}]

        client._apply_logical_ids(states)

        self.assertEqual(states, [{"id": "Car_1"}])
        self.assertFalse(client.inferred_entity_ids)

    def test_poll_replaces_previous_scenario_vehicle_snapshot(self) -> None:
        client = self.make_client()
        three_vehicle_payload = b"".join(
            struct.pack(
                EGO_INFO_FMT,
                6,
                900_000_000,
                b"Car_1",
                *([0.0] * 18),
            )
            for _ in range(3)
        )
        single_vehicle_payload = struct.pack(
            EGO_INFO_FMT,
            7,
            100_000_000,
            b"Car_1",
            *([0.0] * 18),
        )
        receiver = Mock()
        client._receiver = receiver

        receiver.recvfrom.side_effect = [
            (three_vehicle_payload, ("127.0.0.1", 50000)),
            BlockingIOError(),
        ]
        client.poll()
        self.assertEqual(set(client.states), {"Ego", "NPC_1", "NPC_2"})

        receiver.recvfrom.side_effect = [
            (single_vehicle_payload, ("127.0.0.1", 50000)),
            BlockingIOError(),
        ]
        client.poll()

        self.assertEqual(set(client.states), {"Car_1"})
        self.assertEqual(client.states["Car_1"].stamp_seconds, 7)
        self.assertNotIn("Ego", client.states)
        self.assertNotIn("NPC_1", client.states)
        self.assertNotIn("NPC_2", client.states)

    def test_vehicle_info_uses_sim_float_wire_format(self) -> None:
        pose = (
            -2729.244209,
            1614.053824,
            28.314075,
            0.515146,
            0.055806,
            -89.781799,
        )
        payload = struct.pack(
            EGO_INFO_FMT,
            0,
            20_000_000,
            b"Ego",
            *pose,
            *([0.0] * 12),
        )

        states = parse_ego_info_payload(payload)

        self.assertEqual(EGO_INFO_PACKET_SIZE, 108)
        actual_pose = tuple(
            states[0][name]
            for name in ("x_loc", "y_loc", "z_loc", "x_rot", "y_rot", "z_rot")
        )
        self.assertEqual(actual_pose, struct.unpack("<ffffff", struct.pack("<ffffff", *pose)))

    def test_second_receiver_cannot_share_the_same_port(self) -> None:
        first = self.make_client()
        first.open()
        try:
            port = first._receiver.getsockname()[1]
            second = MoraiUdpClient(
                ego_info_ip="127.0.0.1",
                ego_info_port=port,
                ctrl_ip="127.0.0.1",
                ctrl_port=9091,
                entity_ids=[],
                ego_id="Ego",
            )
            with self.assertRaises(RuntimeError):
                second.open()
            second.close()
        finally:
            first.close()


if __name__ == "__main__":
    unittest.main()
