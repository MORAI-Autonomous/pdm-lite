# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from tools.pdm_morai_control import (
    DEFAULT_ATTACH_MGEO_DIR,
    build_parser,
    build_runtime_settings,
    configure_fixed_step,
    fixed_time_status_matches,
    is_new_state_sample,
    main,
    parse_runtime_args,
    rebuild_route_from_runtime_pose,
    resolve_runtime_ego,
    resolve_sut_timing,
    send_ego_control,
    select_runtime_mode,
    should_stop_before_start,
    variable_time_status_matches,
)
from pdm_morai.morai_tcp import MoraiTcpError, TIME_MODE_FIXED, TIME_MODE_VARIABLE


class AttachOnlyRuntimeTest(unittest.TestCase):
    def test_parser_defaults_to_safe_gui_variable_udp(self) -> None:
        args = build_parser().parse_args([])

        self.assertIsNone(args.source)
        self.assertEqual(args.execution_mode, "gui")
        self.assertEqual(args.control_mode, "variable-udp")
        self.assertFalse(args.load_suite)
        self.assertFalse(args.scenario_play)
        self.assertFalse(args.scenario_stop)
        self.assertEqual(select_runtime_mode(args), "gui:variable-udp")

    def test_attach_only_uses_runtime_defaults(self) -> None:
        args = build_parser().parse_args([])

        settings = build_runtime_settings(args, scenario=None)

        self.assertEqual(settings.ego_candidates, ("Ego", "Car_1"))
        self.assertEqual(settings.rx_ip, "127.0.0.1")
        self.assertEqual(settings.rx_port, 9097)
        self.assertEqual(settings.tx_ip, "127.0.0.1")
        self.assertEqual(settings.tx_port, 9091)
        self.assertEqual(settings.control_hz, 60)
        self.assertEqual(settings.target_speed, 10.0)
        self.assertEqual(settings.map_dir, DEFAULT_ATTACH_MGEO_DIR.resolve())

    def test_explicit_ego_disables_candidate_fallback(self) -> None:
        args = build_parser().parse_args(["--ego-id", "HostVehicle"])

        settings = build_runtime_settings(args, scenario=None)

        self.assertEqual(settings.ego_candidates, ("HostVehicle",))

    def test_headless_tcp_lockstep_20ms_preset(self) -> None:
        args = parse_runtime_args(["--run-preset", "headless-tcp-lockstep-20ms"])

        self.assertTrue(args.headless)
        self.assertEqual(args.execution_mode, "headless")
        self.assertEqual(args.control_mode, "fixed-tcp-lockstep")
        self.assertTrue(args.fixed_step)
        self.assertEqual(args.fixed_control_transport, "tcp")
        self.assertTrue(args.load_suite)
        self.assertTrue(args.scenario_play)
        self.assertTrue(args.save_data_each_step)
        self.assertEqual(args.tcp_timeout, 30.0)
        self.assertEqual(args.tcp_settle, 2.0)
        self.assertEqual(args.pdm_step_delta_time, 20.0)
        self.assertEqual(args.physics_delta_time, 20)

    def test_headless_tcp_lockstep_10ms_preset(self) -> None:
        args = parse_runtime_args(["--run-preset", "headless-tcp-lockstep-10ms"])

        self.assertEqual(args.pdm_step_delta_time, 10.0)
        self.assertEqual(args.physics_delta_time, 10)

    def test_run_preset_keeps_explicit_step_override(self) -> None:
        args = parse_runtime_args(
            [
                "--run-preset",
                "headless-tcp-lockstep-20ms",
                "--pdm-step-delta-time",
                "10",
                "--physics-delta-time",
                "10",
            ]
        )

        self.assertEqual(args.pdm_step_delta_time, 10.0)
        self.assertEqual(args.physics_delta_time, 10)

    def test_headless_legacy_alias_does_not_select_control_mode(self) -> None:
        args = parse_runtime_args(["--headless"])

        self.assertEqual(args.execution_mode, "headless")
        self.assertEqual(args.control_mode, "variable-udp")
        self.assertFalse(args.fixed_step)
        self.assertFalse(args.load_suite)
        self.assertFalse(args.scenario_play)

    def test_control_modes_select_transport_independently(self) -> None:
        fixed_auto_udp = parse_runtime_args(["--control-mode", "fixed-auto-udp"])
        fixed_udp = parse_runtime_args(["--control-mode", "fixed-udp"])
        lockstep = parse_runtime_args(["--control-mode", "fixed-tcp-lockstep"])

        self.assertFalse(fixed_auto_udp.fixed_step)
        self.assertEqual(fixed_auto_udp.fixed_control_transport, "udp")
        self.assertEqual(fixed_auto_udp.rtf, 1)
        self.assertEqual(fixed_auto_udp.user_control, 0)
        self.assertTrue(fixed_udp.fixed_step)
        self.assertEqual(fixed_udp.fixed_control_transport, "udp")
        self.assertTrue(lockstep.fixed_step)
        self.assertEqual(lockstep.fixed_control_transport, "tcp")

    def test_suite_metadata_keeps_car_1_fallback(self) -> None:
        args = build_parser().parse_args(["--suite", "test.msuite"])
        binding = SimpleNamespace(
            ego_id="Ego",
            entity_ids=["Ego"],
            ego_info_ip="127.0.0.1",
            ego_info_port=9097,
            ctrl_ip="127.0.0.1",
            ctrl_port=9091,
            control_hz=60,
        )
        scenario = SimpleNamespace(
            binding=binding,
            target_speed=10.0,
            map_dir=DEFAULT_ATTACH_MGEO_DIR,
            actor_metadata={},
        )

        settings = build_runtime_settings(args, scenario)

        self.assertEqual(settings.ego_candidates, ("Ego", "Car_1"))

    def test_runtime_ego_falls_back_to_car_1(self) -> None:
        car_1 = SimpleNamespace(id="Car_1")
        udp = SimpleNamespace(states={"Car_1": car_1})

        selected = resolve_runtime_ego(udp, ("Ego", "Car_1"))

        self.assertIs(selected, car_1)

    def test_state_sample_is_processed_once_per_timestamp(self) -> None:
        ego = SimpleNamespace(stamp_seconds=1, stamp_nanos=20_000_000)

        self.assertTrue(is_new_state_sample(ego, None))
        self.assertFalse(is_new_state_sample(ego, (1, 20_000_000)))

        ego.stamp_nanos = 40_000_000
        self.assertTrue(is_new_state_sample(ego, (1, 20_000_000)))

    def test_fixed_step_control_waits_for_tcp_ack(self) -> None:
        udp = Mock()
        tcp = Mock()
        tcp.manual_control_by_id.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        control = Mock()

        result = send_ego_control(
            udp=udp,
            tcp=tcp,
            fixed_step=True,
            fixed_control_transport="tcp",
            ego_id="Ego",
            control=control,
        )

        self.assertEqual(result, {"result_code": 0, "detail_code": 0})
        tcp.manual_control_by_id.assert_called_once_with("Ego", control)
        udp.send_control.assert_not_called()

    def test_realtime_control_keeps_udp_transport(self) -> None:
        udp = Mock()
        control = Mock()

        result = send_ego_control(
            udp=udp,
            tcp=None,
            fixed_step=False,
            fixed_control_transport="tcp",
            ego_id="Ego",
            control=control,
        )

        self.assertIsNone(result)
        udp.send_control.assert_called_once_with(control)

    def test_fixed_step_tcp_control_failure_is_reported(self) -> None:
        tcp = Mock()
        tcp.manual_control_by_id.return_value = {
            "result_code": 1,
            "detail_code": 7,
        }

        with self.assertRaises(MoraiTcpError):
            send_ego_control(
                udp=Mock(),
                tcp=tcp,
                fixed_step=True,
                fixed_control_transport="tcp",
                ego_id="Ego",
                control=Mock(),
            )

    def test_route_is_built_from_runtime_ego_pose(self) -> None:
        route = Mock()
        route.route_points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        ego = SimpleNamespace(x=10.0, y=20.0, heading_rad=0.5)

        source = rebuild_route_from_runtime_pose(route, ego, 350.0)

        route.rebuild_from_pose.assert_called_once_with(
            10.0,
            20.0,
            0.5,
            min_route_length=350.0,
        )
        self.assertEqual(source, "MGeo runtime Ego pose route")

    def test_load_suite_requires_suite(self) -> None:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = main(["--execution-mode", "headless", "--load-suite"])
        self.assertEqual(result, 2)
        self.assertIn("requires --suite", output.getvalue())

    def test_scenario_play_alone_does_not_request_stop(self) -> None:
        args = build_parser().parse_args(["--scenario-play"])

        self.assertFalse(should_stop_before_start(args))

    @patch("tools.pdm_morai_control.stop_before_start")
    @patch("tools.pdm_morai_control.wait_for_udp_ready")
    @patch("tools.pdm_morai_control.MGeoRoutePlanner")
    @patch("tools.pdm_morai_control.MoraiUdpClient")
    @patch("tools.pdm_morai_control.MoraiTcpClient")
    def test_scenario_play_only_does_not_load_or_stop(
        self,
        tcp_class: Mock,
        udp_class: Mock,
        _route_class: Mock,
        wait_udp_mock: Mock,
        stop_mock: Mock,
    ) -> None:
        tcp = tcp_class.return_value
        tcp.active_suite_status.return_value = {
            "active_suite_name": "Suite",
            "active_scenario_name": "ActiveScenario",
            "scenario_list": ["ActiveScenario"],
        }
        tcp.scenario_status.return_value = {"result_code": 0, "detail_code": 0, "state": 1}
        tcp.get_simulation_time_status.return_value = {
            "mode": TIME_MODE_VARIABLE,
            "target_fps": 60,
            "physics_delta_time": 20,
            "rtf": 0,
            "user_control": 0,
            "step_index": 1,
            "seconds": 1,
            "nanos": 0,
        }
        tcp.scenario_control.return_value = {"result_code": 0, "detail_code": 0}
        wait_udp_mock.return_value = True
        udp = udp_class.return_value
        udp.states = {}
        udp.rx_records = 0
        udp.rx_chunks = 0
        udp.invalid_packets = 0
        udp.tx_packets = 0
        udp.rx_datagrams = 0
        udp.packet_sizes = []
        udp.last_error = None

        result = main(
            [
                "--scenario-play",
                "--tcp-port",
                "20000",
                "--tcp-settle",
                "0",
                "--duration",
                "0.001",
                "--no-send",
            ]
        )

        self.assertEqual(result, 0)
        stop_mock.assert_not_called()
        tcp.load_suite.assert_not_called()
        tcp.scenario_control.assert_called_once()
        wait_udp_mock.assert_called_once()

    def test_load_suite_does_not_request_stop_by_default(self) -> None:
        args = build_parser().parse_args(["--load-suite", "--suite", "test.msuite"])

        self.assertFalse(should_stop_before_start(args))

    def test_scenario_stop_explicitly_requests_stop(self) -> None:
        args = build_parser().parse_args(["--scenario-stop"])

        self.assertTrue(should_stop_before_start(args))

    @patch("tools.pdm_morai_control.wait_for_udp_ready")
    @patch("tools.pdm_morai_control.MGeoRoutePlanner")
    @patch("tools.pdm_morai_control.MoraiUdpClient")
    @patch("tools.pdm_morai_control.MoraiTcpClient")
    def test_plan_only_checks_without_route_or_control_loop(
        self,
        tcp_class: Mock,
        udp_class: Mock,
        route_class: Mock,
        wait_udp_mock: Mock,
    ) -> None:
        tcp = tcp_class.return_value
        tcp.active_suite_status.return_value = {
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario",
            "scenario_list": ["Scenario"],
        }
        tcp.scenario_status.return_value = {"result_code": 0, "detail_code": 0, "state": 1}
        tcp.get_simulation_time_status.return_value = {
            "mode": TIME_MODE_VARIABLE,
            "target_fps": 60,
            "physics_delta_time": 20,
            "rtf": 0,
            "user_control": 0,
            "step_index": 10,
            "seconds": 1,
            "nanos": 20,
        }
        wait_udp_mock.return_value = True
        udp = udp_class.return_value
        udp.states = {}
        udp.rx_records = 0
        udp.rx_chunks = 0
        udp.invalid_packets = 0
        udp.tx_packets = 0
        udp.rx_datagrams = 0
        udp.packet_sizes = []
        udp.last_error = None

        result = main(["--plan-only", "--tcp-port", "20000"])

        self.assertEqual(result, 0)
        route_class.assert_not_called()
        tcp.load_suite.assert_not_called()
        tcp.scenario_control.assert_not_called()

    @patch("tools.pdm_morai_control.wait_for_udp_ready")
    @patch("tools.pdm_morai_control.MGeoRoutePlanner")
    @patch("tools.pdm_morai_control.MoraiUdpClient")
    @patch("tools.pdm_morai_control.MoraiTcpClient")
    def test_udp_preflight_failure_stops_before_fixed_loop(
        self,
        tcp_class: Mock,
        udp_class: Mock,
        _route_class: Mock,
        wait_udp_mock: Mock,
    ) -> None:
        tcp = tcp_class.return_value
        tcp.get_simulation_time_status.return_value = {
            "mode": TIME_MODE_FIXED,
            "target_fps": 50,
            "physics_delta_time": 20,
            "rtf": 1,
            "user_control": 1,
            "step_index": 0,
            "seconds": 0,
            "nanos": 0,
        }
        tcp.active_suite_status.return_value = {
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario",
            "scenario_list": ["Scenario"],
        }
        tcp.scenario_status.return_value = {"result_code": 0, "detail_code": 0, "state": 1}
        tcp.set_simulation_time_mode.return_value = {"result_code": 0, "detail_code": 0}
        wait_udp_mock.return_value = False
        udp = udp_class.return_value
        udp.states = {}
        udp.rx_records = 0
        udp.rx_chunks = 0
        udp.invalid_packets = 0
        udp.tx_packets = 0
        udp.rx_datagrams = 0
        udp.packet_sizes = []
        udp.last_error = None

        result = main(
            [
                "--control-mode",
                "fixed-tcp-lockstep",
                "--tcp-port",
                "20000",
                "--tcp-settle",
                "0",
                "--duration",
                "0.02",
                "--no-send",
            ]
        )

        self.assertEqual(result, 2)
        tcp.fixed_step.assert_not_called()

    @patch("tools.pdm_morai_control.wait_for_udp_ready")
    @patch("tools.pdm_morai_control.MGeoRoutePlanner")
    @patch("tools.pdm_morai_control.MoraiUdpClient")
    @patch("tools.pdm_morai_control.MoraiTcpClient")
    def test_fixed_auto_udp_configures_fixed_time_without_fixed_step(
        self,
        tcp_class: Mock,
        udp_class: Mock,
        _route_class: Mock,
        wait_udp_mock: Mock,
    ) -> None:
        tcp = tcp_class.return_value
        applied = {
            "mode": TIME_MODE_FIXED,
            "target_fps": 50,
            "physics_delta_time": 20,
            "rtf": 1,
            "user_control": 0,
            "step_index": 0,
            "seconds": 0,
            "nanos": 0,
        }
        tcp.get_simulation_time_status.side_effect = [
            {"mode": TIME_MODE_VARIABLE},
            applied,
            applied,
        ]
        tcp.active_suite_status.return_value = {
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario",
            "scenario_list": ["Scenario"],
        }
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }
        tcp.set_simulation_time_mode.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        wait_udp_mock.return_value = False
        udp = udp_class.return_value
        udp.states = {}
        udp.rx_records = 0
        udp.rx_chunks = 0
        udp.invalid_packets = 0
        udp.tx_packets = 0
        udp.rx_datagrams = 0
        udp.packet_sizes = []
        udp.last_error = None

        result = main(
            [
                "--control-mode",
                "fixed-auto-udp",
                "--tcp-port",
                "20000",
                "--tcp-settle",
                "0",
                "--no-send",
            ]
        )

        self.assertEqual(result, 2)
        tcp.set_simulation_time_mode.assert_called_once_with(
            TIME_MODE_FIXED,
            50,
            20,
            1,
            0,
        )
        tcp.fixed_step.assert_not_called()
        wait_udp_mock.assert_called_once()
        self.assertIsNone(wait_udp_mock.call_args[0][1])
        self.assertFalse(wait_udp_mock.call_args[1]["fixed_step"])

    def test_fixed_status_matching_uses_milliseconds(self) -> None:
        status = {
            "mode": TIME_MODE_FIXED,
            "target_fps": 50,
            "physics_delta_time": 20,
            "rtf": 1,
            "user_control": 1,
        }

        self.assertTrue(
            fixed_time_status_matches(
                status,
                control_hz=50,
                physics_delta_time=20,
                rtf=1,
                user_control=1,
            )
        )

    def test_variable_status_matching_accepts_simulator_normalized_rtf(self) -> None:
        status = {
            "mode": TIME_MODE_VARIABLE,
            "target_fps": 60,
            "physics_delta_time": 20,
            "rtf": 1,
            "user_control": 0,
        }

        self.assertTrue(
            variable_time_status_matches(
                status,
                control_hz=60,
                physics_delta_time=20,
            )
        )

    def test_pdm_step_delta_controls_sut_fps_independent_of_physics_delta(self) -> None:
        args = build_parser().parse_args(
            [
                "--fixed-step",
                "--pdm-step-delta-time",
                "20",
                "--physics-delta-time",
                "10",
            ]
        )

        control_hz, time_step = resolve_sut_timing(args, 60)

        self.assertEqual(control_hz, 50)
        self.assertEqual(time_step, 0.020)

    def test_fixed_target_fps_sets_pdm_dt_when_step_delta_is_omitted(self) -> None:
        args = build_parser().parse_args(
            [
                "--fixed-step",
                "--fixed-target-fps",
                "50",
                "--physics-delta-time",
                "10",
            ]
        )

        control_hz, time_step = resolve_sut_timing(args, 60)

        self.assertEqual(control_hz, 50)
        self.assertEqual(time_step, 0.020)

    def test_pdm_step_delta_requires_integer_target_fps(self) -> None:
        args = build_parser().parse_args(
            ["--fixed-step", "--pdm-step-delta-time", "15"]
        )

        with self.assertRaises(ValueError):
            resolve_sut_timing(args, 60)

    def test_fixed_step_result_failure_is_reported(self) -> None:
        tcp = Mock()
        tcp.get_simulation_time_status.return_value = {"mode": 1}
        tcp.set_simulation_time_mode.return_value = {
            "result_code": 1,
            "detail_code": 99,
        }
        args = build_parser().parse_args(
            [
                "--fixed-step",
                "--tcp-port",
                "20000",
                "--fixed-target-fps",
                "50",
                "--physics-delta-time",
                "20",
            ]
        )

        with self.assertRaises(MoraiTcpError):
            configure_fixed_step(tcp, args, 50)

    def test_fixed_step_success_is_verified(self) -> None:
        tcp = Mock()
        applied = {
            "mode": TIME_MODE_FIXED,
            "target_fps": 50,
            "physics_delta_time": 20,
            "rtf": 1,
            "user_control": 1,
        }
        tcp.get_simulation_time_status.side_effect = [{"mode": 1}, applied]
        tcp.set_simulation_time_mode.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        args = build_parser().parse_args(
            [
                "--fixed-step",
                "--tcp-port",
                "20000",
                "--fixed-target-fps",
                "50",
                "--physics-delta-time",
                "20",
            ]
        )

        configure_fixed_step(tcp, args, 50)

        tcp.set_simulation_time_mode.assert_called_once_with(
            TIME_MODE_FIXED,
            50,
            20,
            1,
            1,
        )

    def test_fixed_step_uses_pdm_delta_for_target_fps_only(self) -> None:
        tcp = Mock()
        applied = {
            "mode": TIME_MODE_FIXED,
            "target_fps": 50,
            "physics_delta_time": 10,
            "rtf": 1,
            "user_control": 1,
        }
        tcp.get_simulation_time_status.side_effect = [{"mode": 1}, applied]
        tcp.set_simulation_time_mode.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        args = build_parser().parse_args(
            [
                "--fixed-step",
                "--pdm-step-delta-time",
                "20",
                "--physics-delta-time",
                "10",
            ]
        )
        control_hz, _time_step = resolve_sut_timing(args, 60)

        configure_fixed_step(tcp, args, control_hz)

        tcp.set_simulation_time_mode.assert_called_once_with(
            TIME_MODE_FIXED,
            50,
            10,
            1,
            1,
        )

    def test_fixed_step_applied_status_mismatch_is_reported(self) -> None:
        tcp = Mock()
        tcp.get_simulation_time_status.side_effect = [
            {"mode": 1},
            {
                "mode": TIME_MODE_FIXED,
                "target_fps": 30,
                "physics_delta_time": 33,
                "rtf": 1,
                "user_control": 1,
            },
        ]
        tcp.set_simulation_time_mode.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        args = build_parser().parse_args(
            [
                "--fixed-step",
                "--tcp-port",
                "20000",
                "--fixed-target-fps",
                "50",
                "--physics-delta-time",
                "20",
            ]
        )

        with self.assertRaises(MoraiTcpError):
            configure_fixed_step(tcp, args, 50)


if __name__ == "__main__":
    unittest.main()
