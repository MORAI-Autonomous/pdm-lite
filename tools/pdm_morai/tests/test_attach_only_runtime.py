# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from tools.pdm_morai_control import (
    ActiveScenarioChanged,
    DEFAULT_ATTACH_MGEO_DIR,
    ScenarioBoundaryDetected,
    bootstrap_fixed_tcp_ego,
    build_parser,
    build_runtime_settings,
    configure_fixed_step,
    fixed_step_and_wait_for_ego,
    fixed_time_status_matches,
    hold_for_sim_scenario_transition,
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
    verify_suite_preflight,
)
from pdm_morai.pdm_config import (
    PdmLiteMoraiConfig,
    apply_pdm_profile,
    pdm_profile_target_speed_scale,
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
        self.assertEqual(args.pdm_profile, "normal")
        self.assertEqual(args.scenario_transition_wait, 3.0)
        self.assertEqual(select_runtime_mode(args), "gui:variable-udp")

    def test_pdm_profiles_patch_behavior_parameters(self) -> None:
        normal = PdmLiteMoraiConfig()
        mad_max = PdmLiteMoraiConfig()
        beginner = PdmLiteMoraiConfig()
        idiot = PdmLiteMoraiConfig()

        apply_pdm_profile(mad_max, "mad-max")
        apply_pdm_profile(beginner, "beginner")
        apply_pdm_profile(idiot, "idiot")

        self.assertLess(mad_max.leading_vehicle_time_gap, normal.leading_vehicle_time_gap)
        self.assertGreater(mad_max.throttle_increase_rate, normal.throttle_increase_rate)
        self.assertLess(beginner.route_speed_limit_ratio, normal.route_speed_limit_ratio)
        self.assertGreater(beginner.leading_vehicle_min_gap, normal.leading_vehicle_min_gap)
        self.assertLess(idiot.leading_vehicle_lateral_window, normal.leading_vehicle_lateral_window)
        self.assertLess(idiot.brake_increase_rate, normal.brake_increase_rate)

    def test_pdm_profile_target_speed_scales_are_distinct(self) -> None:
        self.assertEqual(pdm_profile_target_speed_scale("normal"), 1.0)
        self.assertGreater(pdm_profile_target_speed_scale("mad-max"), 1.0)
        self.assertLess(pdm_profile_target_speed_scale("beginner"), 1.0)
        self.assertGreater(pdm_profile_target_speed_scale("idiot"), 1.0)

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
        fixed_udp = parse_runtime_args(["--control-mode", "fixed-udp"])
        lockstep = parse_runtime_args(["--control-mode", "fixed-tcp-lockstep"])

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

        ego.stamp_seconds = 2
        ego.stamp_nanos = 1
        self.assertTrue(is_new_state_sample(ego, (1, 999_999_999)))

        ego.stamp_seconds = 1
        self.assertFalse(is_new_state_sample(ego, (2, 1)))

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

    def test_fixed_step_wait_uses_ack_save_status_then_advanced_udp_stamp(self) -> None:
        previous = SimpleNamespace(stamp_seconds=1, stamp_nanos=999_999_999)
        advanced = SimpleNamespace(stamp_seconds=2, stamp_nanos=1)
        udp = SimpleNamespace(
            states={"Ego": previous},
            poll=Mock(),
            rx_records=1,
            rx_chunks=0,
            invalid_packets=0,
        )
        udp.poll.side_effect = lambda: udp.states.__setitem__("Ego", advanced)
        tcp = Mock()
        tcp.fixed_step.return_value = {"result_code": 0, "detail_code": 0}
        tcp.save_data.return_value = {"result_code": 0, "detail_code": 0}
        tcp.active_suite_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario A",
            "scenario_list": ["Scenario A"],
        }
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }

        ego, fixed_result, save_result = fixed_step_and_wait_for_ego(
            tcp=tcp,
            udp=udp,
            previous_stamp=(1, 999_999_999),
            timeout=1.0,
            status_period=1.0,
            save_data_each_step=True,
            ego_candidates=("Ego",),
            expected_scenario_name="Scenario A",
        )

        self.assertIs(ego, advanced)
        self.assertEqual(fixed_result["result_code"], 0)
        self.assertEqual(save_result["result_code"], 0)
        self.assertEqual(
            [call[0] for call in tcp.method_calls],
            ["fixed_step", "save_data", "active_suite_status", "scenario_status"],
        )

    def test_fixed_step_ack_failure_stops_before_save_status_and_udp_poll(self) -> None:
        tcp = Mock()
        tcp.fixed_step.return_value = {"result_code": 1, "detail_code": 9}
        udp = Mock()

        with self.assertRaisesRegex(MoraiTcpError, r"FixedStep\(1\) failed"):
            fixed_step_and_wait_for_ego(
                tcp=tcp,
                udp=udp,
                previous_stamp=(1, 0),
                timeout=1.0,
                status_period=1.0,
                save_data_each_step=True,
                ego_candidates=("Ego",),
            )

        tcp.save_data.assert_not_called()
        tcp.scenario_status.assert_not_called()
        udp.poll.assert_not_called()

    def test_fixed_tcp_bootstrap_steps_before_first_manual_control(self) -> None:
        ego = SimpleNamespace(stamp_seconds=0, stamp_nanos=20_000_000)
        udp = SimpleNamespace(
            states={},
            poll=Mock(),
            rx_records=0,
            rx_chunks=0,
            rx_datagrams=0,
            invalid_packets=0,
            packet_sizes=[],
            last_error=None,
        )

        def publish_after_step() -> None:
            if udp.poll.call_count >= 2:
                udp.states["Ego"] = ego
                udp.rx_records = 1

        udp.poll.side_effect = publish_after_step
        tcp = Mock()
        tcp.manual_control_by_id.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        tcp.fixed_step.return_value = {"result_code": 0, "detail_code": 0}
        tcp.save_data.return_value = {"result_code": 0, "detail_code": 0}
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }

        result, fixed_result, save_result = bootstrap_fixed_tcp_ego(
            tcp=tcp,
            udp=udp,
            ego_id="Ego",
            timeout=1.0,
            status_period=1.0,
            save_data_each_step=True,
            ego_candidates=("Ego",),
            send_control=True,
        )

        self.assertIs(result, ego)
        self.assertEqual(fixed_result["result_code"], 0)
        self.assertEqual(save_result["result_code"], 0)
        self.assertEqual(
            [call[0] for call in tcp.method_calls],
            ["fixed_step", "save_data", "scenario_status"],
        )

    def test_fixed_tcp_bootstrap_skips_step_when_initial_ego_exists(self) -> None:
        ego = SimpleNamespace(stamp_seconds=0, stamp_nanos=0)
        udp = SimpleNamespace(
            states={"Ego": ego},
            poll=Mock(),
            rx_records=1,
            rx_chunks=0,
            rx_datagrams=1,
            invalid_packets=0,
            packet_sizes=[108],
            last_error=None,
        )
        tcp = Mock()

        result, fixed_result, save_result = bootstrap_fixed_tcp_ego(
            tcp=tcp,
            udp=udp,
            ego_id="Ego",
            timeout=1.0,
            status_period=1.0,
            save_data_each_step=True,
            ego_candidates=("Ego",),
            send_control=True,
        )

        self.assertIs(result, ego)
        self.assertIsNone(fixed_result)
        self.assertIsNone(save_result)
        tcp.manual_control_by_id.assert_not_called()
        tcp.fixed_step.assert_not_called()

    def test_transition_bootstrap_discards_old_ego_and_forces_one_step(self) -> None:
        old_ego = SimpleNamespace(stamp_seconds=44, stamp_nanos=460_000_000)
        new_ego = SimpleNamespace(stamp_seconds=0, stamp_nanos=20_000_000)
        udp = SimpleNamespace(
            states={"Ego": old_ego},
            poll=Mock(),
            rx_records=1,
            rx_chunks=0,
            rx_datagrams=1,
            invalid_packets=0,
            packet_sizes=[108],
            last_error=None,
        )

        def publish_new_ego() -> None:
            if udp.poll.call_count >= 2:
                udp.states["Ego"] = new_ego
                udp.rx_records = 2

        udp.poll.side_effect = publish_new_ego
        tcp = Mock()
        tcp.manual_control_by_id.return_value = {
            "result_code": 0,
            "detail_code": 0,
        }
        tcp.fixed_step.return_value = {"result_code": 0, "detail_code": 0}
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }

        result, fixed_result, _save_result = bootstrap_fixed_tcp_ego(
            tcp=tcp,
            udp=udp,
            ego_id="Ego",
            timeout=1.0,
            status_period=1.0,
            save_data_each_step=False,
            ego_candidates=("Ego",),
            send_control=True,
            force_step=True,
        )

        self.assertIs(result, new_ego)
        self.assertEqual(fixed_result["result_code"], 0)
        tcp.manual_control_by_id.assert_not_called()
        tcp.fixed_step.assert_called_once_with(1)

    @patch("tools.pdm_morai_control.time.sleep")
    @patch("tools.pdm_morai_control.time.monotonic", side_effect=[0.0, 2.0])
    def test_fixed_step_wait_rejects_nonadvancing_udp_stamp(
        self,
        _monotonic: Mock,
        _sleep: Mock,
    ) -> None:
        ego = SimpleNamespace(stamp_seconds=3, stamp_nanos=10)
        udp = SimpleNamespace(
            states={"Ego": ego},
            poll=Mock(),
            rx_records=1,
            rx_chunks=0,
            invalid_packets=0,
        )
        tcp = Mock()
        tcp.fixed_step.return_value = {"result_code": 0, "detail_code": 0}
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }

        with self.assertRaisesRegex(RuntimeError, "fixed_ack_udp_wait_timeout"):
            fixed_step_and_wait_for_ego(
                tcp=tcp,
                udp=udp,
                previous_stamp=(3, 10),
                timeout=1.0,
                status_period=1.0,
                save_data_each_step=False,
                ego_candidates=("Ego",),
            )

    def test_fixed_step_detects_sim_owned_active_scenario_change(self) -> None:
        tcp = Mock()
        tcp.fixed_step.return_value = {"result_code": 0, "detail_code": 0}
        tcp.active_suite_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario B",
            "scenario_list": ["Scenario A", "Scenario B"],
        }
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }
        udp = Mock()

        with self.assertRaises(ActiveScenarioChanged) as raised:
            fixed_step_and_wait_for_ego(
                tcp=tcp,
                udp=udp,
                previous_stamp=(12, 340_000_000),
                timeout=1.0,
                status_period=1.0,
                save_data_each_step=False,
                ego_candidates=("Ego",),
                expected_scenario_name="Scenario A",
            )

        self.assertEqual(raised.exception.previous_name, "Scenario A")
        self.assertEqual(raised.exception.active_name, "Scenario B")
        self.assertEqual(
            [call[0] for call in tcp.method_calls],
            ["fixed_step", "active_suite_status", "scenario_status"],
        )
        udp.poll.assert_not_called()

    def test_fixed_step_timeout_recovers_when_sim_already_changed_scenario(self) -> None:
        tcp = Mock()
        tcp.fixed_step.side_effect = TimeoutError("timed out")
        tcp.active_suite_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario B",
            "scenario_list": ["Scenario A", "Scenario B"],
        }
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }
        udp = Mock()

        with self.assertRaises(ActiveScenarioChanged) as raised:
            fixed_step_and_wait_for_ego(
                tcp=tcp,
                udp=udp,
                previous_stamp=(20, 0),
                timeout=20.0,
                status_period=1.0,
                save_data_each_step=True,
                ego_candidates=("Ego",),
                expected_scenario_name="Scenario A",
            )

        self.assertEqual(raised.exception.active_name, "Scenario B")
        tcp.close.assert_called_once_with()
        tcp.open.assert_called_once_with()
        tcp.active_suite_status.assert_called_once_with()
        tcp.scenario_status.assert_called_once_with()
        tcp.save_data.assert_not_called()
        udp.poll.assert_not_called()

    def test_fixed_step_stops_stepping_immediately_at_scenario_boundary(self) -> None:
        ego = SimpleNamespace(stamp_seconds=20, stamp_nanos=0)
        udp = SimpleNamespace(
            states={"Ego": ego},
            poll=Mock(),
            rx_records=1000,
            rx_chunks=0,
            invalid_packets=0,
        )
        tcp = Mock()
        tcp.fixed_step.return_value = {"result_code": 0, "detail_code": 0}
        tcp.active_suite_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario A",
            "scenario_list": ["Scenario A", "Scenario B"],
        }
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 3,
        }

        with self.assertRaises(ScenarioBoundaryDetected):
            fixed_step_and_wait_for_ego(
                tcp=tcp,
                udp=udp,
                previous_stamp=(20, 0),
                timeout=20.0,
                status_period=1.0,
                save_data_each_step=False,
                ego_candidates=("Ego",),
                expected_scenario_name="Scenario A",
            )

        tcp.active_suite_status.assert_called_once_with()
        tcp.scenario_status.assert_called_once_with()
        udp.poll.assert_not_called()

    def test_transition_hold_confirms_new_name_and_discards_cached_udp(self) -> None:
        udp = SimpleNamespace(states={"Ego": object()}, poll=Mock())
        tcp = Mock()
        tcp.active_suite_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "active_suite_name": "Suite",
            "active_scenario_name": "Scenario B",
            "scenario_list": ["Scenario A", "Scenario B"],
        }
        tcp.scenario_status.return_value = {
            "result_code": 0,
            "detail_code": 0,
            "state": 1,
        }

        change = hold_for_sim_scenario_transition(
            tcp=tcp,
            udp=udp,
            previous_name="Scenario A",
            timeout=0.0,
            status_period=1.0,
        )

        self.assertIsNotNone(change)
        self.assertEqual(change.active_name, "Scenario B")
        self.assertEqual(udp.states, {})
        self.assertEqual(udp.poll.call_count, 1)

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

    def test_explicit_play_allows_fixed_bootstrap_from_stopped_state(self) -> None:
        args = build_parser().parse_args(
            ["--control-mode", "fixed-tcp-lockstep", "--scenario-play"]
        )

        result = verify_suite_preflight(
            args=args,
            binding=None,
            active_status={
                "active_suite_name": "Suite",
                "active_scenario_name": "Scenario",
            },
            scenario_status={"result_code": 0, "detail_code": 0, "state": 3},
        )

        self.assertTrue(result)

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
        self.assertTrue(wait_udp_mock.call_args.kwargs["require_stamp_advance"])

    @patch("tools.pdm_morai_control.wait_for_udp_ready")
    @patch("tools.pdm_morai_control.MGeoRoutePlanner")
    @patch("tools.pdm_morai_control.MoraiUdpClient")
    @patch("tools.pdm_morai_control.MoraiTcpClient")
    def test_fixed_tcp_bootstrap_failure_stops_before_control_loop(
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
        tcp.fixed_step.return_value = {"result_code": 1, "detail_code": 9}
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
        tcp.fixed_step.assert_called_once_with(1)
        wait_udp_mock.assert_not_called()

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

    def test_variable_status_matching_requires_api_forced_zero_flags(self) -> None:
        status = {
            "mode": TIME_MODE_VARIABLE,
            "target_fps": 60,
            "physics_delta_time": 20,
            "rtf": 0,
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

    def test_fixed_step_force_reapply_does_not_trust_stale_matching_status(self) -> None:
        tcp = Mock()
        applied = {
            'mode': TIME_MODE_FIXED,
            'target_fps': 50,
            'physics_delta_time': 20,
            'rtf': 1,
            'user_control': 1,
        }
        tcp.get_simulation_time_status.side_effect = [applied, applied]
        tcp.set_simulation_time_mode.return_value = {
            'result_code': 0,
            'detail_code': 0,
        }
        args = build_parser().parse_args(
            [
                '--fixed-step',
                '--tcp-port',
                '20000',
                '--fixed-target-fps',
                '50',
                '--physics-delta-time',
                '20',
            ]
        )

        configure_fixed_step(tcp, args, 50, force_reapply=True)

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
