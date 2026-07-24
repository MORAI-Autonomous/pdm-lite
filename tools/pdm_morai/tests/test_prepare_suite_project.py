from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


TOOLS_DIR = Path(__file__).resolve().parents[2]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pdm_morai_prepare_suite import (
    SIM_MAP_NAME_BY_ID,
    build_project,
    build_vehicle_info_template,
    parse_xosc,
    resolve_runner_map,
    sim_map_id,
    sim_map_name,
)


class PreparedProjectTest(unittest.TestCase):
    def test_source_project_map_id_has_priority_over_names_and_logic_file(self) -> None:
        warnings: list[str] = []

        runner_map = resolve_runner_map(
            "KATRI",
            "Hwaseong-si",
            warnings,
            source_project_map={"name": "Hwaseong-si", "iD": 100011},
        )
        info = SimpleNamespace(
            source_project_map={"name": "Hwaseong-si", "iD": 100011},
            morai_map="Hwaseong-si",
            runner_map_name=runner_map,
        )

        self.assertEqual(runner_map, "KATRI")
        self.assertEqual(sim_map_id(info), 100011)
        self.assertEqual(sim_map_name(info), "KATRI")
        self.assertIn("SIM map id 100011", warnings[0])

    def test_unknown_source_map_id_falls_back_to_logic_file_then_names(self) -> None:
        warnings: list[str] = []

        runner_map = resolve_runner_map(
            "KATRI",
            "KATRI",
            warnings,
            source_project_map={"name": "Hwaseong-si", "iD": 999999},
        )

        self.assertEqual(runner_map, "KATRI")

    def test_name_fallback_uses_current_katri_map_id(self) -> None:
        self.assertEqual(sim_map_id("KATRI"), 100011)
        self.assertEqual(sim_map_id("K-City"), 100011)
        self.assertEqual(sim_map_id("KATRI (Old)"), 100002)
        self.assertEqual(sim_map_id("Hwaseong-si"), 100003)

    def test_all_map_table_ids_round_trip_through_canonical_names(self) -> None:
        expected_ids = {
            *range(100001, 100012),
            *range(110001, 110013),
        }

        self.assertEqual(set(SIM_MAP_NAME_BY_ID), expected_ids)
        for map_id, map_name in SIM_MAP_NAME_BY_ID.items():
            self.assertEqual(sim_map_id(map_name), map_id)

    def test_map_table_option_names_are_supported_as_fallbacks(self) -> None:
        self.assertEqual(sim_map_id("seoul"), 100001)
        self.assertEqual(sim_map_id("gyeongbu_expressway"), 100006)
        self.assertEqual(sim_map_id("dt_su_kcity"), 100011)
        self.assertEqual(sim_map_id("vm_ub_morai_city"), 110012)

    def test_vehicle_info_pose_fields_match_sim_float_wire_format(self) -> None:
        template = build_vehicle_info_template()
        fields = template["messageTemplate"]["segmentList"][0]["fieldList"]

        self.assertEqual(
            [field["variableType"] for field in fields[3:9]],
            ["FLOAT"] * 6,
        )
        self.assertEqual(
            [field["variableType"] for field in fields[9:]],
            ["FLOAT"] * 12,
        )

    def test_generated_runtime_config_is_nested_under_scenario(self) -> None:
        info = SimpleNamespace(
            path=Path("source.xosc"),
            target_xosc_path=Path("generated/Project/source.xosc"),
            source_project_map=None,
            source_scenario_config=None,
            morai_map="Hwaseong-si",
            runner_map_name="KATRI",
            ego_entity="Ego",
            npc_entities=["NPC_1"],
            vehicles={
                "Ego": SimpleNamespace(initial_speed_mps=0.0),
                "NPC_1": SimpleNamespace(initial_speed_mps=10.0),
            },
        )

        project = build_project(info)

        scenario = project["scenarioList"][0]
        self.assertIn("config", scenario)
        self.assertNotIn("kineticsModelList", project)
        self.assertNotIn("AgentModels", scenario["config"])
        self.assertEqual(
            [(item["parentName"], item["dynamicType"]) for item in scenario["config"]["kineticsModelList"]],
            [("Ego", "Dynamics"), ("NPC_1", "Kinematic")],
        )
        self.assertEqual(
            scenario["config"]["vehicleAgentTypeList"],
            [
                {"ObjectName": "Ego", "AgentType": 1},
                {"ObjectName": "NPC_1", "AgentType": 0},
            ],
        )

    def test_missing_runtime_lists_are_added_to_source_scenario_config(self) -> None:
        info = SimpleNamespace(
            path=Path("source.xosc"),
            target_xosc_path=Path("generated/Project/source.xosc"),
            source_project_map={"type": "Standard", "name": "KATRI", "iD": 100011},
            source_scenario_config={"weatherPreset": "SUNNY", "sensorList": []},
            morai_map="KATRI",
            runner_map_name="KATRI",
            ego_entity="ego",
            npc_entities=["npc1"],
            vehicles={
                "ego": SimpleNamespace(initial_speed_mps=0.0),
                "npc1": SimpleNamespace(initial_speed_mps=0.0),
            },
        )

        config = build_project(info)["scenarioList"][0]["config"]

        self.assertEqual(config["weatherPreset"], "SUNNY")
        self.assertEqual(
            [(item["parentName"], item["dynamicType"]) for item in config["kineticsModelList"]],
            [("ego", "Dynamics"), ("npc1", "Kinematic")],
        )
        self.assertEqual(
            config["vehicleAgentTypeList"],
            [
                {"ObjectName": "ego", "AgentType": 1},
                {"ObjectName": "npc1", "AgentType": 0},
            ],
        )

    def test_legacy_top_level_scenario_config_is_nested_and_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            xosc = root / "scenario.xosc"
            xosc.write_text(
                """\
<OpenSCENARIO>
  <VariableDeclarations>
    <VariableDeclaration name="moraiMap" value="Hwaseong-si" />
  </VariableDeclarations>
  <RoadNetwork><LogicFile filepath="MGeo" /></RoadNetwork>
  <Entities>
    <ScenarioObject name="Ego">
      <Vehicle name="Ego" model3d="HD Ioniq5">
        <Properties><Property name="MotionModel" value="Dynamics" /></Properties>
      </Vehicle>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <Private entityRef="Ego">
          <PrivateAction>
            <TeleportAction>
              <Position><WorldPosition x="0" y="0" z="0" h="0" /></Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>
  </Storyboard>
</OpenSCENARIO>
""",
                encoding="utf-8",
            )
            project_path = root / "Project.mproj"
            project_path.write_text(
                json.dumps(
                    {
                        "map": {"type": "Standard", "name": "Hwaseong-si", "id": 100003},
                        "sensorList": [],
                        "kineticsModelList": [
                            {
                                "modelName": "None",
                                "type": "Off",
                                "optionId": 0,
                                "parentName": "Ego",
                                "dynamicType": "Dynamics",
                            }
                        ],
                        "DynamicsCustomData": [{"objectName": "Ego"}],
                        "vehicleAgentTypeList": [{"ObjectName": "Ego", "AgentType": 1}],
                        "scenarioList": [{"name": "scenario", "path": "scenario.xosc"}],
                    }
                ),
                encoding="utf-8",
            )

            info = parse_xosc(xosc, None, root / "generated")
            project = build_project(info)
            scenario = project["scenarioList"][0]

            self.assertNotIn("kineticsModelList", project)
            self.assertEqual(
                scenario["config"]["kineticsModelList"][0],
                {
                    "modelName": "None",
                    "type": "Off",
                    "optionId": 0,
                    "parentName": "Ego",
                    "dynamicType": "Dynamics",
                },
            )


if __name__ == "__main__":
    unittest.main()
