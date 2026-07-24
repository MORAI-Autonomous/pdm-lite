from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[2]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pdm_morai.scenario import resolve_binding_config


class BindingConfigTest(unittest.TestCase):
    def test_resolves_from_project_interfaces_and_xosc_without_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            suite = root / "scenario.msuite"
            project_dir = root / "Project"
            project_dir.mkdir()
            project = project_dir / "scenario.mproj"
            xosc = project_dir / "scenario.xosc"

            suite.write_text(
                json.dumps({"projectList": [{"filepath": "Project/scenario.mproj"}]}),
                encoding="utf-8",
            )
            project.write_text(
                json.dumps(
                    {
                        "scenarioList": [{"path": "scenario.xosc"}],
                        "interfaceList": [
                            {"name": "Vehicle Info Ego", "path": "Vehicle Info Ego.json"},
                            {"name": "Manual Control", "path": "Manual Control.json"},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (project_dir / "Vehicle Info Ego.json").write_text(
                json.dumps(
                    {
                        "interface": {
                            "name": "Vehicle Info Ego",
                            "direction": "SEND",
                            "hertz": 25,
                            "destinationAddr": {"ip": "127.0.0.1", "port": 19097},
                        }
                    }
                ),
                encoding="utf-8",
            )
            (project_dir / "Manual Control.json").write_text(
                json.dumps(
                    {
                        "interface": {
                            "name": "Manual Control",
                            "direction": "RECEIVE",
                            "hertz": 50,
                            "hostAddr": {"ip": "127.0.0.1", "port": 19091},
                        }
                    }
                ),
                encoding="utf-8",
            )
            xosc.write_text(
                """\
<OpenSCENARIO>
  <Entities>
    <ScenarioObject name="ego">
      <Vehicle name="ego">
        <Properties><Property name="external_control" value="true" /></Properties>
      </Vehicle>
    </ScenarioObject>
    <ScenarioObject name="npc1"><Vehicle name="npc1" /></ScenarioObject>
  </Entities>
</OpenSCENARIO>
""",
                encoding="utf-8",
            )

            binding = resolve_binding_config(suite)

            self.assertEqual(binding.ego_id, "ego")
            self.assertEqual(binding.entity_ids, ["ego", "npc1"])
            self.assertEqual(binding.ego_info_port, 19097)
            self.assertEqual(binding.ctrl_port, 19091)
            self.assertEqual(binding.feedback_hz, 25)
            self.assertEqual(binding.control_hz, 50)

    def test_scenario_name_selects_matching_project_xosc(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            suite = root / "suite.msuite"
            project_dir = root / "Project"
            project_dir.mkdir()
            project = project_dir / "project.mproj"
            first_xosc = project_dir / "first.xosc"
            second_xosc = project_dir / "second case.xosc"

            suite.write_text(
                json.dumps({"projectList": [{"filepath": "Project/project.mproj"}]}),
                encoding="utf-8",
            )
            project.write_text(
                json.dumps(
                    {
                        "scenarioList": [
                            {"name": "first", "path": "first.xosc"},
                            {"name": "second scenario", "path": "second case.xosc"},
                        ],
                        "interfaceList": [
                            {"name": "Vehicle Info Ego", "path": "Vehicle Info Ego.json"},
                            {"name": "Manual Control", "path": "Manual Control.json"},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (project_dir / "Vehicle Info Ego.json").write_text(
                json.dumps(
                    {
                        "interface": {
                            "name": "Vehicle Info Ego",
                            "direction": "SEND",
                            "destinationAddr": {"ip": "127.0.0.1", "port": 19097},
                        }
                    }
                ),
                encoding="utf-8",
            )
            (project_dir / "Manual Control.json").write_text(
                json.dumps(
                    {
                        "interface": {
                            "name": "Manual Control",
                            "direction": "RECEIVE",
                            "hostAddr": {"ip": "127.0.0.1", "port": 19091},
                        }
                    }
                ),
                encoding="utf-8",
            )
            first_xosc.write_text(
                """\
<OpenSCENARIO>
  <Entities><ScenarioObject name="first_ego"><Vehicle name="first_ego" /></ScenarioObject></Entities>
</OpenSCENARIO>
""",
                encoding="utf-8",
            )
            second_xosc.write_text(
                """\
<OpenSCENARIO>
  <Entities><ScenarioObject name="second_ego"><Vehicle name="second_ego" /></ScenarioObject></Entities>
</OpenSCENARIO>
""",
                encoding="utf-8",
            )

            binding = resolve_binding_config(suite, "second scenario")

            self.assertEqual(binding.scenario_path, second_xosc.resolve())
            self.assertEqual(binding.ego_id, "second_ego")


if __name__ == "__main__":
    unittest.main()
