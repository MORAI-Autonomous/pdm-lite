from __future__ import annotations

import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace


TOOLS_DIR = Path(__file__).resolve().parents[2]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pdm_morai_prepare_suite import parse_initial_state, write_prepared_xosc


class PreparedXoscTest(unittest.TestCase):
    def test_lane_position_is_resolved_from_mgeo_and_ego_speed_is_forced_to_zero(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            mgeo_dir = root / "MGeo"
            mgeo_dir.mkdir()
            (mgeo_dir / "link_set.json").write_text(
                """[{"idx":"L1","points":[[0,0,1],[10,0,1]],"road_id":"R1"}]""",
                encoding="utf-8",
            )
            source = root / "source.xosc"
            target = root / "generated" / "source.xosc"
            target.parent.mkdir()
            source.write_text(
                """\
<OpenSCENARIO>
  <Storyboard><Init><Actions>
    <Private entityRef="ego">
      <PrivateAction><TeleportAction><Position>
        <LanePosition laneId="L1" roadId="R1" s="5" offset="2">
          <Orientation h="1.25" type="absolute" />
        </LanePosition>
      </Position></TeleportAction></PrivateAction>
      <PrivateAction><LongitudinalAction><SpeedAction><SpeedActionTarget>
        <AbsoluteTargetSpeed value="12" />
      </SpeedActionTarget></SpeedAction></LongitudinalAction></PrivateAction>
    </Private>
  </Actions></Init></Storyboard>
</OpenSCENARIO>
""",
                encoding="utf-8",
            )

            parsed_root = ET.parse(source).getroot()
            poses, speeds = parse_initial_state(parsed_root, {}, mgeo_dir)
            self.assertAlmostEqual(poses["ego"].x, 5.0)
            self.assertAlmostEqual(poses["ego"].y, 2.0)
            self.assertAlmostEqual(poses["ego"].z, 1.0)
            self.assertAlmostEqual(poses["ego"].h, 1.25)
            self.assertEqual(speeds["ego"], 12.0)

            info = SimpleNamespace(
                path=source,
                target_xosc_path=target,
                target_project_path=target.parent / "Project.mproj",
                source_project_map=None,
                morai_map="KATRI",
                runner_map_name="KATRI",
                logic_file="MGeo",
                ego_entity="ego",
                parameters={},
            )
            write_prepared_xosc(info, headless_control=False, mgeo_mode="copy")
            generated = ET.parse(target).getroot()

            self.assertIsNotNone(generated.find(".//LanePosition"))
            self.assertEqual(
                generated.find(".//AbsoluteTargetSpeed").get("value"),
                "0",
            )

    def test_lane_position_has_priority_over_world_position(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            mgeo_dir = root / "MGeo"
            mgeo_dir.mkdir()
            (mgeo_dir / "link_set.json").write_text(
                """[{"idx":"AS001","points":[[10,20,1],[20,20,1]],"road_id":"R1"}]""",
                encoding="utf-8",
            )
            xosc = ET.fromstring(
                """\
<OpenSCENARIO><Storyboard><Init><Actions><Private entityRef="ego">
  <PrivateAction><TeleportAction><Position>
    <WorldPosition x="999" y="999" z="999" h="0" />
  </Position></TeleportAction></PrivateAction>
  <PrivateAction><TeleportAction><Position>
    <LanePosition laneId="AS001" roadId="R1" s="5" offset="0" />
  </Position></TeleportAction></PrivateAction>
</Private></Actions></Init></Storyboard></OpenSCENARIO>
"""
            )

            poses, _ = parse_initial_state(xosc, {}, mgeo_dir)

            self.assertEqual((poses["ego"].x, poses["ego"].y, poses["ego"].z), (15.0, 20.0, 1.0))

    def test_gui_generation_enforces_external_ego_and_preserves_preset(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = root / "source.xosc"
            target = root / "generated" / "Project" / "source.xosc"
            target.parent.mkdir(parents=True)
            source.write_text(
                """\
<OpenSCENARIO>
  <VariableDeclarations>
    <VariableDeclaration name="moraiMap" value="Hwaseong-si" />
  </VariableDeclarations>
  <RoadNetwork><LogicFile filepath="MGeo" /></RoadNetwork>
  <Entities>
    <ScenarioObject name="Ego">
      <Vehicle name="Ego" model3d="HD Ioniq5" role="none">
        <Properties><Property name="MotionModel" value="Kinematic" /></Properties>
      </Vehicle>
      <ObjectController>
        <Controller controllerType="all" name="Preset">
          <Properties><Property name="name" value="Off" /></Properties>
        </Controller>
      </ObjectController>
    </ScenarioObject>
    <ScenarioObject name="NPC_1">
      <Vehicle name="NPC_1" model3d="HD Ioniq6" role="none">
        <Properties>
          <Property name="MotionModel" value="Kinematic" />
          <Property name="custom" value="keep" />
        </Properties>
      </Vehicle>
      <ObjectController>
        <Controller controllerType="lateral" name="Lateral">
          <Properties><Property name="model" value="Mobil" /></Properties>
        </Controller>
      </ObjectController>
    </ScenarioObject>
  </Entities>
</OpenSCENARIO>
""",
                encoding="utf-8",
            )
            info = SimpleNamespace(
                path=source,
                target_xosc_path=target,
                target_project_path=target.parent / "Project.mproj",
                source_project_map=None,
                morai_map="Hwaseong-si",
                runner_map_name="KATRI",
                logic_file="MGeo",
                ego_entity="Ego",
                parameters={},
            )

            write_prepared_xosc(info, headless_control=False, mgeo_mode="copy")

            generated = ET.parse(target).getroot()
            objects = {item.get("name"): item for item in generated.iter("ScenarioObject")}
            ego = next(child for child in objects["Ego"] if child.tag == "Vehicle")
            npc = next(child for child in objects["NPC_1"] if child.tag == "Vehicle")
            ego_properties = {
                prop.get("name"): prop.get("value") for prop in ego.iter("Property")
            }
            npc_properties = {
                prop.get("name"): prop.get("value") for prop in npc.iter("Property")
            }

            self.assertEqual(ego_properties["MotionModel"], "Dynamics")
            self.assertEqual(ego_properties["external_control"], "true")
            self.assertEqual(ego_properties["AgentType"], "Ego")
            self.assertEqual(
                [
                    prop.get("value")
                    for controller in objects["Ego"].iter("Controller")
                    if controller.get("name") == "Preset"
                    for prop in controller.iter("Property")
                    if prop.get("name") == "name"
                ],
                ["Off"],
            )

            self.assertEqual(npc.get("name"), "NPC_1")
            self.assertEqual(npc.get("role"), "none")
            self.assertEqual(
                npc_properties,
                {"MotionModel": "Kinematic", "custom": "keep"},
            )
            self.assertEqual(
                [controller.get("name") for controller in objects["NPC_1"].iter("Controller")],
                ["Lateral"],
            )


if __name__ == "__main__":
    unittest.main()
