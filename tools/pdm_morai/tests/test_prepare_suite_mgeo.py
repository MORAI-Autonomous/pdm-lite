from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


TOOLS_DIR = Path(__file__).resolve().parents[2]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pdm_morai_prepare_suite import (
    PrepareError,
    copy_mgeo_sidecar,
    output_logic_file,
    remove_project_tree,
)


class MGeoSidecarTest(unittest.TestCase):
    def test_reference_mode_does_not_delete_external_logic_path(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            project_dir = root / "data" / "generated-suite" / "case" / "Project"
            project_dir.mkdir(parents=True)
            source = root / "data" / "morai" / "mgeo" / "KATRI"
            source.mkdir(parents=True)
            (source / "link_set.json").write_text("{}", encoding="utf-8")
            info = SimpleNamespace(
                logic_file="../../../morai/mgeo/KATRI",
                runner_map_name="KATRI",
                in_place=False,
            )

            with patch("pdm_morai_prepare_suite.resolve_mgeo_source", return_value=source):
                copy_mgeo_sidecar(info, project_dir, "reference")

            self.assertTrue(source.exists())
            self.assertTrue((source / "link_set.json").exists())

    def test_copy_mode_normalizes_external_logic_path_to_project_mgeo(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            project_dir = root / "generated" / "Project"
            project_dir.mkdir(parents=True)
            source = root / "source" / "MGeo"
            source.mkdir(parents=True)
            (source / "link_set.json").write_text("{}", encoding="utf-8")
            info = SimpleNamespace(
                logic_file="../../../morai/mgeo/KATRI",
                runner_map_name="KATRI",
                in_place=False,
            )

            with patch("pdm_morai_prepare_suite.resolve_mgeo_source", return_value=source):
                self.assertEqual(output_logic_file(info, project_dir, "copy"), "MGeo")
                copy_mgeo_sidecar(info, project_dir, "copy")

            self.assertTrue((project_dir / "MGeo" / "link_set.json").exists())
            self.assertTrue(source.exists())

    def test_remove_project_tree_rejects_external_path(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            project_dir = root / "generated" / "Project"
            project_dir.mkdir(parents=True)
            external = root / "data" / "morai" / "mgeo" / "KATRI"
            external.mkdir(parents=True)

            with self.assertRaises(PrepareError):
                remove_project_tree(external, project_dir)

            self.assertTrue(external.exists())


if __name__ == "__main__":
    unittest.main()
