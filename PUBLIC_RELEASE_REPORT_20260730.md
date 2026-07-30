# Public release preparation report — 2026-07-30

## Scope

- Base: `origin/main` commit `44759df`
- Local preparation branch: `public/hwjung/0730-github-release`
- No commit, push, GitLab/GitHub merge request, or remote change was performed.

## MGeo separation

- Removed 19 tracked KATRI MGeo data files (44,659,915 bytes) from the public
  working tree.
- Added `data/morai/mgeo/` to `.gitignore` to prevent accidental re-addition.
- The controller, environment checker, and suite preparation commands now
  accept an authorized external MGeo directory.
- PDM control exits before route planning when `--mgeo-dir` is missing or does
  not contain `link_set.json`.

The removal only affects the current public tree. The old internal Git history
still contains the data objects. Do not mirror that history to a public GitHub
repository. Publish from a clean-history/squashed export or perform a separately
approved history rewrite before external release.

## Route calculation benchmark

The bundled and external paths referenced byte-identical `link_set.json` files.
Both paths were on the same local storage. Each test alternated the two paths to
reduce filesystem-cache ordering bias.

| Trial | Path | Samples | Median MGeo load | Median 350 m route build |
|---|---|---:|---:|---:|
| 1 | repository path | 7 | 158.492 ms | 50.381 ms |
| 1 | external path | 7 | 168.026 ms | 50.485 ms |
| 2, reversed order | external path | 15 | 158.910 ms | 51.168 ms |
| 2, reversed order | repository path | 15 | 169.239 ms | 50.153 ms |

Average of the two trial medians:

- MGeo load: repository 163.866 ms, external 163.468 ms.
- Route build: repository 50.267 ms, external 50.827 ms.
- External-path route-build difference: approximately +0.560 ms (+1.1%).

Conclusion: there is no meaningful performance regression from supplying the
same MGeo data through an external local path. JSON loading and route generation
dominate; the directory location itself does not. Network drives and slow
removable storage were not tested and may have different load times.

Reproduce with `tools/benchmark_mgeo_route.py`.

## Runtime configuration

- Added `--config <json>` to the existing PowerShell launcher path.
- Added `configs/pdm_morai.example.json` with the commonly used GUI TCP
  lockstep settings.
- Command-line values override JSON config values, preserving one-off testing.
- `mgeo_dir` remains an explicit local-machine value in the config.

Typical run:

```powershell
Copy-Item .\configs\pdm_morai.example.json .\configs\pdm_morai.local.json
# Edit mgeo_dir in pdm_morai.local.json.
.\tools\run_pdm_morai_control.ps1 --config .\configs\pdm_morai.local.json
```

## Verification

- Unit tests after MGeo deletion: 82 passed.
- Environment check passed with an external MGeo directory.
- External-only MGeo benchmark completed after bundled-data deletion.
- New package tree contains no `data/morai/mgeo/KATRI` directory.

## Approval gates before GitHub publication

1. Select the outbound license or usage terms for MORAI-authored source.
2. Confirm the approved channel and audience for separately supplied MGeo data.
3. Create the GitHub repository from a clean tree without importing the internal
   Git history that contains deleted MGeo blobs.
4. Review the final diff, then explicitly approve commit/push/MR execution.
