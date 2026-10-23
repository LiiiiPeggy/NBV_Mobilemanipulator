# Grounded-SAM per-object failure analysis (seed42)

Date: 2026-10-23  
Layout: clear-aisle table `(-3.90,-7.50)` after Stage 0 placement fix  
Detector: NBV-vendored Grounding DINO + SAM2 (PyTorch fallback; `_C` CUDA ops not required)  
Thresholds: box/text 0.25 (unchanged); eval dist **0.15 m**

## Aggregate @0.15 m

| Metric | Value |
|--------|-------|
| 2D detections | 10 |
| Localized 3D | 10 |
| TP / FP / FN | **4 / 6 / 7** |
| Precision | 0.40 |
| Unique recall | **0.364** |
| Mean err (TP only) | **0.043 m** |

Previous FOV layout reported TP=2 / recall≈0.18; new placement improved TP without lowering thresholds.

## Failure-type counts (A–H)

| Type | Count | Meaning |
|------|------:|---------|
| TP | 4 | Matched |
| B_gdino_miss | 4 | Visible but no compatible GDINO box |
| F_localization_error | 3 | Class box exists but 3D err > 0.15 m |
| H_false_positive | 6 | Extra preds |
| A / C / D / E / G | 0 | Not dominant this frame |

## Per GT object

| ID | Class | Vis | Result | Failure | err_m |
|----|-------|-----|--------|---------|-------|
| obj_00 | banana | Y | TP | — | 0.040 |
| obj_01 | can | Y | TP | — | 0.092 |
| obj_02 | bottle | Y | TP | — | 0.027 |
| obj_03 | bowl | Y | FN | B | — |
| obj_04 | box | Y | FN | F | 0.748 |
| obj_05 | box | Y | FN | F | 0.624 |
| obj_06 | apple | Y | TP | — | 0.015 |
| obj_07 | box | Y | FN | B | — |
| obj_08 | bottle | Y | FN | B | — |
| obj_09 | bottle | Y | FN | F | 0.841 |
| obj_10 | cone | Y | FN | B | — |

## Interpretation

1. **TP mean_err≈0.043 m** only describes the 4 matched objects — not global localization quality.
2. Main FN modes: **GDINO miss** (bowl / cracker / floor mustard / cone) and **3D association/localization** on ambiguous `box`/`bottle` (multiple instances, FP boxes).
3. Frozen class aliases live in `scripts/analyze_gsam_failures.py` (`CLASS_ALIASES`); not retuned on this test set beyond documenting synonyms.

## Artifacts

- RGB / depth / cam TF: `reports/gsam_analysis/`
- 2D pred + vis: `pred2d_seed42.json`, `pred2d_seed42.vis.png`
- 3D pred: `pred3d_seed42.json`
- Eval: `eval_seed42.json`
- Failure table + vis: `failure_table_seed42.json`, `failure_table_seed42.vis.png`
