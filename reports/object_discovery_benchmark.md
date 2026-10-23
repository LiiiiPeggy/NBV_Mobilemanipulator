# Object discovery benchmark

Date: 2026-10-23
Commit: `156f533`
Seed: 42

## Formal @0.15 m (multiview accumulate)

| Metric | Value |
|--------|------:|
| n_gt | 11 |
| TP/FP/FN | 5/25/6 |
| Precision | 0.167 |
| Unique recall | 0.455 |
| Mean err (TP) | 0.045157415286428804 |

## Memory

| set | TP | FP | FN | Prec | Recall |
|-----|---:|---:|---:|-----:|-------:|
| all | 5 | 6 | 6 | 0.455 | 0.455 |
| confirmed | 4 | 4 | 7 | 0.500 | 0.364 |

Artifacts: `reports/object_discovery_benchmark/`, `reports/multiview_ycb/seed42/`.
