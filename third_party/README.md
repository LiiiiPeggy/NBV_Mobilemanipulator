# third_party

本地克隆官方依赖（各自保留独立 `.git`，不作为本仓库子树提交大文件）。

销定版本（2026-10-21 验证）：

| 目录 | 仓库 | Commit |
|------|------|--------|
| fkie-nbv-planner | https://github.com/fkie/fkie-nbv-planner | 91091f8a58301a69bbf240bce073059092e6f606 |
| fkie_ddynamic_reconfigure | https://github.com/fkie/fkie_ddynamic_reconfigure | 7613e5319d815741fedc762cb39d19b3cc3cb6af |
| fkie_environmental_measurements | https://github.com/fkie/fkie_environmental_measurements | 75d331770c4c43fc0247876a192e5dd6809cd759 |
| SSMI-Example | https://github.com/ExistentialRobotics/SSMI-example | 750122b8dbb193dcf67af5429be6c8df3b3b462e |
| agx | https://github.com/LiiiiPeggy/agx | clone tip（整机 URDF；gitignore） |

克隆：

```bash
./scripts/clone_third_party.sh
```
