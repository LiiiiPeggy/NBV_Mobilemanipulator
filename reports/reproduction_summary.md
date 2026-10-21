# 复现总结

日期：2026-10-21  
根目录：`/home/ubuntu/locomani/NBV_Mobilemanipulator`

## 总览

| 目标 | 结果 |
|------|------|
| 宿主 22.04 不动、容器 20.04/Noetic | 满足 |
| FKIE / SSMI 独立 Docker | 满足（`nbv_fkie` / `nbv_ssmi`） |
| `/workspace` 挂载与持久化 | 已验证 |
| GPU + 图形（无 xhost +） | 已验证（A6000 GL / Ignition / Gazebo GUI） |
| FKIE 编译 + Action 调用 | 通过（完整探索闭环待平台栈） |
| SSMI 官方示例：相机 + 语义图 + 规划话题 | 通过 |

## 镜像

| 镜像 | 基础 | 备注 |
|------|------|------|
| `nbv-fkie:noetic` | osrf/ros:noetic-desktop-full | Gazebo Classic 11 |
| `nbv-ssmi:noetic` | 同上 + ignition-fortress + gflags | Fortress 6.17.1 |

未使用、未停止他人容器。

## 关键命令

```bash
cd /home/ubuntu/locomani/NBV_Mobilemanipulator
source scripts/setup_x11.sh
./scripts/build_fkie.sh && ./scripts/start_fkie.sh
./scripts/build_ssmi.sh && ./scripts/start_ssmi.sh   # 勿与 FKIE 完整仿真并行
```

## 证据索引

- `reports/host_environment.md`
- `reports/fkie_reproduction.md` + `logs/fkie_smoke_run.log`
- `reports/ssmi_reproduction.md` + `logs/ssmi_smoke_run.log` + `logs/ssmi_glxinfo.txt`

## 待办

1. FKIE 完整机器人仿真与非空 NBV goals  
2. REMANI / TopAY 后端接入  
3. 4070 Laptop 同 Compose 复用  
