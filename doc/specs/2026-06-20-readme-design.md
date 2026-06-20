# Design Spec: Bilingual README for GazeboLib

*   **Date**: 2026-06-20
*   **Status**: Draft
*   **Topic**: Adding bilingual `README.md` (Chinese) and `README_EN.md` (English) to the project root.

---

## 1. Overview & Goals

The goal of this task is to establish clear, professional documentation at the root of the `GazeboLib` repository. We want to welcome developers and operators to this ROS 2 / Gazebo-based Multi-AGV Warehouse Simulation project with two high-quality README files:
1.  **`README.md`**: Main Chinese document.
2.  **`README_EN.md`**: Main English document.

Both documents will feature language selector links at the top to allow seamless switching between them.

---

## 2. Document Structure

### 2.1 README.md (简体中文)
- **项目标题**: 多 AGV 仓库仿真系统
- **语言切换**: `[English Version](./README_EN.md) | 简体中文`
- **项目简介**: 简要说明系统是基于 ROS 2 Humble 和 Gazebo Classic 11 构建的多车仓储物流调度与路权协作仿真项目。
- **核心功能**:
  - 工业级 Nav2 导航与避障。
  - 双重导航驱动切换（支持基于 Gazebo 真值的直线 P 控制轻量级回退导航）。
  - 路权分配与自动避碰算法（让行逻辑）。
  - 基于 YOLOv8 的视觉目标识别（`amr_vision`）。
  - 基于 Rosbridge 的可视化 Web 操作面板（`amr_web`）。
- **目录结构**: 包含 `src/` 与 `doc/` 等关键目录与文件树。
- **快速开始**:
  - 系统依赖说明（系统、ROS 2 Humble、Gazebo Classic 11、Nav2、rosbridge）。
  - 安装步骤。
  - 编译指令（必须使用顺序编译：`colcon build --symlink-install --executor sequential`）。
  - 启动系统指令（运行工作空间环境、推荐一键启动脚本等）。
  - Web 操作面板的使用方法。
- **文档指南**: 链接至 `doc/` 目录下的深度专题设计文档。

### 2.2 README_EN.md (English)
- **Project Title**: Multi-AGV Warehouse Simulation System
- **Language Selector**: `English | [简体中文](./README.md)`
- **Project Overview**: A high-level description of the ROS 2 Humble + Gazebo Classic 11 simulation system.
- **Key Features**:
  - Industrial-grade Nav2 navigation and obstacle avoidance.
  - Dual-mode navigation driver switching (with ground truth waypoint following fallback via proportional control).
  - Right-of-Way (RoW) conflict resolution and multi-robot collision avoidance.
  - YOLOv8-based target detection and visual tracking (`amr_vision`).
  - Web control interface via Rosbridge (`amr_web`).
- **Directory Structure**: Workspace folders breakdown.
- **Quick Start**:
  - Dependencies (Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, etc.).
  - Installation.
  - Sequential compilation instructions.
  - Running instructions (一键启动/One-key execution command).
  - Connecting the Web Dashboard interface.
- **Detailed Documentation Links**: Direct references to details inside `doc/`.

---

## 3. Reference Material / Existing Documents

The README files will aggregate and reference details from the following documents:
- **`doc/operations-guide.md`**: Outlines build steps, setup, and launch commands.
- **`doc/navigation-modes-and-fallback.md`**: Explains double-driver fallback (Nav2 vs Ground Truth).
- **`doc/right-of-way-design.md`**: Documents right-of-way resolution rules and priorities.
- **`doc/mapping-guide.md`**: Details 2D SLAM and map optimization.

---

## 4. Implementation Steps

1.  Create `README.md` (Chinese) in `/home/li/GazeboLib/README.md`.
2.  Create `README_EN.md` (English) in `/home/li/GazeboLib/README_EN.md`.
3.  Ensure all relative links to source packages (`src/amr_description`, `src/amr_vision`, `src/amr_web`) and documentation are clickable and correct.
4.  Perform verification of syntax, links, and code snippets before finishing.
