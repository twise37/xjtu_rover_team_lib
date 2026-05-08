# 西安交通大学机器人团队资源库

> 面向机器人研发协作的团队公共资源库，集中管理主控程序、开发板例程、机械模型与工程文档。

![Repository](https://img.shields.io/badge/repository-team--library-2f6fed)
![Status](https://img.shields.io/badge/status-active-28a745)
![Language](https://img.shields.io/badge/language-C%20%2F%20C%2B%2B-orange)
![License](https://img.shields.io/badge/license-internal-lightgrey)

## 项目概览

本仓库用于沉淀西安交通大学机器人团队在机器人研发过程中的通用资产，帮助团队成员快速复用已有成果、统一协作方式，并降低新成员上手成本。

主要内容包括：

| 模块 | 说明 |
| --- | --- |
| `mainboard/` | 机器人主板相关驱动、通信协议与核心控制逻辑 |
| `development/` | 各类开发板例程、验证代码与功能模块 |
| `models/` | 机械结构、零件、装配体等 3D 模型文件 |
| `docs/` | 设计说明、接口文档、开发规范与技术沉淀 |

## 目录结构

```text
xjtu_rover_team_lib/
├── README.md              # 仓库说明与快速入口
├── CONTRIBUTING.md        # 协作与提交规范
├── .gitignore             # 常见临时文件与构建产物忽略规则
├── .editorconfig          # 编辑器基础格式约定
├── .gitattributes         # Git 文本与二进制文件处理规则
├── mainboard/             # 主板代码
├── development/           # 开发板代码
├── models/                # 3D 模型文件
└── docs/                  # 开发文档
```

## 快速开始

### 克隆仓库

```bash
git clone https://github.com/laihanwen/xjtu_rover_team_lib.git
cd xjtu_rover_team_lib
```

### 创建开发分支

建议每个功能、修复或实验使用独立分支：

```bash
git checkout -b your-name/feature-name
```

分支命名建议：

- `name/feature-xxx`：新增功能或模块
- `name/fix-xxx`：问题修复
- `name/docs-xxx`：文档补充
- `name/experiment-xxx`：实验性验证

## 协作流程

1. 从最新主分支创建个人开发分支。
2. 在对应目录中补充代码、模型或文档。
3. 提交前检查文件命名、目录归属和 README 是否需要同步更新。
4. 使用清晰的 commit 信息说明本次变更。
5. 通过 Pull Request 合并到主分支，并在 PR 中写明变更内容、测试方式和注意事项。

更多细节请参考 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 文档约定

- 新增模块时，请在对应目录添加或更新 `README.md`。
- 接口、协议、引脚定义、依赖版本等信息应写入 `docs/`，避免只存在于聊天记录或个人笔记中。
- 机械模型建议补充导出格式、单位、适配版本和装配说明。
- 实验性代码请标注适用硬件、接线方式和验证状态。

## 当前待补充

- 主控板具体型号、芯片平台与开发环境。
- 开发板清单与对应例程说明。
- 通信协议、传感器接口、控制算法的设计文档。
- 机械模型的版本、单位、坐标系和装配关系。

## 联系方式

- 团队邮箱：2777651780@qq.com

---

维护建议：当目录结构、硬件平台或协作流程发生变化时，请同步更新本 README。
