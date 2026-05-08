# 3D 模型

本目录用于存放机器人的机械结构、零件模型、装配文件和可制造导出文件。请尽量保留源文件与通用交换格式，方便不同软件环境下查看和加工。

## 支持格式

| 格式 | 用途 |
| --- | --- |
| `.step` / `.stp` | 通用 3D 交换格式，推荐用于跨软件协作 |
| `.stl` | 3D 打印、快速预览和部分仿真场景 |
| `.sldprt` / `.sldasm` | SolidWorks 零件与装配文件 |
| `.iges` / `.igs` | IGES 通用交换格式 |

## 推荐目录组织

```text
models/
├── README.md
├── assemblies/            # 装配体
├── parts/                 # 零件模型
├── exports/               # STEP、STL 等导出文件
└── drawings/              # 工程图或加工图纸
```

## 文件命名建议

建议使用可读、稳定的英文或拼音命名：

```text
module_part_version.format
chassis_side_plate_v1.step
arm_joint_mount_v2.sldprt
```

## 提交注意事项

- 请标注模型单位、坐标系和适配版本。
- 修改装配关系后，请同步导出通用格式。
- 大型二进制文件提交前请确认是否确实需要纳入仓库。
- 临时备份、软件缓存和渲染中间文件不应提交。
