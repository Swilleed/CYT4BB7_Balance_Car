# CYT4BB 平衡小车项目

本工程是基于英飞凌 Traveo II 系列 **CYT4BB7** 芯片开发的平衡小车控制系统。项目采用了逐飞科技提供的开源库进行开发，旨在实现高效率、高稳定性的平衡控制。

## 🛠️ 开发环境

- **IDE**: IAR For ARM (建议版本 9.40.1 及以上)
- **硬件**: CYT4BB7 核心板及平衡小车底盘
- **库版本**: 逐飞科技 CYT4BB 开源库

## 📂 项目结构

```text
.
├── libraries/          # 逐飞开源库主体
│   ├── sdk/            # 芯片底层 SDK
│   ├── zf_common/      # 通用功能库
│   ├── zf_driver/      # 外设驱动库
│   └── zf_device/      # 模块驱动 (如传感器、显示屏等)
├── project/
│   ├── code/           # 核心应用代码
│   │   ├── Algorithm/  # 控制算法 (PID、姿态解算等)
│   │   ├── Apps/       # 应用逻辑
│   │   └── Drivers/    # 硬件抽象层
│   ├── iar/            # IAR 工程文件
│   └── user/           # 芯片入口与中断向量 (main_cm7_0/1)
└── README.md
```

## 🚀 快速上手

1. **打开工程**: 进入 `project/iar/` 目录，双击 `cyt4bb7.eww` 文件使用 IAR 打开。
2. **选择核心**: 本芯片为双核架构，建议根据功能需求在 `main_cm7_0.c` 或 `main_cm7_1.c` 中编写逻辑。
3. **编译与烧录**: 点击 IAR 的 `Make` 进行全工程编译，通过调试器烧录至核心板。

## 📚 相关资料

- **代码仓库**: [逐飞开源库 Gitee 仓库](https://gitee.com/seekfree/CYT4BB7_Library)
- **官方资料**: [逐飞资料页](https://www.seekfree.com.cn/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/%E6%A0%B8%E5%BF%83%E6%9D%BF/%E8%8B%B1%E9%A3%9E%E5%87%8C%E7%B3%BB%E5%88%97/cyt4bb7%E6%A0%B8%E5%BF%83%E6%9D%BF/) (含 IAR 及注册机等)
- **开发文档**: [CYT4BB7 开发资料汇总](https://gitee.com/seekfree/CYT2BL3_Library/raw/master/%E3%80%90%E6%96%87%E6%A1%A3%E3%80%91%E8%AF%B4%E6%98%8E%E4%B9%A6%20%E8%8A%AF%E7%89%87%E6%89%8B%E5%86%8C%E7%AD%89/%E6%A0%B8%E5%BF%83%E6%9D%BF%E6%96%87%E6%A1%A3/CYT4BB7%E5%BC%80%E5%8F%91%E8%B5%84%E6%96%99%E6%B1%87%E6%80%BB%E2%80%94%E2%80%94%E9%80%90%E9%A3%9E%E7%A7%91%E6%8A%80(CYT2BL3%E5%8F%AF%E5%8F%82%E8%80%83).pdf)

## ⚖️ 许可协议

本项目库文件部分遵循 **GPL-3.0** 开源协议，详情请参阅 [libraries/doc/GPL3_permission_statement.txt](libraries/doc/GPL3_permission_statement.txt)。
