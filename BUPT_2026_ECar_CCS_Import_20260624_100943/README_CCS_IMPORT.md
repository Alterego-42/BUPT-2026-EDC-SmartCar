# CCS 导入说明

这是 LP-MSPM0G3507 的 TI clang / SysConfig 工程包，可直接导入 Code Composer Studio。

## 导入方式

1. 打开 CCS。
2. 选择 `File -> Import...`。
3. 选择 `Code Composer Studio -> CCS Projects` 或 `Existing CCS/CCE Eclipse Projects`。
4. Root directory 选择本文件夹。
5. 选择 `BUPT_2026_ECar_LP_MSPM0G3507_nortos_ticlang`。
6. 勾选复制到 workspace 或保持当前位置均可。
7. 点击 Finish 后 Build。

如果 CCS 未自动识别工程，可选择通过 `BUPT_2026_ECar.projectspec` 导入。

## 文件说明

- `src/main.c`：主循环，按键按住时长选择任务。
- `src/tasks.c`：6 个正式任务和 3 个调试任务。
- `src/board_pins.h`：全部引脚和主要参数。
- `BUPT_2026_ECar.syscfg`：PWM/SysConfig 配置。
- `README_TASKS.md`：任务档位和调参说明。
- `ticlang/makefile`：命令行验证用，CCS 导入不依赖它。

## 任务档位

按键每 `700 ms` 一档，松手启动：

1. 自动巡迹
2. 定点瞄准
3. 巡迹到靶位联动
4. 动态瞄准
5. 同步绘制
6. 四圈连续运行
7. 模块自检
8. 传感器看板
9. 直线测试
