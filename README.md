# SYT 人机交互界面
## Release Note
### Summary
#### 主要功能
syt_hmi 集成客户交付界面，测试及开发人员的调试界面，便于可视化的进行系统的管理。

1. 支持Log监控，检测图像监控。
2. 补料模式。
3. 创建样式。

#### 已知bug
1. 有概率闪退。
#### 注意事项
1. 未集成ota更新，不要点ota更新功能。

### Change Log
- v1.0.5:
    - 增加样式预览功能。
    - 更新界面布局。
    - 隐藏部分功能。
- v1.0.4:
    - 增加从已有样式创建的功能。
    - 修复若干问题。
- v1.0.3:
    - 增加更新本地固件功能。
    - 通过配置文件读取限制值。
    - 更新流程控制接口。
    - 修复若干问题。
- v1.0.2:
    - 增加选运行模式功能。
    - 按钮按照运行逻辑互斥。
    - 交互上增加的一些内容（提醒裁片放置位置、创建样式时提交后不允许回退更改等）。
- v1.0.1:
    - 增加开发者调试窗口。
    - 设置log级别过滤。
    - 开放停止、开始、复位等按钮。
    - 增加删除样式功能。
    - 完善一些逻辑。
- v1.0.0:
    - 增加手动创建样式，导入样式功能。
    - 流程控制按键功能。
    - 补料模式按钮。
- v0.0.9: 样式创建和选择，查看log，监控视觉图像功能。
