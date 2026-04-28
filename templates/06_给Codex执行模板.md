# 给 Codex 的执行任务

请先阅读：

```text
F:\CYT4BB7\CYT4BB\任务.md
本轮 00_issue.md
本轮 05_chatgpt_analysis.md
```

## 当前阶段

```text
填写阶段编号和名称
```

## 允许修改范围

例如：

```text
只允许修改 SearchLine.c 和 main_cm7_0.c，用于新增圆环观察日志。
不允许修改 ServoPID.c/.h。
不允许改变 edge->mid_x。
不允许补线。
不允许改速度。
```

## 本轮要完成

```text
填写明确任务，例如：
根据 ChatGPT 分析，调整左环候选观察字段，让普通左弯不连续触发 LC，但左环 1 点和 2 点仍能触发。
```

## 验收标准

```text
IAR 构建 0 error / 0 warning。
串口格式说明清楚。
不破坏无圆环基线。
```

## 禁止事项

```text
不要整目录复制旧圆环备份。
不要一次同时改识别、状态机、补线、舵机、速度。
不要把圆环控制逻辑接入 ServoPID，除非当前阶段明确允许。
```
