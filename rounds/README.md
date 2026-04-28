# 调试轮次目录

每一轮测试新建一个子目录，例如：

```text
20260428_左环阶段1_特征观察
20260428_普通弯误触发排查
20260429_左环阶段3_状态机验收
```

每一轮目录建议包含：

```text
00_issue.md
01_raw_uart.txt
02_obs_clean.txt
03_obs.csv
04_prompt_for_chatgpt.md
05_chatgpt_analysis.md
06_task_for_codex.md
07_codex_result.md
```

不要把所有问题混在一个目录里。一次只分析一个阶段、一个现象。
