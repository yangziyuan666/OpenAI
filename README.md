# 智能车调试交流仓库

这个目录用于在“网页 ChatGPT 长上下文分析”和“本地 Codex 修改工程”之间传递资料。

核心目标：

```text
串口原始日志放文件
网页 ChatGPT 负责长日志分析
分析结论落成 markdown 文件
Codex 读取分析结论后修改 F:\CYT4BB7\CYT4BB 工程
```

这样可以避免把大量串口日志直接塞进聊天上下文，导致上下文很快被日志占满。

## 推荐结构

```text
交流仓库/
  README.md
  rounds/                 每一轮调试一个子目录
  templates/              提交日志、请求分析、请求改代码的模板
  scripts/                日志清洗/分块脚本
```

每一轮建议这样命名：

```text
rounds/20260428_左环阶段1_特征观察/
  00_issue.md
  01_raw_uart.txt
  02_obs_clean.txt
  03_obs.csv
  04_prompt_for_chatgpt.md
  05_chatgpt_analysis.md
  06_task_for_codex.md
  07_codex_result.md
```

## 一轮调试流程

1. 把串口原始日志保存成：

```text
rounds/日期_问题名/01_raw_uart.txt
```

2. 运行清洗脚本：

```powershell
powershell -ExecutionPolicy Bypass -File "F:\CYT4BB7\CYT4BB\交流仓库\scripts\split_obs_log.ps1" -InputPath "F:\CYT4BB7\CYT4BB\交流仓库\rounds\日期_问题名\01_raw_uart.txt"
```

脚本会生成：

```text
02_obs_clean.txt
03_obs.csv
chunks/
```

3. 把这一轮目录提交到 GitHub。

4. 网页 ChatGPT 分析时，给它：

```text
04_prompt_for_chatgpt.md
01_raw_uart.txt 或 02_obs_clean.txt
03_obs.csv
```

如果 GitHub 是公开仓库，可以给网页链接；如果是私有仓库，建议直接上传文件或使用 GitHub 连接器。

5. 把网页 ChatGPT 的分析结论保存到：

```text
05_chatgpt_analysis.md
```

6. 把需要 Codex 改代码的要求保存到：

```text
06_task_for_codex.md
```

7. 让本地 Codex 读取 `06_task_for_codex.md` 和 `05_chatgpt_analysis.md`，然后修改工程、构建验证。

## GitHub 使用建议

建议建一个单独的私有仓库，例如：

```text
smartcar-debug-exchange
```

不要直接把完整工程传上去。这个交流仓库只放：

```text
串口日志
分析结论
任务说明
关键截图
```

注意：

- GitHub 单文件超过 100 MB 会比较麻烦。
- 如果日志很大，先用脚本分块，优先上传 `02_obs_clean.txt` 和 `03_obs.csv`。
- 公开仓库不要放隐私路径、账号、比赛敏感策略。

## 本地初始化 Git

如果还没初始化，可以在本目录执行：

```powershell
cd "F:\CYT4BB7\CYT4BB\交流仓库"
git init
git add .
git commit -m "init exchange workflow"
```

你在 GitHub 新建空仓库后，再执行：

```powershell
git remote add origin https://github.com/你的用户名/smartcar-debug-exchange.git
git branch -M main
git push -u origin main
```

如果你使用 GitHub Desktop，也可以直接把这个目录作为本地仓库发布。

## 与圆环任务书的关系

圆环开发的总任务书在：

```text
F:\CYT4BB7\CYT4BB\任务.md
```

本交流仓库只负责一轮一轮传递日志和分析。真正修改工程前，Codex 必须同时参考：

```text
F:\CYT4BB7\CYT4BB\任务.md
交流仓库/rounds/某一轮/05_chatgpt_analysis.md
交流仓库/rounds/某一轮/06_task_for_codex.md
```
