# CYT4BB 无圆环基线备份

本分支用于保存 `F:\CYT4BB7\CYT4BB - 副本` 的无圆环基线工程，方便后续圆环开发失败时回溯。

## 用途

- 保存没有圆环元素干扰的稳定基线。
- 后续圆环开发以当前 `F:\CYT4BB7\CYT4BB` 为工作目录，本备份只用于对照和恢复。
- 不要直接把旧圆环失败版本整套覆盖回来。

## 说明

- IAR 编译输出、BrowseInfo、Obj、Exe 等生成文件已通过 `.gitignore` 排除。
- 保留源码、工程配置、库文件、文档和参考图片。
- 当前对应 GitHub 分支建议名：`baseline-no-ring-copy`。

## 恢复建议

如需恢复关键无圆环文件，优先对照这些文件：

```text
project\code\SearchLine.c
project\code\ServoPID.c
project\code\ServoPID.h
project\code\TrackTypes.h
project\user\main_cm7_0.c
```

恢复前请先备份当前工作目录，避免覆盖正在调试的代码。
