# GitHub Workflows

此目录包含 Agilebot Isaac Sim 项目的 GitHub Actions 工作流文件。

## 工作流

### sync-changelog.yml

**目的**：自动将 `CHANGELOG.md` 或 `CHANGELOG_CN.md` 中的计划功能同步到 `README.md` 和 `README_cn.md` 文件。

**触发方式**：
- 自动：当 `CHANGELOG.md` 或 `CHANGELOG_CN.md` 被推送到 `main` 或 `master` 分支时
- 手动：通过 GitHub Actions 的 `workflow_dispatch` 事件

**工作原理**：
1. 在 Ubuntu Latest 上运行
2. 检出仓库
3. 设置 Python 3.11 环境
4. 执行同步脚本：`.github/scripts/sync_changelog.py`
   - 从 CHANGELOG.md（英文）或 CHANGELOG_CN.md（中文）读取
5. 显示更改的文件
6. 如果有更改则提交（跳过 CI 避免循环触发）

**为什么跳过 CI**：提交使用 `[skip ci]` 标记以避免工作流递归触发。

---

## 脚本

### sync_changelog.py

位于 `.github/scripts/` 目录中。

**目的**：读取 `CHANGELOG.md` 或 `CHANGELOG_CN.md` 中的计划功能部分，并更新英文和中文 README 文件。

**功能特性**：
- ✅ 从 CHANGELOG.md 或 CHANGELOG_CN.md 提取计划功能
- ✅ 为 README.md 生成英文版本
- ✅ 为 README_cn.md 生成中文版本
- ✅ 保留现有格式
- ✅ 提供详细的进度和错误消息
- ✅ 正确处理 UTF-8 编码
- ✅ 支持双语的 CHANGELOG（英文和中文）

---

## 使用方法

### 手动触发

要手动触发工作流：

1. 转到 GitHub 仓库页面
2. 导航到 **Actions** 标签页
3. 选择 **Sync Changelog to README** 工作流
4. 点击 **Run workflow** 按钮
5. （可选）如果需要，选择分支

### 自动触发

只需将更改推送到 CHANGELOG 文件：

```bash
# 更新英文版本
git add CHANGELOG.md
git commit -m "feat: add new planned features"
git push

# 或更新中文版本
git add CHANGELOG_CN.md
git commit -m "feat: 添加新的计划功能"
git push
```

工作流将自动：
- 检测 CHANGELOG.md 或 CHANGELOG_CN.md 的更改
- 运行同步脚本
- 更新两个 README 文件
- 提交更改

---

## 文件结构

```
.github/
├── workflows/
│   ├── sync-changelog.yml      # GitHub Action 工作流定义
│   ├── README.md             # 英文说明（本文件）
│   └── README_CN.md          # 中文说明
└── scripts/
    └── sync_changelog.py         # 同步脚本
```

---

## 故障排除

### 工作流未触发

**检查**：
- 是否推送到 `main` 或 `master` 分支？
- `CHANGELOG.md` 文件是否真的更改了？
- 工作流文件是否是有效的 YAML？

### 工作流因权限错误失败

**检查**：
- 仓库已启用 Actions
- 工作流文件有正确的权限
- Token 有写入访问权限

### 脚本错误

**检查**：
- CHANGELOG.md 具有正确的 `### Planned` 部分格式
- 仓库同时存在 README.md 和 README_cn.md
- Python 版本是 3.11（必需）

---

## 最佳实践

1. **保持 CHANGELOG.md 更新**：始终在更改之前先添加计划功能
2. **审查自动提交**：工作流会自动提交，合并前请审查
3. **遵循 changelog 格式**：使用标准的 CHANGELOG.md 格式
4. **先本地测试**：如果要测试更改，可以本地运行脚本

---

## 相关资源

- [GitHub Actions 文档](https://docs.github.com/zh/actions)
- [工作流语法](https://docs.github.com/zh/actions/using-workflows/workflow-syntax-for-github-actions)
- [CHANGELOG.md](../CHANGELOG.md) - 主 changelog 文件
