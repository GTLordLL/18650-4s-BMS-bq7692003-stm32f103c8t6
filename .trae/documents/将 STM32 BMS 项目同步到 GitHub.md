## 执行步骤

1. **初始化 Git 仓库**
   - 在项目目录下执行 `git init` 初始化 git 仓库

2. **创建 .gitignore 文件**
   - 为 STM32 项目创建合适的 .gitignore 文件，排除编译产物、临时文件等

3. **添加文件到暂存区**
   - 执行 `git add .` 将所有项目文件添加到暂存区

4. **创建初始提交**
   - 执行 `git commit -m "Add STM32 BMS project"` 创建初始提交

5. **添加远程仓库**
   - 执行 `git remote add origin <你的 GitHub 仓库 URL>` 添加远程仓库
   - **注意**：执行时需要你提供完整的 GitHub 仓库 HTTPS URL（例如：https://github.com/username/bms.git）

6. **推送到 GitHub**
   - 执行 `git push -u origin main` 将代码推送到 GitHub（如果远程仓库默认分支是 master，则使用 master）

---

**请在确认计划后，提供你的 GitHub 仓库 URL**