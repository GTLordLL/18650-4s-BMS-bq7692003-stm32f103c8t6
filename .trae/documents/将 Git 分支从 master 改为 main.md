## 将分支从 master 改为 main

1. **重命名本地分支**
   - 执行 `git branch -m master main` 将本地 master 分支重命名为 main

2. **删除远程的 master 分支**
   - 执行 `git push origin :master` 删除 GitHub 上的 master 分支

3. **推送新的 main 分支**
   - 执行 `git push -u origin main` 将 main 分支推送到 GitHub 并设置为上游分支

完成后，你的项目将使用 main 作为默认分支。