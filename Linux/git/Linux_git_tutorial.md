# Git 零基础教程

> 每个操作后展示实际效果，手把手教你掌握 Git

## 准备工作
1. 打开终端（Linux/macOS）或命令提示符/PowerShell（Windows）
2. 确保已安装 Git（输入 `git --version` 检查版本）
3. 配置用户名和邮箱：
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

---

## 第一步：初始化仓库

### 操作：
1. 创建一个新目录并进入：
```bash
mkdir my_project && cd my_project
```
2. 初始化 Git 仓库：
```bash
git init
```

### 预期结果：
```
Initialized empty Git repository in /path/to/my_project/.git/
```

---

## 第二步：创建文件并添加到暂存区

### 操作：
1. 创建一个新文件 `hello.txt` 并写入内容：
```bash
echo "Hello Git Learner!" > hello.txt
```
2. 将文件添加到暂存区：
```bash
git add hello.txt
```

### 预期结果：
- 文件 `hello.txt` 被添加到暂存区，无错误提示。

---

## 第三步：提交更改

### 操作：
1. 提交暂存区的更改：
```bash
git commit -m "Add hello.txt with initial content"
```

### 预期结果：
```
[main (root-commit) abc1234] Add hello.txt with initial content
 1 file changed, 1 insertion(+)
 create mode 100644 hello.txt
```

---

## 第四步：查看提交历史

### 操作：
1. 查看提交记录：
```bash
git log
```

### 预期结果：
```
commit abc1234 (HEAD -> main)
Author: Your Name <your.email@example.com>
Date:   YYYY-MM-DD HH:MM:SS

    Add hello.txt with initial content
```

---

## 第五步：修改文件并查看状态

### 操作：
1. 修改 `hello.txt` 文件：
```bash
echo "This is my first Git project." >> hello.txt
```
2. 查看当前状态：
```bash
git status
```

### 预期结果：
```
On branch main
Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   hello.txt
```

---

## 第六步：查看更改内容

### 操作：
1. 查看文件的更改：
```bash
git diff
```

### 预期结果：
```
diff --git a/hello.txt b/hello.txt
index 1234567..89abcde 100644
--- a/hello.txt
+++ b/hello.txt
@@ -1 +1,2 @@
 Hello Git Learner!
+This is my first Git project.
```

---

## 第七步：撤销更改

### 操作：
1. 恢复文件到上次提交的状态：
```bash
git restore hello.txt
```

### 预期结果：
- 文件 `hello.txt` 的更改被撤销。

---

## 第八步：创建分支

### 操作：
1. 创建并切换到新分支：
```bash
git checkout -b feature_branch
```

### 预期结果：
```
Switched to a new branch 'feature_branch'
```

---

## 第九步：合并分支

### 操作：
1. 切换回主分支：
```bash
git checkout main
```
2. 合并分支：
```bash
git merge feature_branch
```

### 预期结果：
```
Updating abc1234..def5678
Fast-forward
 hello.txt | 1 +
 1 file changed, 1 insertion(+)
```

---

## 第十步：推送到远程仓库

### 操作：
1. 添加远程仓库：
```bash
git remote add origin https://github.com/your_username/my_project.git
```
2. 推送到远程仓库：
```bash
git push -u origin main
```

### 预期结果：
```
Enumerating objects: 3, done.
Counting objects: 100% (3/3), done.
Writing objects: 100% (3/3), 250 bytes | 250.00 KiB/s, done.
Total 3 (delta 0), reused 0 (delta 0)
To https://github.com/your_username/my_project.git
 * [new branch]      main -> main
```

---

## 练习任务
1. 创建一个新分支 `practice_branch` 并切换到该分支。
2. 在分支中添加一个新文件 `practice.txt`，写入任意内容并提交。
3. 合并 `practice_branch` 到 `main` 分支。
4. 推送更新到远程仓库。

恭喜！你已经完成了 Git 的基础操作训练。要深入学习，建议每天花 10 分钟练习这些命令。
