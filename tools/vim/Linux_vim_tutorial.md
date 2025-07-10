# Vim 零基础教程

> 每个操作后展示实际效果，手把手教你掌握 Vim

## 准备工作
1. 打开终端（Linux/macOS）或命令提示符/PowerShell（Windows）
2. 输入 `vim` 启动编辑器

```bash
vim
```

---

## 第一步：创建新文件

### 操作：
1. 在普通模式下输入：`:e hello.txt`
2. 按回车键

### 预期结果：
```
~
~                
~
~
~
~
~
~
~
~
"hello.txt" [New]
```

> 状态：已创建名为 `hello.txt` 的新文件（尚未保存到磁盘）

---

## 第二步：进入插入模式并添加内容

### 操作：
1. 按 `i` 键（屏幕底部显示 `-- INSERT --`）
2. 输入以下内容：
```
Hello Vim Learner!

This is my first Vim document.
I'm following a step-by-step tutorial.
```
3. 按 `Esc` 键返回普通模式

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.
I'm following a step-by-step tutorial.
~
~
~
~
~
~
-- INSERT --
```
> 按 `Esc` 后底部状态消息消失

---

## 第三步：保存文件

### 操作：
1. 输入：`:w`
2. 按回车键

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.
I'm following a step-by-step tutorial.
~
~
~
~
~
"hello.txt" [New] 4L, 90B written       4,38      all
```

---

## 第四步：基础光标移动练习
你可以尝试以下命令来移动光标：
| 命令 | 功能 |
|------|------|
| `h` | 左移 |
| `j` | 下移 |
| `k` | 上移 |
| `l` | 右移 |
| `w` | 移动到下一个单词开头 |
| `b` | 移动到上一个单词开头 |
| `e` | 移动到单词结尾 |
| `0` | 移动到行首 |
| `^` | 移动到行首非空白字符 |
| `$` | 移动到行尾 |
| `gg` | 移动到文件开头 |
| `G` | 移动到文件末尾 |
| `:<行号>` | 跳转到指定行 |
---

## 第五步：文本编辑 - 删除与修改

### 操作 1（删除单词）：
1. 确保光标在第三行行首
2. 按 `w` 移动到 "following" 开头
3. 输入：`dw`（删除单词）

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.
I'm a step-by-step tutorial.
~
~
~
~
~
```

### 操作 2（修改单词）：
1. 移动光标到 "tutorial" 的 "t" 下
2. 输入：`cw`（删除单词并进入插入模式）
3. 输入：`guide`（替换文本）
4. 按 `Esc`

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.
I'm a step-by-step guide.
~
~
~
~
~
```

---

## 第六步：添加新行

### 操作：
1. 按 `o`（在下方新建一行）
2. 输入：`Let me practice more:`
3. 按 `Esc`

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.
I'm a step-by-step guide.
Let me practice more:
~
~
~
~
~
```

---

## 第七步：复制粘贴操作

### 操作：
1. 移动光标到第二行
2. 按 `V`（进入行可视模式）
3. 按 `j` 选择两行（第二和第三行）
4. 按 `y` 复制
5. 按 `G` 跳转到文件末尾
6. 按 `p` 粘贴

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.
I'm a step-by-step guide.
Let me practice more:
This is my first Vim document.
I'm a step-by-step guide.
~
~
~
~
```

---

## 第八步：搜索文本

### 操作：
1. 输入：`/first`
2. 按回车键
3. 按 `n` 两次

### 预期结果：
```
Hello Vim Learner!

This is my first Vim document.   <-- 第一个匹配项（高亮显示）
I'm a step-by-step guide.
Let me practice more:
This is my first Vim document.   <-- 第二个匹配项（按 n 后跳转到这里）
I'm a step-by-step guide.
~
~
~
/first
```

> 提示：按 `Esc` 清除搜索高亮

---

## 第九步：替换文本

### 操作：
1. 输入：`:%s/document/file/g`
2. 按回车键

### 预期结果：
```
Hello Vim Learner!

This is my first Vim file.   <-- 所有 document 被替换为 file
I'm a step-by-step guide.
Let me practice more:
This is my first Vim file.
I'm a step-by-step guide.
~
~
~
:%s/document/file/g
```

---

## 第十步：保存并退出

### 操作：
1. 输入：`:wq`
2. 按回车键

### 预期结果：
- 文件保存为 `hello.txt`
- 返回终端/命令提示符界面

---

## 第十一步：逐个删除字母

### 操作：
1. 将光标移动到需要删除的字母上
2. 按 `x` 键（删除光标所在的字母）
3. 重复按 `x` 删除更多字母

### 预期结果：
- 删除光标所在的字母，后续字母向左移动填补空位。

---

## 第十二步：撤销与恢复

### 操作 1（撤销操作）：
1. 按 `u` 键（撤销上一步操作）
2. 多次按 `u` 可连续撤销多步操作

### 操作 2（恢复操作）：
1. 按 `Ctrl + r`（恢复被撤销的操作）

### 预期结果：
- 按 `u` 后，最近的更改被撤销。
- 按 `Ctrl + r` 后，撤销的更改被恢复。

---

## 最终文件内容
使用 `cat hello.txt`（Linux/macOS）或 `type hello.txt`（Windows）查看：

```
Hello Vim Learner!

This is my first Vim file.
I'm a step-by-step guide.
Let me practice more:
This is my first Vim file.
I'm a step-by-step guide.
```

---

## 练习任务
1. 重新打开文件：`vim hello.txt`
2. 完成以下操作：
   - 在末尾添加一行 "Vim is powerful!"
   - 将 "guide" 改为 "tool"
   - 删除最后两行
   - 搜索 "Vim" 并跳转到第三个匹配项
   - 保存为 `hello_final.txt`

> 提示：使用命令 `:saveas hello_final.txt` 另存为新文件

恭喜！你已经完成了 Vim 的基础操作训练。要深入学习，建议每天花 10 分钟练习这些命令。