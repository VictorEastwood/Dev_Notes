# 树莓派 2.8 英寸 HDMI 屏幕配置教程

> 适用于：Raspberry Pi 4 + Ubuntu Server 22.04 + 2.8 英寸 HDMI 屏幕 (480x640)

---

## 问题症状

- ❌ 屏幕方向错误（需要旋转 90°）
- ❌ 字体太小看不清
- ❌ 系统检测到错误分辨率（1080x1920 而非 480x640）
- ❌ 上下有黑边

---

## 解决方案

### Step 1: 编辑配置文件

```bash
sudo nano /boot/firmware/config.txt
```

### Step 2: 修改显示配置

找到 `display_auto_detect=1` 这行，**注释掉**（前面加 `#`），然后在文件末尾添加：

```ini
# === 注释掉自动检测 ===
# display_auto_detect=1

# === HDMI DPI 2.8英寸屏幕 (480x640) ===
# 禁用 HDMI 热插拔检测
hdmi_edid_file=1

# 强制 HDMI 输出
hdmi_force_hotplug=1
hdmi_drive=2

# 自定义 HDMI 时序 (480x640 @ 60Hz)
hdmi_cvt=480 640 60 6 0 0 0
hdmi_group=2
hdmi_mode=87

# 显示旋转（逆时针 90°）
display_rotate=3

# 禁用扫描
disable_overscan=1
```

保存退出：`Ctrl+X` → `Y` → `Enter`

### Step 3: 配置大字体

```bash
sudo tee /etc/default/console-setup > /dev/null << 'EOF'
CHARMAP="UTF-8"
CODESET="Lat15"
FONTFACE="TerminusBold"
FONTSIZE="32x16"
EOF

sudo setupcon
```

### Step 4: 重启生效

```bash
sudo reboot
```

---

## 配置说明

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `hdmi_cvt` | 480 640 60 6 0 0 0 | 自定义分辨率 480x640 @ 60Hz |
| `display_rotate` | 3 | 逆时针旋转 90°（0=0°, 1=90°顺, 2=180°, 3=270°） |
| `FONTSIZE` | 32x16 | 大字体，适合 2.8 英寸屏幕 |

---

## 常见问题

### Q: 屏幕还是方向不对？

尝试其他旋转值：
- `display_rotate=0` - 0°（默认）
- `display_rotate=1` - 顺时针 90°
- `display_rotate=2` - 180°
- `display_rotate=3` - 逆时针 90°（270°）

### Q: 字体还是太小/太大？

修改 `/etc/default/console-setup` 中的 `FONTSIZE`：

```bash
# 可选字体大小
FONTSIZE="20x10"   # 中等
FONTSIZE="24x12"   # 较大
FONTSIZE="32x16"   # 最大（推荐）
```

然后执行：
```bash
sudo setupcon
```

### Q: 有黑边怎么办？

确保配置中有：
```ini
disable_overscan=1
```

---

## 临时测试字体（不永久保存）

```bash
# 测试不同大小
sudo setfont Lat15-Terminus20x10
sudo setfont Lat15-Terminus24x12
sudo setfont Lat15-Terminus32x16
```

---

## 配置文件完整示例

`/boot/firmware/config.txt`:

```ini
[all]
kernel=vmlinuz
cmdline=cmdline.txt
initramfs initrd.img followkernel

[pi4]
max_framebuffers=2
arm_boost=1

[all]
dtparam=audio=on
dtparam=i2c_arm=on
dtparam=spi=on
disable_overscan=1
enable_uart=1
camera_auto_detect=1
# display_auto_detect=1  # 已注释
arm_64bit=1

# === HDMI DPI 2.8英寸屏幕 (480x640) ===
hdmi_edid_file=1
hdmi_force_hotplug=1
hdmi_drive=2
hdmi_cvt=480 640 60 6 0 0 0
hdmi_group=2
hdmi_mode=87
display_rotate=3

dtoverlay=dwc2

[cm4]
dtoverlay=dwc2,dr_mode=host
```

---

_最后更新：2026-03-12_
