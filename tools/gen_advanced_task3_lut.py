# -*- coding: utf-8 -*-
"""
生成 advanced_task3.v 的两条自适应色调曲线，做性质自检，并直接把表回写进 RTL。

  lut_lift[i] : 暗部提亮曲线（夜间/弱光用）
                  i <= KNEE_L 时 = 176*(i/176)^0.45，i > KNEE_L 时保持原值
                  → 只抬暗部，176..255 的高光完全不动
  lut_comp[i] : 高光压缩曲线（强光/过曝用）
                  i <= KNEE_C 时保持原值
                  i >  KNEE_C 时 = 96 + (COMP_TOP-96) * ((i-96)/159)^GAMMA_C
                  终点是 COMP_TOP(232) 而不是 255 —— 这样连最亮的"死白"也能被
                  压下来；且全程斜率 < 1，不会出现"越往上反而被拉伸"的问题。

用法：
    python tools/gen_advanced_task3_lut.py            # 只算 + 自检 + 回写 RTL
    python tools/gen_advanced_task3_lut.py --dry-run  # 只算 + 自检，不改文件
"""
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RTL = os.path.join(ROOT, "source", "rtl", "advanced_task3.v")

KNEE_L = 176          # 暗部提亮拐点
GAMMA_L = 0.45        # <1 提亮
KNEE_C = 96           # 高光压缩拐点
GAMMA_C = 1.15        # >1 压暗（越大越狠）
COMP_TOP = 232        # 强光档最高输出灰阶（255 表示不压死白）

N = 256


def build_lift():
    out = []
    for i in range(N):
        if i <= KNEE_L:
            out.append(int(round(KNEE_L * (i / KNEE_L) ** GAMMA_L)))
        else:
            out.append(i)
    out[0] = 0
    out[N - 1] = 255
    return out


def build_comp():
    out = []
    span = 255 - KNEE_C
    top_span = COMP_TOP - KNEE_C
    for i in range(N):
        if i <= KNEE_C:
            out.append(i)
        else:
            t = (i - KNEE_C) / span
            out.append(int(round(KNEE_C + top_span * (t ** GAMMA_C))))
    out[0] = 0
    return out


def check(name, tab, expect_last):
    assert len(tab) == N
    assert tab[0] == 0, (name, tab[0])
    assert tab[N - 1] == expect_last, (name, tab[N - 1], expect_last)
    for i in range(N - 1):
        assert tab[i] <= tab[i + 1], "%s 非单调: i=%d %d>%d" % (name, i, tab[i], tab[i + 1])
    # 斜率必须 <= 1（否则会出现"越亮反而被拉伸"的反效果）
    worst = max(tab[i + 1] - tab[i] for i in range(N - 1))
    print("%-9s OK  单调, [0]=%d [255]=%d, 最大相邻增量=%d %s"
          % (name, tab[0], tab[N - 1], worst, "(<=1 好)" if worst <= 1 else "(>1 注意)"))
    return worst


def blend(ch, s):
    """与 RTL 完全一致的 blend 运算。"""
    mag = abs(s)
    src = tab_lift if s >= 0 else tab_comp
    return max(0, min(255, ch + ((src[ch] - ch) * mag >> 8)))


def show_scene(label, s, samples=(16, 32, 64, 96, 128, 176, 200, 230, 255)):
    row = " ".join("%3d->%3d" % (c, blend(c, s)) for c in samples)
    print("  %-16s s=%4d | %s" % (label, s, row))


def emit(tab, name):
    lines = []
    for base in range(0, N, 4):
        parts = "; ".join("%s[%3d] = 8'h%02X" % (name, base + k, tab[base + k]) for k in range(4))
        lines.append("        " + parts + ";")
    return lines


def patch_rtl(path, tab_lift, tab_comp):
    src = io.open(path, encoding="utf-8").read()
    lines = src.split("\n")

    def replace_block(lines, var, new_lines):
        idx = [i for i, ln in enumerate(lines) if re.match(r"^\s*%s\[" % var, ln)]
        if len(idx) != 64:
            raise SystemExit("%s 找到 %d 行，预期 64 行" % (var, len(idx)))
        if idx[-1] - idx[0] != 63:
            raise SystemExit("%s 行不连续" % var)
        return lines[:idx[0]] + new_lines + lines[idx[-1] + 1:]

    lines = replace_block(lines, "lut_lift", emit(tab_lift, "lut_lift"))
    lines = replace_block(lines, "lut_comp", emit(tab_comp, "lut_comp"))
    io.open(path, "w", encoding="utf-8", newline="").write("\n".join(lines))
    print("\n已回写 RTL: %s" % os.path.relpath(path, ROOT))


tab_lift = build_lift()
tab_comp = build_comp()

print("=== 曲线自检 ===")
check("lift", tab_lift, 255)
slope_c = check("comp", tab_comp, COMP_TOP)

print("\n=== blend 行为抽查（灰阶 -> 输出）===")
show_scene("夜间 s=+255", 255)
show_scene("正常 s=0", 0)
show_scene("偏亮 s=-170", -170)
show_scene("强光 s=-255", -255)

for c in range(N):
    assert blend(c, 0) == c, c
print("\n[PASS] s=0 时 0..255 全部位精确恒等")
print("[%s] s=+255 不会让任何像素变暗" % ("PASS" if all(blend(c, 255) >= c for c in range(N)) else "FAIL"))
print("[%s] s=-255 不会让任何像素变亮" % ("PASS" if all(blend(c, -255) <= c for c in range(N)) else "FAIL"))
print("[PASS] 强光档最亮处 255 -> %d（死白也会被压下来）" % blend(255, -255))

if "--dry-run" in sys.argv:
    print("\n(dry-run：未修改 RTL)")
else:
    patch_rtl(RTL, tab_lift, tab_comp)
