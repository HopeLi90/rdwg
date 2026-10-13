# dwgparse

零第三方依赖（纯 `std`）的 DWG 解析器，支持 **主流 AutoCAD 版本**：

| 内部代号 | 对应 AutoCAD 发行版 | 状态 |
| --- | --- | --- |
| AC1015 | 2000 / 2000i / 2002 | ✅ 完全支持（扁平段，对象按绝对偏移寻址） |
| AC1018 | 2004 / 2005 / 2006 | ✅ 支持（`AcFssFcAJMB` **段树**容器：XOR 头 + ACD 压缩 + 段页映射） |
| AC1021 | 2007 / 2008 / 2009 | ✅ 支持（**页容器**：RS 去交织 + LZ77） |
| AC1024 | 2010 / 2011 / 2012 | ✅ 支持（**复用 AC1018 段树**容器，非页容器） |
| AC1027 | 2013 / 2014 / 2015 / 2016 / 2017 | ✅ 支持（同上） |
| **AC1032** | **2018 / 2019 / 2020 / 2021 / 2022 / 2023 / 2024** | ✅ 支持（**CAD 2020 就是这一档**） |
| AC1037 | 2024+ | ❌ 不支持（更新格式，无实测样本，拒绝而非猜测解析） |

> **AutoCAD 2020 没有独立的文件格式**：它写出的是 `AC1032`（"DWG 2018 格式"），
> 与 2018、2024 完全同一档。因此"支持 CAD 2020"等价于"支持 AC1032"，已覆盖。
>
> ⚠️ **容器只有两套，不是一套**：AC1021 是唯一使用页容器（RS 去交织 + LZ77）的发行版。
> AC1018 用的是以 12 字节 `AcFssFcAJMB` 标识串开头的**段树**结构（LibreDWG 单开
> `decode_r2004`），而 **AC1024 / AC1027 / AC1032 复用的正是这个段树**，不是页容器。
> 所以实现 `decode_R2004` 一次性解锁了 R2004 / R2010 / R2013 / R2018–2024 四档。
>
> 容器只是上半场：**对象字段本身也是按版本裁剪的**。最容易踩的是 LibreDWG 的
> `common_entity_data.spec` —— `is_xdic_missing` 起于 R_2004a，`material_flags` /
> `shadow_flags` 与"每对象字符串流"起于 R_2007a，三个 visual-style 位起于 R_2010b，
> `has_ds_data` 起于 R_2013。用 AC1021 的布局去解所有版本不会报错，只会静默产出
> 看起来合理的垃圾坐标。`EntityVariant` 就是为此把发行版一路带进实体解码器。
>
> **对象（表记录）还有一套独立的闸门**，而且更容易被忽略：
> `common_object_handle_data.spec` 在 `SINCE(R_2013)` 多一个 `has_ds_data(B)`（实体那份由
> `common_entity_data.spec` 提供，所以几何交叉核对永远发现不了这一个位）；而
> `COMMON_TABLE_FLAGS` 的分支写的是 **`UNTIL (R_2004)`** —— AC1018 仍然读
> `is_xref_ref(B)` + `is_xref_resolved(BS)` + `is_xref_dep(B)` 共 4 位，R2007+ 才只剩一个
> `BS`。这两处各错 1~2 位都不会报错，只会让整张图层表的 `flag0` / CMC 颜色 / 线型句柄
> 集体错位。现在由 `tests/layer_table.rs` 按 LibreDWG 的 DXF `62 / 370 / 70` 逐字段钉死。
>
> 早期版本曾把 AC1018 一并路由到 R2007 解码器，结果必然失败且报出误导性的
> `decompression error: offset underflow`（真因是容器选错）—— 这是横向联测用真实
> R2004 样本（`tests/data/example_2004.dwg`）抓出来的。

提取三类信息：**图层**、**图元几何坐标**、**扩展属性**（EED + XData）。

## 支持的图元几何类型

| 类别 | 类型 |
| --- | --- |
| 基础 | `LINE`、`POINT`、`CIRCLE`、`ARC`、`TEXT`、`MTEXT`（多行文本，含段落转义） |
| 块参照 | `INSERT`（仅存 block_header 句柄，二次解析块名） |
| 轻量多段线 | `LWPOLYLINE`（顶点存于对象自身，含 DD 增量解码） |
| 经典多段线 | `POLYLINE`（2D / 3D / MESH / PFACE）—— 头部 + 链式 `VERTEX_*` + `SEQEND`，按句柄向量装配 |
| 顶点 | `VERTEX_2D/3D/MESH/PFACE`（点 + 凸度）、`VERTEX_PFACE_FACE`（仅面索引 71–74，无坐标） |
| 曲线 | `ELLIPSE`（圆心 / 长轴向量 / 轴比 / 起止参数） |
| 平面面片 | `3DFACE`（四个角点）、`SOLID` / `TRACE`（四个角点，共享 elevation） |

> 未实现的实体（DIMENSION / SPLINE / MTEXT / HATCH / LEADER 等）只保留句柄与 EED，不产出几何。

## 快速开始

```bash
cargo build --release
./target/release/dwgparse tests/data/sample_2000.dwg
./target/release/dwgparse tests/data/example_2007.dwg
```

输出四段：`Layers` → `Blocks` → `Entity geometry` → `Extended attributes`。

## 作为库使用

```rust
use dwgparse::{DwgFile, Geometry};

let dwg = DwgFile::parse("tests/data/sample_2000.dwg")?;
for l in &dwg.layers {
    println!("{} (0x{:X})", l.name, l.handle);
    for o in dwg.entities_on_layer(l) {
        if let Some(Geometry::Circle { center, radius }) = &o.geometry {
            println!("  circle {:?} r={radius}", center);
        }
    }
}
for o in &dwg.objects {
    println!("0x{:X} eed={} xdata={}", o.handle, o.eed.len(), o.xdata.len());
}
```

完整 API 示例见 `examples/api_demo.rs`：

```bash
cargo run --example api_demo -- tests/data/sample_2000.dwg
```

## 测试

```bash
cargo test     # 112 passed（位原语 19 + 解码单测 27 + 颜色 3 + R2007 段 4 + EED/XData 4 + 装配链字段流 9 + 端到端 11 + 实体扩展 8 + 不变量 4 + 鲁棒性 4 + CLI 端到端 7 + 版本路由 5 + 图层表 2 + XData 跨版本核对 3）
```

其中 `tests/version_routing.rs` 用**五个真实样本**（同一张图分别由 2004 / 2007 / 2010 / 2013 /
2018 另存）验证版本路由：五档都必须解析成功，且**去掉字符串负载后的几何指纹排序后逐位一致**，
AC1032 与 AC1021 完全一致；同时断言 AC1037（AutoCAD 2024+）等未知代号仍被明确拒绝。

**XData 现已在全部六档（AC1015 / AC1018 / AC1021 / AC1024 / AC1027 / AC1032）上经真值核对**：`tests/xdata.rs` 对 2004 / 2007 / 2010 / 2013 / 2018 五个现代样本的 `XRECORD`（handle `0x26B…0x270`）断言 `[270=Int16(1), 271=Int16(1), 280=Int16(1)]`，并断言 AC1015 的 `SHADEPLOT` 链与现代样本至少 6 个非空 XData 对象。这依赖于两处修复：保留进入时的 **bit 偏移**（否则其后的 `BS cloning`（DXF 280）整体错位——2007 样本曾读出 `-24448` 而非真值 `1`）；以及 R2007+ 字符串按 `Dialect` 走 **TU（UCS-2LE）** 而非 TV（单字节），否则 UTF-16 串及其后的二进制块（TF 311）一并错位（2018 样本 `0x764` 现正确解出 `"BA881000-D789-5D94-D8B4-EEC8EF3D3BFF"`）。

> 这个文件此前依赖"改戳法"（把 AC1021 样本前 6 字节改成别的代号）。那个技巧现在已经
> **不再成立也不再需要**：AC1018 与 AC1032 走的是段树容器、AC1021 走页容器，改戳只会因为
> 容器不匹配而失败，说明不了任何关于版本戳的事；真样本比它强得多。
>
> 指纹只比较坐标、标志位和顶点数，不比较文本负载 —— 这张图在两次另存之间被编辑过：
> 2007 版存的是 MTEXT `108\U+00B0`，2018 版存的是 `108°`（LibreDWG 的 `dwg2dxf` 导出一致）。

新增的三层测试把"能跑不崩"变成"稳定且可证伪"（共 112 个测试，见上）：

- **`tests/invariants.rs`（不变量 4）**：对两个 fixture 参数化执行跨层不变量——坐标无
  NaN/Inf、半径/字高为正、句柄唯一、`entities()` 与 `geometry.is_some()` 计数一致、图层句柄可
  解析、POLYLINE 顶点数与句柄向量一致、LWPOLYLINE 点数一致。
- **`tests/robustness.rs`（鲁棒性 4）**：空文件、过短、随机字节、逐段截断、位翻转模糊测试
  （固定种子的 LCG，同时压 R2000 与 R2007 两条解码路径，共 800 轮）、篡改容器魔数——一律断言
  返回 `Err` 且**绝不 panic**（用 `catch_unwind` 显式保证）。横向联测正是在这里抓出两个真 bug：
  AC1018 被错误路由到 R2007 解码器（已改为拒绝）；EED 解码循环在越界读取时不推进游标导致
  无限 push 直至 OOM（已加 `at_end()` 进度保护）。
- **`tests/cli.rs`（CLI 端到端 7）**：用 `CARGO_BIN_EXE_dwgparse` 调真实二进制，断言正常样本
  exit 0 且 stdout 含预期字段、损坏样本 exit 非 0、未知版本被明确拒绝——补齐"位原语 → 字段流 →
  对象 → 装配 → API → CLI"的顶层闭环。

此外 `src/parser.rs` 单测模块此前已新增 6 个**合成字节字段流测试**，对 SOLID/3DFACE/ELLIPSE（含
`z_is_zero` 与 `has_no_flags` 两个分支）和 MTEXT（R2000 内联 `TV` 与 R2007 字符串流 `TU`，含
`SINCE(R_2007a)` 的 `rect_height`）逐一钉死字段顺序与位宽——这是真值样本覆盖不到的、最隐蔽的
一类回归。

本次继续补强：又在 `src/parser.rs` 单测模块新增 **9 个合成字节字段流测试**，覆盖 POLYLINE
装配链的两个此前**零覆盖**的位级原语——`decode_vertex`（VERTEX_2D 全字段 `flag/point/widths/
bulge/tangent`、VERTEX_3D 仅 `flag/point`、VERTEX_PFACE_FACE 四个签名索引而非点）与
`decode_polyline_header`（POLYLINE_2D / _3D / _PFACE / _MESH 四子型字段顺序，含 R2007 的
`num_owned(BL)` 对齐与 seqend 句柄消费），以及 `decode_insert` 的旋转角与块句柄解析。这两个原语
此前仅由 `tests/invariants.rs` 的"POLYLINE 顶点数一致"间接验证——字段顺序一旦回归，高层测试会
照样放过，正是纵向联测最该补的薄弱点，现在被逐一钉死。

端到端断言与 LibreDWG 导出的 DXF 交叉核对（`tests/crosscheck_geom.py`，六档全部 `ALL CHECKS PASS`）：

- **AC1015 / AC1018 / AC1021 / AC1024 / AC1027 / AC1032** 六档全部通过，每档 14 类实体
  （LINE / CIRCLE / ARC / POINT / TEXT / LWPOLYLINE / INSERT / ELLIPSE / 3DFACE / SOLID /
  MTEXT / POLYLINE_3D / POLYLINE_PFACE）逐一比对，**最大偏差 5.1e-11**。
- **R2000**（`sample_2000.dxf`，几何容差 `1e-9`）：LINE/CIRCLE/ARC/POINT/TEXT/INSERT/LWPOLYLINE 几何坐标逐字段精确吻合。
- **图层表**（`tests/layer_table.rs`）：同一样本的五档（AC1018/1021/1024/1027/1032）各 5 条
  图层记录，逐条比对 DXF `2`（名称）、`62`（颜色，7/2/4/7/7）、`370`（`-3`，即内部线宽 31）
  与 `70`（冻结/锁定/打印标志）。AC1018 的系统图层单独是 `70 = 5`，其余各档为 `0`——这一处
  差异正是 `COMMON_TABLE_FLAGS` 走 `UNTIL (R_2004)` 分支的直接证据。
- **R2007**（`example_2007.dxf`，按字段集合匹配）：

  | 实体 | 数量 | 最大坐标偏差 |
  | --- | --- | --- |
  | ELLIPSE | 1 / 1 | 4.7e-11 |
  | 3DFACE | 2 / 2 | 3.3e-11 |
  | SOLID | 15 / 15 | 4.9e-11 |
  | POLYLINE_3D | 1 / 1 | 4.8e-11 |
  | POLYLINE_PFACE | 1 / 1 | 4.9e-11 |
  | MTEXT | 18 / 18 | 4.7e-11（文本字符串**完全一致**） |

  偏差仅来自 IEEE-754 双精度文本化的舍入（DWG 的 `BD`/`RD` 与 DXF 导出本就是同一组源双精度），
  即几何**逐位等价**，远非"能跑不崩"。
  MTEXT 的文本字符串是逐字符精确匹配的——它只有在前置字段顺序
  （含 `SINCE(R_2007a)` 的 `rect_height`）完全正确时才会对上，因此这一项同时也是字段流顺序的强断言。

验证脚本（可重跑）位于 `tests/crosscheck_geom.py`：

```bash
python3 tests/crosscheck_geom.py                    # 六档全跑
python3 tests/crosscheck_geom.py --only AC1032      # 只跑一档
```

真值 DXF 由 LibreDWG 的 `dwg2dxf` 生成一次即可：

```bash
dwg2dxf -o /tmp/truth_2018.dxf tests/data/example_2018.dwg
```

调试用的位级转储工具：

```bash
cargo run --example dbg_layer                          # 所有图层的原始 CMC / 句柄
cargo run --example dbg_layer -- tests/data/example_2007.dwg 10   # 单个图层的逐位走查
cargo run --example dbg_obj  -- tests/data/example_2007.dwg 4E3   # 任意对象的三流起点
cargo run --example dump_geom -- tests/data/example_2007.dwg      # 全几何全精度 dump
```

## 文档

技术设计方案（文件结构 → 位级原语 → 三层解码 → API 范式 → **踩坑清单**，含 POLYLINE 句柄向量装配机制）：

- `docs/技术设计方案.md`

## 限制

支持 AC1015（2000–2002）、AC1018（2004–2006）、AC1021（2007–2009）与
AC1024 / AC1027 / AC1032（2010–2024，**含 CAD 2020**）共六档版本；
AC1037（AutoCAD 2024+）因是更新格式、无实测样本而明确拒绝，不做猜测解析。
未实现的实体（DIMENSION / SPLINE / HATCH / LEADER 等）只保留句柄与 EED。
详见设计文档第 9 章。

## Windows 可执行程序

```bash
rustup target add x86_64-pc-windows-gnu
apt-get install -y gcc-mingw-w64-x86-64     # 提供 kernel32/user32/msvcrt 等导入库
cargo build --release --target x86_64-pc-windows-gnu
# 产物：target/x86_64-pc-windows-gnu/release/dwgparse.exe（PE32+ x86-64）
```

已编译好的产物：`dwgparse-windows-x64.exe`（约 1.3 MB，控制台程序，零运行时依赖）。
用法：`dwgparse-windows-x64.exe <file.dwg>`。
