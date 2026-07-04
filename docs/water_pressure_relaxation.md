# NoitaWorld 水/岩石质量-动量-压力释放模型

目标：先做一个适合 CPU 原型、后续可迁移到 GPU compute shader 的格点水模型。它不使用“水粒子抢格子”的原地 swap，而是允许水在搬运阶段重叠，再用压力释放把超过容量的水推向周围。

## Cell 数据

每个格子保存：

```txt
material    0 = air/fluid space, 1 = rock/solid
mass        当前格子的水量，0 表示无水
momentum_x  x 方向总动量
momentum_y  y 方向总动量
```

速度不是单独的主状态，而由动量和质量得到：

```txt
vx = momentum_x / mass
vy = momentum_y / mass
```

这样多个水团进入同一格时可以直接相加：

```txt
new_mass      = sum(mass_i)
new_momentum  = sum(momentum_i)
new_velocity  = new_momentum / new_mass
```

这避免了“多个水同时抢同一个格子”的冲突。

## 一个时间步

```txt
step():
  1. gravity
  2. advect/accumulate
  3. pressure_relax 多次迭代
  4. damping
  5. update_texture
```

### 1. gravity

对每个有水的非岩石格：

```txt
momentum_y += mass * gravity * dt
```

自由落体水只增加向下速度，不会凭空横向扩散。

### 2. advect / accumulate

根据速度把水搬运到目标格：

```txt
target = current + velocity * dt
```

当前 CPU 原型中目标会落到整数格子上，并沿路径检查岩石。若路径撞到岩石：

```txt
撞到左右岩石 -> vx = 0
撞到上下岩石 -> vy = 0
```

然后把水量和动量累加到 next buffer 的目标格。岩石格不能进入。

注意：CPU 原型可以直接 scatter 相加；GPU 版本不能简单让很多线程同时写同一个 cell，除非使用 atomic add。更 GPU 友好的版本可以改成 gather：每个目标格只读取周围可能流入自己的源格并求和，或者用专门的 scatter-list + sort/reduce/atomic pass。

### 3. pressure_relax

容量参数：

```txt
capacity = 1.0
max_stable_mass = 1.05
pressure = max(0, mass - max_stable_mass)
```

当一个格子质量超过 `max_stable_mass` 时，多出来的水被视为压力来源。对每个有正压力的格子，查看邻居：

```txt
邻居不是岩石
且 pressure_self > pressure_neighbor
```

把超出的质量按压力差比例释放给这些邻居：

```txt
weight_i = pressure_self - pressure_neighbor_i
flow_i = excess_mass * weight_i / sum(weight)
```

释放时同时转移动量：

```txt
flow_momentum = source_momentum * (flow_mass / source_mass)
```

并添加一个小的压力冲量，使被释放的水带有朝向邻居的初速度：

```txt
pressure_impulse = direction * flow_mass * pressure_impulse_strength
```

### 4. 多次迭代

每个 step 中重复压力释放若干次：

```txt
for i in 0 .. pressure_iterations-1:
    pressure_relax()
```

迭代越多，水越接近不可压，压力传播越快，但计算量也越大。

## 为什么这个模型比 water-mass-only 更合理

如果直接用质量差驱动横向扩散，自由落体水柱会错误地向两边散开。这里把质量和压力分开：

```txt
mass 表示有多少水
pressure 只来自超容量/重叠/受约束
```

所以自由落体时：

```txt
mass ≈ 1
pressure ≈ 0
```

不会横向散开。撞到地面或岩石后，水被累积到同一格，产生 `mass > 1.05`，于是压力释放把水推向左右、上方或其他可释放方向。

## 当前简化

- 只模拟 rock 和 water。
- rock 是固定 solid。
- water 可以重叠累加。
- 压力释放用局部 4 邻域。
- 不是严格 Navier-Stokes，也没有全局压力泊松求解。
- 先追求可视化和可调参数，后续再迁移 GPU。

## 下一版改进：多半径压力释放 / block pressure release

当前版本的压力释放只看 4 邻域，所以压力传播速度受限：

```txt
一个压力格每次 relaxation 最多把质量传到上下左右相邻 1 格
```

即使每帧做 8 次 relaxation，压力实际也只能扩散约 8 个 cell，而且还会被每轮的局部分配和重力重新压缩抵消。因此当持续添加水时，水深不明显增加，底部可能不断被重力压缩，而横向/向上释放追不上。

改进思路：对一个有超容量压力的 cell，不只看相邻 4 格，而是看一个局部窗口，例如半径 `r = 4` 的 `9x9` 区域，或半径 `r = 8` 的 `17x17` 区域。把可释放的超容量质量按压力差分配到窗口内所有可达候选格。

### 候选格

对源格 `S`：

```txt
pressure_S = max(0, mass_S - max_stable_mass)
excess_S   = mass_S - max_stable_mass
```

枚举窗口内候选格 `T`：

```txt
abs(dx) <= release_radius
abs(dy) <= release_radius
T 不是岩石
pressure_S > pressure_T
```

其中：

```txt
pressure_T = max(0, mass_T - max_stable_mass)
```

为了避免穿墙传送，候选格最好再加一个近似可达性条件：

```txt
从 S 到 T 的直线路径不穿过岩石
```

CPU 原型可以用 Bresenham / 小步采样检查；GPU 版本可以先不用精确路径，只用局部窗口 + solid mask，后续再加阻挡修正。

### 权重

按压力差和距离衰减分配：

```txt
pressure_diff = pressure_S - pressure_T
distance      = sqrt(dx*dx + dy*dy)
distance_falloff = 1 / (1 + distance)
weight_T = pressure_diff * distance_falloff
```

也可以加入方向偏好。例如为了减少无意义向上喷发，可以暂时使用：

```txt
if dy < 0: upward_factor = 0.65
else:      upward_factor = 1.0
weight_T *= upward_factor
```

但如果希望压力近似各向同性，可以不加方向偏好。

### 释放质量

当前方案先尝试“全部释放超容量质量”：

```txt
release_mass = excess_S * pressure_release_fraction
```

其中当前计划：

```txt
pressure_release_fraction = 1.0
```

然后：

```txt
flow_T = release_mass * weight_T / sum(weight)
```

这会让压力传播远快于 4 邻域逐格传递，能更明显地增加水体深度和横向扩散速度。

### 动量处理

质量释放时继续转移动量：

```txt
flow_momentum = source_momentum * (flow_T / mass_S)
```

并给一个压力冲量：

```txt
dir = normalize(Vector2(dx, dy))
flow_momentum += dir * flow_T * pressure_impulse_strength
```

这样从高压格释放出去的水会获得朝目标方向的初速度。

### CPU 实现方式

为了避免一边读一边写导致顺序偏差，仍然用 delta buffer：

```txt
for each source cell S:
    if pressure_S <= 0: continue
    collect candidates in local window
    compute weights
    for each candidate T:
        delta_mass[S] -= flow_T
        delta_mass[T] += flow_T
        delta_momentum[S] -= source_momentum_part
        delta_momentum[T] += source_momentum_part + pressure_impulse

apply all delta buffers after scanning all cells
```

这仍然是同步 pressure pass。

### GPU 实现备注

多半径 release 如果用 scatter，会遇到多个 cell 写同一目标的冲突。可选方案：

```txt
1. atomic add：简单，但浮点 atomic 性能和平台支持需要评估。
2. scatter-list + sort/reduce：每个源格输出若干条 (target, mass, momentum)，再按 target 归并。
3. gather：每个目标格反向扫描半径内源格，计算有多少水会流入自己；无写冲突，但计算更重。
```

CPU 原型优先验证效果，后续 GPU 化时再决定 atomic / reduce / gather。

### 暂定参数

```txt
release_radius = 6 或 8
pressure_release_fraction = 1.0
pressure_iterations = 8  // 暂时不改，先看多半径 release 的效果
max_stable_mass = 1.05
```

## 当前实验参数：release radius = 8, distance decay = 0.1

本次实验把压力释放从 4 邻域改成局部窗口：

```txt
pressure_release_radius = 8
pressure_distance_decay = 0.1
pressure_release_fraction = 1.0
pressure_iterations = 16
```

当前权重公式：

```txt
pressure_diff = pressure_S - pressure_T
distance = sqrt(dx*dx + dy*dy)
weight_T = pressure_diff / (1 + distance * pressure_distance_decay)
```

注意这里不是 `pressure_diff / (1 + distance)`，而是把距离衰减强度降到 0.1：

```txt
1 + distance * 0.1
```

含义：远处仍然被惩罚，但惩罚很弱。例如：

```txt
distance = 1  -> denominator = 1.1
distance = 4  -> denominator = 1.4
distance = 8  -> denominator = 1.8
```

这样可以让压力在一个 step 内更快传播到半径 8 内的区域，避免只靠逐格扩散导致释放速度赶不上重力累积。

当前 CPU 原型枚举的是以源格为中心的正方形窗口：

```txt
ox = -8 .. 8
oy = -8 .. 8
```

所以最多检查 `17x17 - 1 = 288` 个候选 cell。候选必须满足：

```txt
不是源格自己
在世界范围内
不是 rock
pressure_S > pressure_T
```

当前还没有做“源格到目标格之间是否被岩石阻挡”的 line-of-sight 检查，所以水压可能会在半径内绕过很薄的岩石局部传递。这是后续需要修正的地方。

## 当前改动：密度越大释放越快 + 目标密度上限 1.3

在保持“重叠 -> 压力释放”的机制不变时，加入两个参数：

```txt
pressure_density_speed = 1.0
pressure_target_max_mass = 1.3
```

### 1. 密度越大，释放越快

源格超过稳定质量的部分：

```txt
excess = max(0, mass_S - max_stable_mass)
```

释放量从原来的：

```txt
releasable = excess * pressure_release_fraction
```

改成：

```txt
density_speed_factor = 1 + excess * pressure_density_speed
releasable = excess * pressure_release_fraction * density_speed_factor
```

也就是说，源格越拥挤，单位 pressure pass 中尝试释放的质量越多。当前：

```txt
pressure_density_speed = 1.0
```

示例：

```txt
mass = 1.10, excess = 0.05 -> factor = 1.05
mass = 1.50, excess = 0.45 -> factor = 1.45
mass = 2.00, excess = 0.95 -> factor = 1.95
```

这会让高密度重叠水团更快释放，而低压力水仍然较慢。

### 2. 不向密度大于 1.3 的目标继续释放

候选目标格必须满足：

```txt
target_mass < pressure_target_max_mass
```

当前：

```txt
pressure_target_max_mass = 1.3
```

第二遍真正写 delta 时也会限制目标容量：

```txt
target_capacity = 1.3 - (target_mass + pending_incoming_mass)
flow = min(flow, target_capacity)
```

这避免压力释放继续把水塞进已经很密的格子。注意这个容量限制在 CPU delta-buffer 中会使用本 pass 已经累计的 incoming mass，因此会有轻微顺序性，但能有效防止过度堆叠。

### 当前初始实验参数

```txt
gravity = 0.28
pressure_iterations = 8
pressure_release_radius = 1
pressure_distance_decay = 0.1
pressure_density_speed = 1.0
pressure_target_max_mass = 1.3
max_stable_mass = 1.05
pressure_release_fraction = 1.0
```

## 回滚：移除密度加速与目标密度上限

上一版尝试过：

```txt
pressure_density_speed = 1.0
pressure_target_max_mass = 1.3
```

即“密度越大释放越快”和“不向 mass >= 1.3 的目标释放”。实际观察效果更差，因此当前已从 pressure release 实现中移除这两个机制。

当前 release 又恢复为：

```txt
excess = max(0, mass_S - max_stable_mass)
releasable = excess * pressure_release_fraction
```

候选目标只要求：

```txt
目标不是 rock
pressure_S > pressure_T
```

不再检查目标质量是否超过 1.3，也不再用 excess 放大释放量。

## 当前改动：向上压力的特殊填充规则

目标：保留当前“先压缩/重叠，再压力释放”的机制，但解决底部长期压缩、水面不升高的问题。

### 原因

普通压力邻居 release 会把 `excess` 按压力差分给窗口内所有候选格。如果左、右、下没有正压力差，而上方有正压力差，那么理论上 excess 会分给上方。但在普通 release 中，上方质量仍然只是按权重分配到若干上方候选格，不能很好模拟静水中压力把体积向上支撑到水面。

### 新规则

压力权重仍然先正常计算：

```txt
pressure_S = mass_S - max_stable_mass
pressure_T = max(0, mass_T - max_stable_mass)
weight_T = (pressure_S - pressure_T) / (1 + distance * pressure_distance_decay)
```

但候选格分成两类：

```txt
非上方候选：oy >= 0，包括左右和下方
上方候选：oy < 0，包括正上方和左右上方
```

先计算：

```txt
total_weight = 所有候选权重之和
upward_weight = 所有 oy < 0 候选权重之和
releasable = excess * pressure_release_fraction
upward_releasable = releasable * upward_weight / total_weight
```

然后：

```txt
1. 非上方候选仍按普通压力权重释放。
2. 上方候选不再按普通权重直接分散。
3. upward_releasable 改为执行“上方特殊填充”。
```

### 上方特殊填充

从当前 cell 上方第一行开始，逐行向上填：

```txt
for oy = -1, -2, ..., -radius:
    先填正上方 x
    再填 x-1, x+1
    再填 x-2, x+2
    ...
    当前这一行填完后，才进入更高一行
```

每个目标格必须满足：

```txt
在范围内
不是 rock
当前质量 + 本 pass 已累计流入质量 < max_stable_mass
```

目标容量：

```txt
capacity = max_stable_mass - (target.mass + max(0, pending_delta_mass))
flow = min(remaining_upward, capacity)
```

也就是说，上方填充不会继续把上层 cell 塞到超过 `max_stable_mass`。

### 特殊情况

如果左、右、下没有正压力差，而只有上方有正压力差，那么：

```txt
upward_weight == total_weight
upward_releasable == releasable
```

于是所有可释放 excess 都会尝试进入上方特殊填充。这正是“底部不能再左右/向下释放时，压力把体积向上支撑”的近似。

### 当前限制

- 该规则仍是局部的，最多只填到 `pressure_release_radius` 范围内。
- 仍然不是完整压力泊松求解。
- 只检查目标格是否是 rock，没有做路径穿墙检测；但由于上方填充按行逐次填充、半径通常较小，穿墙感比任意半径最低质量搜索弱得多。

## 修正：上方填充应使用“非上/下左右填充后的剩余释放量”

上一版的问题：

```txt
upward_releasable = releasable * upward_weight / total_weight
```

这意味着上方填充只拿到“按权重本来会给上方”的那一小部分。只要左右方向有空格，左右也会持续吃掉 release，于是水被铺成薄膜，底部很难把体积真正推高。

修正后：

```txt
1. 先只处理非上方候选：oy >= 0，包括左右和下方。
2. 非上方候选每格最多填到 max_stable_mass。
3. 如果 releasable 还有剩余，剩余部分全部进入上方特殊填充。
```

也就是：

```txt
remaining_release = releasable
先用 pressure 权重填 left/right/down，且受目标容量限制
remaining_release -= actual_flow
再把 remaining_release 用上方特殊填充逐行向上填
```

这样当左右/下方已经没有容量或没有压力差时，全部剩余 excess 会向上支撑水面；不会再被左右无限摊薄。

## 当前回滚：移除上下/左右 directional relay fill

最新实现已经删除“excess 沿上/下/左/右方向穿过已满水格，继续寻找可填充 cell”的机制。

原因：只给某些方向特殊传递，或者把 excess 远距离填补，会让水柱/水面出现很强的人造形状，例如尖锐喷泉或不自然的厚度变化。

当前 `pressure_relax_once()` 回到局部作用域：

```txt
for each source cell S:
    if mass_S <= max_stable_mass:
        continue

    pressure_S = mass_S - max_stable_mass
    excess_S   = mass_S - max_stable_mass
    releasable = excess_S * pressure_release_fraction

    在 pressure_release_radius 半径内枚举候选 T：
        T 在范围内
        T 不是 rock
        pressure_S > pressure_T

    weight_T = (pressure_S - pressure_T) / (1 + distance * pressure_distance_decay)
    flow_T   = releasable * weight_T / sum(weight)
```

也就是说：

- 不再有 `upward_fill`。
- 不再有 left/right/down 的 relay fill。
- 不再“穿过已满水格继续找更远的空位”。
- 压力只直接作用到当前半径窗口内的低压 cell。
- 默认 `pressure_release_radius = 1` 时，就是非常局部的邻域压力释放。

保留的部分：

- 仍然是“advect 允许重叠 -> pressure release 释放超容量”的整体框架。
- 仍然用 delta buffer，避免同一 pass 内读写顺序造成明显偏差。
- 释放质量时仍然按比例转移动量，并加入很小的压力方向冲量。

## 当前实验：增大压力释放初始冲量

为了测试“压缩释放时给更大的初始动量”是否能让高压水更快横向/向上逃逸，当前默认值改为：

```txt
pressure_impulse_strength = 1.0
```

Godot 调试面板中已经加入滑条：

```txt
pressure impulse strength: 0.0 .. 5.0
```

它影响的是 release 时这项：

```txt
impulse = direction * flow_mass * pressure_impulse_strength
```

因此它不改变释放出去的质量数量，只改变释放出去的水获得的初始动量。值越大，压缩后的水越容易被“喷”出去；太大时可能产生抖动或喷泉。

## 当前状态：删除水枪 UI，保留带初速度注水 API

Godot 侧已经删除“水枪模式”和对应 UI 滑条，界面又回到普通画笔：

```txt
1 = 水
2 = 岩石
3 = 擦除
LMB = 画当前材料
RMB = 擦除
```

但 C++ 底层仍保留这个通用方法：

```txt
inject_water(x, y, radius, mass_per_cell, velocity_x, velocity_y)
```

它不是固定“水枪工具”，而是供之后鼠标/脚本自行调用的注水 API。它会在圆形范围内添加水，并直接给新加入的水初始动量：

```txt
added_momentum = added_mass * initial_velocity
```

例如之后如果希望“按住鼠标添加向右初速度的水”，可以在 GDScript 中直接调用：

```txt
world.inject_water(local.x, local.y, radius, mass_per_cell, horizontal_speed, 0.0)
```

当前主场景默认尺寸也改回几万格级别：

```txt
world_size = 320 x 180 = 57,600 cells
display_scale = 3.0
```
