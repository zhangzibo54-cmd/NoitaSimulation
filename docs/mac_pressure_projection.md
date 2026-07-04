# MacWorld：MAC 网格 + PCG 压力投影方案

这个文件记录新的流体方案。旧的 `NoitaWorld` 仍然保留；新方案单独实现为：

```txt
src/mac_world.h
src/mac_world.cpp
project/mac_main.gd
```

当前 `project/main.tscn` 指向 `res://mac_main.gd`，所以 Godot 运行主场景时会使用新的 `MacWorld`。

## 1. 网格变量：为什么 face velocity 不会重复存储

MAC grid 把变量放在不同位置：

```txt
cell center:
    material    air / rock
    mass        水量/占据率
    pressure    压力未知数 p

vertical face:
    u           水平速度

horizontal face:
    v           竖直速度
```

如果 cell 是 `(i,j)`，它的四条边速度是：

```txt
left   face: u(i,   j)
right  face: u(i+1, j)
up     face: v(i, j)
down   face: v(i, j+1)
```

存储尺寸：

```txt
mass / pressure / material: width * height
u: (width + 1) * height
v: width * (height + 1)
```

所以相邻两个 cell 公用同一个 face 速度。例如：

```txt
cell(i,j) 的 right face = u(i+1,j)
cell(i+1,j) 的 left face = u(i+1,j)
```

它只存一份，不会左右各存一份。这解决了“一个边到底属于哪个 cell”的问题：边速度属于 face，不属于某个 cell。

## 2. 当前时间步

当前每个 `step()` 做：

```txt
1. predict_velocity_explicit()
2. build_pressure_system()
3. solve_pressure_pcg()
4. apply_pressure_projection()
5. advect_mass_finite_volume()
6. clamp_velocities()
7. update_texture()
```

## 3. 显式速度预测：包含对流、粘性、重力

连续形式：

```txt
∂u/∂t + (u · ∇)u = - 1/rho ∇p + nu ∇²u + f
```

压力先不算，得到临时速度：

```txt
u* = u^n + dt [ - (u^n · ∇)u^n + nu ∇²u^n + f ]
```

这里没有忽略粘性项，`nu ∇²u` 已经放进 `predict_velocity_explicit()`。

当前实现只更新“至少一侧邻接液体 cell”的 face velocity；完全处在空气中的 face 会被清零。否则空气区域会持续受重力加速，之后水进入该区域时会继承不合理的空气速度。

### 3.1 对流项

二维速度：

```txt
U = (u, v)
```

对流项：

```txt
(U · ∇)u = u ∂u/∂x + v ∂u/∂y
(U · ∇)v = u ∂v/∂x + v ∂v/∂y
```

当前最初实现使用中心差分近似。例如对某个 `u` face：

```txt
du/dx ≈ (u(x+1,y) - u(x-1,y)) / 2
du/dy ≈ (u(x,y+1) - u(x,y-1)) / 2
```

因为 `u` face 上没有直接存 `v`，所以用周围四个 `v` face 平均得到该位置的 advecting velocity：

```txt
v_at_u_face ≈ average(v 左上, v 右上, v 左下, v 右下)
```

同理，对 `v` face 计算对流时，需要用周围四个 `u` face 平均出：

```txt
u_at_v_face
```

### 当前实验：暂时关闭显式对流项

为了确认静水后内部持续流动是不是由显式中心差分对流项引起，当前代码已经临时关闭：

```txt
-(U · ∇)u
-(U · ∇)v
```

所以当前预测速度实际是：

```txt
u* = u + dt [ nu ∇²u ]
v* = v + dt [ nu ∇²v + gravity ]
```

这不是最终物理版本，只是诊断实验：如果关闭后静水内部流动明显减少，说明原来的中心差分对流项确实在放大/维持非物理涡动；后续可以改成 upwind、semi-Lagrangian 或 MacCormack，而不是直接用中心差分显式对流。

### 3.2 粘性项

粘性项是速度的 Laplacian：

```txt
∇²u = ∂²u/∂x² + ∂²u/∂y²
```

离散为：

```txt
∇²u ≈ u_left + u_right + u_up + u_down - 4u_center
```

当前默认：

```txt
viscosity = 0.04
```

注意：当前是显式粘性，`dt` 和 `viscosity` 太大可能不稳定。

### 3.3 重力

重力只加到竖直 face velocity：

```txt
v* = v + dt * gravity
```

屏幕 y 向下，所以正 gravity 表示向下。

## 4. 为什么这里暂时不用 semi-Lagrangian

Semi-Lagrangian 本质是对速度采样点反向追踪：

```txt
x_old = x - dt * velocity(x)
u*(x) = sample(u_old, x_old)
```

它很稳定，但会引入数值耗散，而且需要插值采样。当前为了让公式和差分推导更直接，先使用原始有限差分：

```txt
u* = u + dt [ - (u · ∇)u + nu ∇²u + f ]
```

这更接近原始 PDE 离散，但稳定性要求更严格：

```txt
dt * max_velocity / h < 1
dt * viscosity / h² 不宜太大
```

## 5. divergence：如何从 face velocity 计算 b

对 cell `(i,j)`：

```txt
div(U*) =
    u*(i+1,j) - u*(i,j)
  + v*(i,j+1) - v*(i,j)
```

如果网格大小 `h != 1`，需要除以 `h`。当前代码默认 `h = 1`。

压力方程来自：

```txt
U_new = U* - alpha ∇p
div(U_new) = s
```

其中 face 上：

```txt
alpha_f = dt / rho_f
```

当前版本先不再把 `mass` 当作真实密度。这里采用常数水密度：

```txt
rho_f = rho_water = 1.0
alpha_f = dt / rho_water
```

原因：`mass` 更像 volume fraction / cell 水量，不应该直接解释为水的真实密度。否则薄水 cell 会得到很小的 `rho` 和很大的 `alpha = dt/rho`，压力会过度把自由表面推向空气。

当前用阈值决定谁是 pressure unknown：

```txt
pressure_active_mass = adjustable, default 0.30
```

规则：

```txt
mass > pressure_active_mass:
    作为 pressure-active liquid cell，拥有压力未知数 p

mass <= pressure_active_mass:
    不作为压力未知数，视为自由表面/薄水/空气侧，p = 0
```

所以小于 0.30 的 cell 可以继续携带质量、受重力、参与质量输运；但它不作为邻居 pressure unknown 参与线性系统。对相邻的 active cell 来说，它就是 `p_neighbor = 0` 的自由表面。

代入：

```txt
div(U*) + A p = s
```

当前实现构造正定形式：

```txt
A p = b
```

其中：

```txt
A p = Σ_faces alpha_f * (p_center - p_neighbor)
b   = s - div(U*)
```

注意符号来自当前速度修正式：

```txt
U_new = U* - alpha * grad(p)
```

如果你的纸面定义把离散 gradient / Laplacian 方向反过来，公式可能写成 `div(U*) - s`；关键是必须和最终 velocity correction 的符号一致。

### 5.1 目标 divergence：过密流出 + 内部欠密流入

之前只做：

```txt
div(U_new) = 0
```

这只能防止速度继续压缩，但不能修复已经出现的：

```txt
mass > 1
```

所以当前加入：

```txt
target_mass = 1.0
density_correction_strength = 0.5
underfill_correction_strength = 0.2

over = max(mass - target_mass, 0)
s = density_correction_strength * over / dt

if internal_liquid:
    under = max(target_mass - mass, 0)
    s -= underfill_correction_strength * under / dt
```

含义：

```txt
mass > 1:
    s > 0，要求该 cell 产生净流出

mass < 1 且是内部液体:
    s < 0，要求该 cell 产生净流入，用来填补水体内部/贴岩石底部的稀疏空洞

mass < 1 但在自由表面:
    不加 underfill，避免水面永远吸水、坍缩或从空气边界产生不自然流入
```

内部液体 `internal_liquid` 的当前判定：

```txt
当前 cell 不是 rock
当前 cell mass > 0.01
四邻域中，非 solid 的邻居都满足 mass > pressure_active_mass
```

也就是说：

```txt
rock/边界 不算空气
mass <= pressure_active_mass 的非 solid 邻居算自由表面/空气侧
```

也就是说，新压力解算使用：

```txt
1. face coefficient alpha_f = dt / rho_water
2. RHS 中的 s(mass) 过密/内部欠密修正项
3. mass > pressure_active_mass 的 active set；内部欠密 cell 也可加入 active set；薄自由表面 cell 作为 p=0
```

## 6. 固体边界和自由边界

### 6.1 岩石 / solid

岩石要求 no-through：

```txt
U_new · n = 0
```

当前实现：

```txt
如果一个 face 任一侧是 rock 或越界：
    该 face 法向速度 = 0
```

压力矩阵中，solid 邻居使用 Neumann-like 处理：

```txt
solid 不是 pressure unknown
solid 邻居不增加 off-diagonal
也不作为 p=0 的自由面
```

直觉：墙不会吸收压力，也不会允许法向通量。

### 6.2 空气 / free surface

液体旁边如果是 air：

```txt
p_air = 0
```

这是 Dirichlet 自由表面边界。矩阵中表现为：

```txt
air 邻居增加 diagonal
但没有 off-diagonal unknown
```

这表示水面接触大气压，压力参考值为 0。

## 7. PCG 如何求解

压力方程是大稀疏线性系统：

```txt
A p = b
```

每个液体 cell 最多连接上下左右，所以 `A` 是 5-point stencil。当前实现不显式存矩阵，只实现：

```txt
apply_laplacian(x) = A x
```

PCG 的核心变量：

```txt
r = b - A p        residual
z = M^-1 r         preconditioned residual
d                 conjugate search direction
q = A d
```

当前预条件器是 Jacobi preconditioner：

```txt
M = diag(A)
z_i = r_i / A_ii
```

每轮 PCG：

```txt
q = A d
alpha = (r · z) / (d · q)
p = p + alpha d
r = r - alpha q
z = M^-1 r
beta = (r_new · z_new) / (r_old · z_old)
d = z + beta d
```

为什么比 Jacobi 快：

```txt
Jacobi 每轮只是局部平均；
PCG 每轮选择 A-共轭方向，并用全局内积求最优步长，
所以不会反复破坏之前已经修正好的误差方向。
```

## 8. 压力投影

解出 `p` 后，用 pressure gradient 修正 face velocity：

```txt
u_new(face between L and R) = u* - dt/rho * (p_R - p_L)
v_new(face between U and D) = v* - dt/rho * (p_D - p_U)
```

如果一侧是 air：

```txt
p_air = 0
```

如果 face 接 solid：

```txt
face velocity = 0
```

## 9. 水量搬运

压力投影只修正速度，不直接移动水量。水量之后用有限体积通量更新：

```txt
mass_new = mass_old - dt * divergence(mass_flux)
```

每条 face 的 mass flux 用 upwind donor：

```txt
如果 face velocity > 0:
    donor = 左/上 cell
否则:
    donor = 右/下 cell

flux = velocity * dt * donor_mass
```

这一步现在使用两阶段 conservative flux limiter：

```txt
1. 先计算所有 face 的 raw flux，但不立刻应用。
2. 对每个 donor cell 统计 total_outflow。
3. 如果 total_outflow > mass_cell，则把该 cell 的所有流出 flux 等比例缩小：

   scale = mass_cell / total_outflow

4. 再统一应用所有缩放后的 flux。
```

这个修复很重要：旧版本每条 face 独立限制流量，一个 cell 可能同时从多个方向流出，导致 `next_mass < 0`；负值被清零后会破坏守恒，并可能诱发巨大质量/显示溢出。新版本保证：

```txt
每个 cell 的总流出 <= 当前 mass
```

并且在写回时会清理：

```txt
非有限数值 NaN/Inf -> 0
mass < 0.01 -> 0
```

### 9.1 压力求解优化

当前 PCG 不再在整个 `width * height` 网格上做所有内积和矩阵乘，而是在 `build_pressure_system()` 中生成：

```txt
active_cells = mass > pressure_active_mass 且不是 rock 的 cell
```

PCG 的 `A*x`、dot product、向量更新主要遍历 `active_cells`。同时每步预计算 face 上的：

```txt
u_alpha = dt / rho_u_face
v_alpha = dt / rho_v_face
```

避免在 PCG 每次 `apply_laplacian()` 时反复计算密度和边界判断。这对水只占一部分区域时会明显快于全图遍历。

## 10. 当前参数

```txt
world_size = 320 x 180 = 57,600 cells
dt = 0.18
gravity = 0.75
viscosity = 0.04
PCG max iterations = 20
density_correction_strength = 0.5
underfill_correction_strength = 0.2
display_scale = 3.0
```

当前实验还移除了 mass 搬运后的上限裁剪：

```txt
旧: mass = clamp(next_mass, 0, 2)
新: mass = next_mass
```

也就是说不再因为 `mass > 2` 直接截断水量。注意这只去掉了上限截断；如果后续仍出现质量漂移，真正原因通常在 flux 计算/边界通量/负质量清零，而不是上限本身。

这些参数是为了先验证“压力投影是否比局部 mass release 更稳定”。后续如果显式对流仍然振荡，可以再改：

```txt
更小 dt
更大 viscosity
更强 velocity damping
或改用 semi-Lagrangian / MacCormack advection
```

## 代码结构重构：MacWorld / MacSimulation 分层

现在 MAC 水体原型拆成两层：

- `src/mac_world.h/.cpp`
  - Godot 侧 `Node2D` 包装层。
  - 负责 `_ready()`、`_process()`、`_draw()`、GDExtension 方法绑定、Image/ImageTexture 更新、暂停/速度/显示缩放等和 Godot 生命周期相关的事情。
  - 它不再直接保存 `mass/u/v/pressure/material` 等求解器数组，而是把参数设置、绘制、注水、step 等操作转发给 `MacSimulation`。

- `src/mac_simulation.h/.cpp`
  - 纯 C++ 模拟核心，尽量不依赖 Godot API。
  - 保存材料、质量、压力、MAC 速度场、PCG 缓冲区、flux 缓冲区等数据。
  - 负责水体模拟步骤：显式速度预测、压力系统组装、PCG 求解、压力投影、有限体积质量平流、速度限制、统计信息。
  - 提供 `fill_rgba_pixels()` 把当前模拟状态转成 RGBA 字节数组，供 `MacWorld` 上传到 Godot 纹理。

这样后续加入沙子、烟、火、材料属性表、GPU 版本时，不需要继续把所有逻辑塞进 Godot 节点类里。`MacWorld` 可以继续作为“场景里的可显示节点”，而具体物理可逐步拆到更多 simulation/material 文件中。
