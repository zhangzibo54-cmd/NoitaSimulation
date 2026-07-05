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

## 代码结构重构：材料定义 / 流体求解器 / 组装层

本轮把之前仍然偏臃肿的 `MacSimulation` 继续拆分：

- `src/material_defs.h/.cpp`
  - 定义全局材料 ID：`AIR / ROCK / WATER / SAND / SMOKE`。
  - 定义 `MaterialDef` 属性表：`solid / liquid / gas / powder / pressure_solved / blocks_velocity / density / viscosity`。
  - 通过 `noita::get_material_def(material_id)` 查询属性。
  - 这样材料常数不再属于某一个具体 solver，后续沙子、烟、火、岩浆都能共享同一套材料定义。

- `src/mac_fluid_solver.h/.cpp`
  - 原来 `MacSimulation` 里的 MAC 水体求解代码移动到了这里。
  - 负责当前水体状态、MAC 速度场、压力 PCG、质量平流、流体绘制数据生成等。
  - 这是“具体工作者”：只关心当前 MAC fluid prototype 怎么算。

- `src/mac_simulation.h/.cpp`
  - 现在变成组装/调度层。
  - 当前只持有一个 `MacFluidSolver fluid_solver` 并把 API 转发给它。
  - 之后可以继续加入 `PowderSolver / GasSolver / FireSolver`，并在 `MacSimulation::step()` 中统一决定执行顺序和跨材料交互。

- `src/mac_world.h/.cpp`
  - 仍然是 Godot `Node2D` 包装层，只处理 GDExtension 绑定、纹理上传、显示和输入转发。

目标结构变为：

```text
MacWorld        Godot-facing Node2D / rendering wrapper
  -> MacSimulation    scheduler / orchestration / interaction layer
       -> MacFluidSolver   MAC water solver
       -> future PowderSolver / GasSolver / FireSolver
  -> material_defs    shared material constants/properties
```

注意：当前还没有真正把 `WorldGrid` 独立出来。现阶段 `MacFluidSolver` 仍然拥有当前水体网格数组；下一步如果要让沙子和水强交互，应该继续把 `material/mass/temperature` 等共享状态抽成 `WorldGrid`，让各 solver 操作同一个 grid。

## WorldGrid 共享状态抽取

现在已经把 solver 之间需要共享的 cell-centered 世界状态抽到了 `src/world_grid.h/.cpp`：

- `material`：每个 cell 的材料 ID，例如 `AIR / ROCK / WATER / SAND / SMOKE`。
- `mass`：每个 cell 的材料量。当前主要表示水量，后续粉末/气体也可以复用或扩展出分层质量。
- `pressure`：流体压力解算后的 cell-centered pressure。它现在属于世界状态，因此沙子、气体、火焰等未来 solver 可以读取流体压力做交互。
- `velocity_x / velocity_y`：cell-centered 速度摘要。MAC fluid 内部仍保留更精确的 face velocity `u/v`，但每步结束会把它平均回 cell-centered velocity，方便其他 solver 读取。
- `temperature`：预留温度场，之后可用于火、蒸汽、岩浆、水汽化等。
- `lifetime`：预留生命周期/年龄场，之后可用于烟雾消散、火焰持续时间、粒子老化等。

`next_mass` 没有放进 `WorldGrid`。它是当前有限体积平流算法的一帧临时缓冲区：先根据速度把下一帧的质量写入 `next_mass`，确认守恒和非负之后再提交回 `WorldGrid::mass`。这种 scratch buffer 属于具体 solver 的内部实现，不应该成为跨 solver 共享世界状态。

当前 ownership：

```text
MacSimulation
  owns WorldGrid grid
  owns MacFluidSolver fluid_solver

MacFluidSolver
  binds to WorldGrid
  owns solver-only scratch buffers: next_mass, rhs, residual, PCG vectors, MAC face velocities u/v, flux buffers
  reads/writes shared grid fields: material, mass, pressure, velocity_x, velocity_y
```

因此之后添加沙子时，更合理的方向是：

```text
MacSimulation::step()
  powder_solver.step(grid)
  fluid_solver.step(grid)      // 当前通过 bind_world(grid)
  resolve_material_interactions(grid)
  gas_solver.step(grid)
```

跨材料交互应该优先通过 `WorldGrid` 里的共享字段通信，而不是让 solver 之间直接互相调用。

## PowderSolver：沙子/粉末原型

现在新增 `src/powder_solver.h/.cpp`，作为第一个非流体 solver。当前实现以沙子为代表：

- `WorldGrid` 维护 canonical active lists：
  - `active_liquid_cells`
  - `active_powder_cells`
- `PowderSolver` 不拥有世界状态，只接受 `WorldGrid &grid` 作为输入/输出。
- `PowderSolver` 可以有自己的临时顺序数组 `powder_order`，用于本帧排序和扫描；这种排序结果属于 solver scratch，不放进 `WorldGrid`。

当前 `MacSimulation::step()` 调度顺序：

```text
if powder exists:
  grid.rebuild_active_cells()
  powder_solver.step(grid)          // 沙子先移动，成为流体边界
  grid.rebuild_active_cells()

fluid_solver.step()                 // 以岩石/沙子为 blocks_velocity 边界解液体

if powder exists:
  resolve_fluid_powder_interactions()
```

沙子更新规则：

1. 每帧随机决定同一行是左到右还是右到左扫描，减少方向偏置。
2. 按 `y` 从下到上处理，避免上面的沙子一帧内穿过下面刚落下的沙子。
3. 优先尝试直下。
4. 直下被挡时尝试斜下，斜下方向受到当前水平速度影响，否则由本帧扫描方向打破平局。
5. 若沙子已有明显水平速度，则允许很慢的水平侧移，用于压力推动后的堆体松动。
6. 沙子进入水格时会和水交换位置，形成“沙子沉入水中，水被挤到原位置”的近似。

流体-沙子交互目前放在 `MacSimulation::resolve_fluid_powder_interactions()`：

- 只遍历 `active_powder_cells`。
- 检查沙子四邻域的液体压力。
- 如果邻接液体压力超过阈值，就给沙子一个远离液体方向的小速度增量。
- 如果对应方向被边界挡住，则该方向增量被取消。

这个交互是有意放在组装层，而不是放在 `MacFluidSolver` 或 `PowderSolver` 内部：因为它属于跨 solver 规则，而不是某一个 solver 的内部数值方法。

## PCG 热路径优化：预打包压力 stencil rows

`apply_laplacian()` 是 PCG 中最热的 SpMV 操作。之前每次 PCG 迭代、每个 active cell 都会重复：

- `i % width` 和 `i / width` 反推坐标；
- `u_index/v_index/cell_index` 重算邻居和 face 下标；
- `is_pressure_active_masked()` 做 bounds check 和 mask 查询；
- 对四个方向做 `a > 0` 分支；
- 每轮开头对整张 `r_ax` 做 `std::fill()`。

这些 alpha、邻居 active 拓扑在一次 `solve_pressure_pcg()` 的所有迭代中不变，因此现在在 `build_pressure_system()` 压缩最终 active set 之后，每步重建一次 `pressure_rows`：

```cpp
struct PressureStencilRow {
    int32_t self;
    int32_t nbr[4];
    float diag;
    float offdiag[4];
};
```

其中：

- `diag = a_left + a_right + a_up + a_down`；
- active 邻居存真实线性下标；
- inactive/air/solid 邻居把 `nbr[k]` 填成 `self`，同时 `offdiag[k] = 0`。

于是 `apply_laplacian()` 变为 branch-light 的扁平 SpMV：

```cpp
ax = diag * p[self]
   - offdiag[0] * p[nbr[0]]
   - offdiag[1] * p[nbr[1]]
   - offdiag[2] * p[nbr[2]]
   - offdiag[3] * p[nbr[3]];
```

并删除了对整网格 `r_ax` 的清零，因为 PCG 只读取 active rows 写入的项。

这个缓存不是跨帧缓存，而是每个 simulation step 在 `build_pressure_system()` 中重建一次，所以不会因为下一帧 mass/material/active set 改变而 stale。

## PCG 预条件器：Jacobi -> 串行 MIC(0)/IC(0) 风格不完全 Cholesky

原先 PCG 使用 Jacobi 预条件：

```cpp
z[i] = residual[i] * diag_inv[i];
```

它完全并行、非常便宜，但只使用矩阵对角项，不能很好地传播大尺度低频误差。对于大水体，PCG 轮数容易顶到 `pressure_iterations` 上限。

现在改成每个 pressure solve 中构建一次 MIC(0)/IC(0) 风格的不完全 Cholesky 预条件器：

```text
A ≈ L D L^T
solve M z = r, M = L D L^T
```

实现位置：

- `MacFluidSolver::build_mic_preconditioner()`
- `MacFluidSolver::apply_preconditioner()`

构建时机：

```text
build_pressure_system()
  -> active set / pressure stencil rows 确定
  -> build_mic_preconditioner()

solve_pressure_pcg()
  -> 每轮 PCG 复用这个 preconditioner
```

注意它不是跨帧缓存。每个 simulation step 都会根据当前 `material / mass / active set / alpha` 重建一次，所以不会因为水体拓扑变化而 stale。

当前实现是稳定优先版本：

- 只保留 left/up 下三角连接，zero fill-in；
- 构建 `L D L^T`；
- 如果某行分解出现过小或非有限对角，则该行退回接近 Jacobi 的安全对角；
- apply 阶段需要一次 forward sweep 和一次 backward sweep。

这比 Jacobi 单轮更贵，因为三角求解是串行的；但它通常能显著减少 PCG 所需轮数，并改善大水体压力平衡的稳定性。

并行/GPU 注意：

- Jacobi 预条件完全并行，适合 GPU / SIMD / 多线程；
- MIC(0)/IC(0) 的 forward/backward triangular solve 有数据依赖，是 CPU 单线程更友好的稳定解；
- 将来如果把压力求解迁移到 GPU，建议切回 Jacobi/weighted Jacobi、red-black smoother，或直接上 multigrid，而不是照搬这个串行 MIC(0)。

## 回退：MIC(0)/IC(0) 预条件器暂时停用

实际测试中，当前不完全 Cholesky/MIC 风格预条件器带来了不稳定和更高单步开销，因此 PCG 已回退到原来的 Jacobi 预条件：

```cpp
z[i] = residual[i] * diag_inv[i];
```

保留的优化：

- `pressure_rows` 预打包 stencil；
- `apply_laplacian()` 的快速 SpMV；
- `pressure_active_mask`。

也就是说，当前继续优化 `A*q` 热路径，但预条件器回到并行友好且稳定的 Jacobi。以后如果需要更强压力求解，优先考虑 multigrid / red-black smoother，而不是当前这个串行 MIC 实现。

## Toxic liquid：随质量平流的毒素标量

新增毒液不是单独一套流体 solver，而是在 `WorldGrid` 中加入一个 cell-centered 标量：

```cpp
std::vector<float> toxic;
```

含义是“毒素总量”，不是浓度。浓度由下面计算：

```cpp
toxic_concentration = toxic[i] / mass[i]
```

这样更适合守恒搬运：液体质量流过 face 时，毒素按 donor cell 的浓度随质量一起搬运。

当前 advection 规则：

```text
mass_flux = velocity * dt * donor_mass
toxic_flux = mass_flux * clamp(toxic[donor] / mass[donor], 0, 1)
```

然后：

```text
next_mass[from]  -= mass_flux
next_mass[to]    += mass_flux
next_toxic[from] -= toxic_flux
next_toxic[to]   += toxic_flux
```

也就是说毒素扩散不单独跑一个 pass，而是自然跟随水的有限体积质量输运发生。

显示/材料判定在整次 advection 结束后统一提交：

```cpp
world->toxic[i] = clamp(next_toxic[i], 0, world->mass[i]);
float c = world->toxic[i] / world->mass[i];
world->material[i] = c >= 0.25 ? MATERIAL_TOXIC : MATERIAL_WATER;
```

因此：

- 浓度 `< 0.25`：仍然保留毒素标量，但显示为普通水；
- 浓度 `>= 0.25`：按浓度逐渐把水染成绿色毒液；
- 毒素随水流守恒搬运，不会因为低于显示阈值而立刻消失。

Godot 控制：

```text
6 = toxic liquid
```

## Volume / density / immiscible oil-water phase split

本轮把 `WorldGrid` 里的液体主标量从语义上修正为：

```cpp
volume[i]   // cell 的填充体积/占据率，不是物理质量
density[i]  // 材料密度；液体格由水/油相比例混合得到
oil[i]      // 不相容油相体积
```

对于液体：

```text
water_volume = volume - oil
oil_fraction = oil / volume
density = rho_water * (1 - oil_fraction) + rho_oil * oil_fraction
```

`material` 仍然只是显示/交互用的主导相：

```text
oil_fraction >= 0.5 -> MATERIAL_OIL
else                 -> MATERIAL_WATER / MATERIAL_TOXIC
```

毒液 `toxic` 仍是可溶标量；油 `oil` 是不相容相体积。二者结构上都是随液体搬运的 cell-centered 标量，但物理含义不同。

### 不改变速度，只改变 face flux 的油水比例

这里没有单独修改 `u` / `v`，也没有给 `v_up` 之类的速度项添加特殊规则。压力投影和速度场先照常给出一个总的体积通量：

```text
volume_flux = velocity_face * dt * donor_volume
```

然后只在有限体积平流阶段，把这个总通量拆成油通量和水通量。新的局部规则不再只用 donor cell 的油水比例，而是用 face 两侧 cell 的合计比例：

```text
phi_face = (oil_A + oil_B) / (volume_A + volume_B)
requested_oil_flux   = abs(volume_flux) * phi_face
requested_water_flux = abs(volume_flux) * (1 - phi_face)
```

直觉：一个 face 两侧的液体共同决定“通过这条边的局部混合物是什么”，而不是只由流出方决定。这样油水界面处的通量会更像局部界面交换，而不是单纯把 donor 的浓度复制出去。

注意这里完全不调整 `u/v` 或 `v_up`。速度场只决定总 `volume_flux`，油/水只是在这个总通量内部重新分账。

### 守恒约束

直接用 `phi_face` 有一个问题：donor 可能没有足够的油或水。例如 donor 几乎全是水，但 receiver 很油，于是 `phi_face` 可能要求从 donor 流出很多油。为避免凭空生成相体积，代码对每个 donor 的所有流出 face 做 component availability bound：

```text
sum(oil_out_from_donor)   <= donor_oil
sum(water_out_from_donor) <= donor_water
```

因此更新仍然是保守的：

```text
next_volume[from] -= oil_flux + water_flux
next_volume[to]   += oil_flux + water_flux
next_oil[from]    -= oil_flux
next_oil[to]      += oil_flux
```

水相没有单独数组，而是始终由 `volume - oil` 得到。

### 当前代码位置

- `src/world_grid.h/.cpp`
  - `mass` 语义改成 `volume`。
  - 新增 `density` 和 `oil`。
  - 新增 `make_oil()`。
- `src/material_defs.h`
  - 新增 `MATERIAL_OIL`，密度暂设为 `0.80`。
- `src/mac_fluid_solver.cpp`
  - `cell_density()` 对液体使用水/油混合密度。
  - `advect_mass_finite_volume()` 中按 `phi_face` 拆分油/水通量。
  - 渲染按 `oil_fraction` 把水色混合到油色。
- `project/mac_main.gd`
  - `7` 键选择 oil brush。

## TODO：静水角落的显示/数值抖动处理

现象：在岩石边界的小角落、狭窄凹槽、薄水/油交界处，压力投影和有限体积平流可能产生持续的小幅波动。表现上是水面/油面在局部一直闪动、起伏，尤其是贴近不规则固体边界的位置。

暂时不直接修改物理求解。后续可考虑两类方案：

1. **物理层稳定化**
   - 对近静止液体加入更强的局部速度阻尼；
   - 对小于阈值的局部压力残差/速度残差做 dead-zone；
   - 对接近静水且封闭的小区域提高 PCG 迭代或增加局部粘性；
   - 改进自由表面/固体边界处的 pressure-active 判定，减少薄液体格反复参与/退出压力解算。

2. **只在最终展示阶段做视觉平滑**
   - 不改 `volume/oil/velocity/pressure` 真实模拟数据；
   - `fill_rgba_pixels()` 渲染时，对液体颜色或显示深度做 3x3 邻居平均 / bilateral-like 平滑；
   - 只平滑液体内部或液体-液体交界，避免把颜色糊到岩石和空气里；
   - 这样可以隐藏小角落数值噪声，但不会改变模拟本身。

当前优先级：先观察；如果视觉抖动影响调试，再优先实现“展示阶段邻居平均”，因为它风险最低，不会破坏守恒。

## 温度、火焰、气体与材料反应表

本轮加入了第一版 Noita-like 材料反应系统，目标不是完整化学模拟，而是建立一个可扩展的“局部规则表 + active list”的反应层。

### 新增材料

`src/material_defs.h` 新增：

```cpp
MATERIAL_FIRE
MATERIAL_STEAM
MATERIAL_TOXIC_GAS
MATERIAL_FLAMMABLE_GAS
MATERIAL_GLASS
```

含义：

- `FIRE`：火焰，不是流体也不是气体，由 `FireSolver` 管理。
- `STEAM`：蒸汽，走 `GasSolver`，遇冷凝结为水。
- `TOXIC_GAS`：毒气，走 `GasSolver`，接触液体会溶回毒液。
- `FLAMMABLE_GAS`：可燃气体，走 `GasSolver`，高温或接触火后燃烧。
- `GLASS`：玻璃，固体边界，来自高温沙子。

### WorldGrid 中的 active lists

`WorldGrid` 现在维护：

```cpp
active_liquid_cells
active_powder_cells
active_gas_cells
active_fire_cells
active_reaction_cells
```

其中 `active_reaction_cells` 的标准是：

```text
material == FIRE
或 temperature > 0.05
或 material 是 liquid/gas/powder
```

也就是说反应层不需要每次盲目处理所有规则对象，只遍历“可能参与反应”的 cell。当前网格 320x180 下即使 rebuild 是全图扫描也还可以接受；后续如果世界更大，可以把 active list 改成 dirty-region / chunk active。

### FireSolver

新增：

```text
src/fire_solver.h
src/fire_solver.cpp
```

当前规则：

1. `FIRE` 每帧加热四邻域。
2. `FIRE` 有生命周期，烧完后变成 `SMOKE`。
3. `FIRE` 接触 `WATER` 会熄灭并生成少量 `STEAM`。
4. `FIRE` 接触 `OIL` 或 `FLAMMABLE_GAS` 会尝试传播。
5. `FIRE` 接触 `TOXIC` 会使毒液蒸发成 `TOXIC_GAS`。

火焰本身主要由：

```cpp
temperature[i]
lifetime[i]
```

驱动，不参与液体压力解算。

### GasSolver 扩展

`GasSolver` 不再只处理 `SMOKE`，现在统一处理所有 `def.gas == true` 的材料：

- `SMOKE`：原来的烟，扩散并消散。
- `STEAM`：上升更快，冷却到阈值以下变回水。
- `TOXIC_GAS`：接触液体时溶解回毒液。
- `FLAMMABLE_GAS`：高温时变成火。

气体仍然使用局部扩散 CA，不走 MAC pressure projection。

### MaterialReactionSolver：反应表

新增：

```text
src/material_reaction_solver.h
src/material_reaction_solver.cpp
```

内部用一个小表记录规则：

```cpp
struct ReactionRule {
    uint8_t subject;
    TriggerKind trigger;      // SelfTemperature 或 NeighborMaterial
    uint8_t neighbor;
    float min_temperature;
    uint8_t output;
    float output_volume;
};
```

当前表：

```text
WATER + 高温       -> STEAM
STEAM + 低温       -> WATER
TOXIC + 高温       -> TOXIC_GAS
OIL + 高温         -> FIRE + SMOKE
SAND + 高温        -> GLASS
WATER + 邻接 FIRE  -> STEAM
OIL + 邻接 FIRE    -> FIRE + SMOKE
TOXIC + 邻接 FIRE  -> TOXIC_GAS
FLAMMABLE_GAS + FIRE -> FIRE
```

岩石当前只会存热，不会变岩浆；这是刻意保守的第一版，避免岩浆流体还没定义好就把固体边界破坏掉。

### 为什么连锁反应放到下一帧

`MaterialReactionSolver::step()` 对每个 active cell 每帧最多应用一条规则。比如：

```text
OIL 邻接 FIRE -> 当前 cell 变 FIRE
新 FIRE 再加热周围 -> 下一帧发生
```

这样做有三个好处：

1. 避免单帧无限连锁反应；
2. 结果更稳定、可调；
3. 以后上 GPU/chunk 并行更简单。

### 当前调度顺序

`MacSimulation::step()` 现在大致是：

```text
1. powder_solver.step(grid)
2. fluid_solver.step(grid)
3. gas_solver.step(grid)
4. fire_solver.step(grid)
5. reaction_solver.step(grid)
6. rebuild active lists
```

火焰先加热，反应表再把满足条件的材料转化。新生成的火/气体继续在下一帧参与求解。

### Godot 操作键

```text
1 water
2 rock
3 air/erase
4 sand
5 smoke
6 toxic liquid
7 oil
8 fire
9 steam
0 toxic gas
F flammable gas
```

### 后续 TODO

- 加 `LAVA`：高温岩石/火焰可产生熔岩，熔岩遇水变岩石/蒸汽。
- 加 `WOOD` 或可燃固体：让火焰有更明显的燃料来源。
- 把 `ReactionRule` 进一步拆成“自身温度规则 / 邻接规则 / 混合规则”，并支持概率、热量释放、消耗比例。
- 让液体中的 `oil_fraction` 而不只是主导 `MATERIAL_OIL` 参与燃烧：例如 oil_fraction > 0.2 且高温就部分消耗油相。

### GasSolver 调整：更明显、更长寿、按浓度上升

本轮针对烟/蒸汽/毒气/可燃气体做了三处调整：

1. **气体更长寿、更明显**
   - `max_lifetime` 从约 520 提高到约 1600；
   - `min_mass` 降低，低浓度气体不会过早消失；
   - 渲染时气体视觉强度提高，生命周期衰减变慢；
   - 火焰熄灭、毒液/水受热时生成的气体 volume 提高。

2. **上升规则加入浓度判断**
   - 气体仍按 active list 从上到下处理；
   - 当一个 gas cell 想向上扩散时，如果上方也是气体且 `volume >= 当前 cell volume`，则本帧不向上流入；
   - 直觉：上面已经同浓度或更浓，不应该继续把气体往上压缩；气体会改为横向/向下少量扩散。

3. **保持 top-to-bottom scan**
   - 气体排序仍是 `ay < by`，即屏幕/网格上方优先处理；
   - 这样上面的低浓度/空格先获得移动机会，下面气体再尝试补上，适合“上升”类 CA。

## Rigid solid chunks：固体连通块刚体原型

本轮加入第一版可旋转刚体系统：

```text
src/rigid_body_solver.h
src/rigid_body_solver.cpp
```

### 目标

让 `ROCK / GLASS` 这类 solid 像素在失去支撑后可以变成独立刚体块，并支持：

- 初始/局部 solid island 识别；
- 固体破坏后重新检查断裂块；
- 动态刚体下落、平移、旋转；
- 静止/碰撞后烙回 `WorldGrid`；
- Godot 中鼠标拖拽、旋转刚体。

### 高效识别：dirty cells + flood fill

不每帧全图扫描刚体。`RigidBodySolver` 维护：

```cpp
std::vector<int32_t> dirty_cells;
```

只有这些事件会标记 dirty：

```text
画 rock
擦除 air
动态刚体烙回 grid
```

处理 dirty 时，只从 dirty cell 和其四邻域作为 seed 开始，对 `ROCK / GLASS` 做 BFS flood fill：

```text
seed solid cell
  -> flood fill 找到所在 solid island
  -> 如果 island 接触世界边界：认为 anchored/static
  -> 如果 island 不接触边界且大小不超过阈值：转成 RigidBody
```

因此复杂度主要是：

```text
O(被破坏/修改区域附近的连通固体大小)
```

而不是每帧 `O(width * height)`。

为了避免一次破坏触发超大地形 flood fill 太慢，当前有上限：

```cpp
max_dynamic_cells = 3000
max_flood_cells   = 12000
```

超过阈值的连通块会保持静态。

### RigidBody 数据

当前刚体保存：

```cpp
struct RigidBody {
    vector<BodyCell> cells;       // 局部像素，含 material
    float x, y;                   // 质心位置
    float angle;                  // 旋转角
    float vx, vy;
    float angular_velocity;
    AABB min/max;                 // broad phase / picking
};
```

每个 `BodyCell` 保存相对质心的局部坐标：

```cpp
lx, ly
material
```

渲染/碰撞时把局部点旋转回世界格子：

```text
world = center + R(angle) * local
cell = floor(world)
```

### 更新与碰撞

当前是轻量 grid collider：

```text
vy += gravity
position += velocity
angle += angular_velocity
rasterize cells
if any raster cell hits blocks_velocity or out of bounds:
    revert
    damp velocity/angular velocity
    if speed small: bake back to grid
```

动态刚体目前只和静态 grid collider 碰撞；刚体-刚体碰撞还没做。

### 刚体破坏

空气刷擦除时会同时调用：

```cpp
RigidBodySolver::destroy_circle()
```

它会删除动态刚体中落在圆内的局部 cell。当前第一版不会把被切开的动态刚体再 split 成多个刚体；只是减少 cell 并继续保持一个 body。后续可以对 body 内部 cell 再做一次局部连通分裂。

### Godot 控制

GDScript 新增：

```text
MMB 拖拽刚体
Shift + MMB 旋转当前拖拽刚体
```

对应 C++ GDExtension API：

```cpp
start_rigid_drag(x, y)
update_rigid_drag(x, y, rotate)
end_rigid_drag()
get_rigid_body_count()
```

### 当前限制 / TODO

1. **动态刚体暂时不作为 MAC pressure 的移动边界**
   - 动态块从 `WorldGrid` 中移除并由 overlay 渲染；
   - 烙回 grid 后才重新成为流体边界；
   - TODO：移动刚体 face velocity 边界：

```text
solid face velocity = rigid body velocity at face
v(point) = linear_velocity + angular_velocity x (point - center)
```

2. **刚体-刚体碰撞暂未实现**
   - 当前只有动态刚体 vs 静态 grid collider；
   - 后续可用 AABB broad phase + raster overlap 近似。

3. **动态刚体破坏暂不 split**
   - `destroy_circle()` 只删除 body 内部 cell；
   - 后续可对剩余 local cells 做连通分量检测，拆成多个 RigidBody。

4. **anchor 规则暂时简单**
   - 当前接触世界边界即 anchored；
   - 后续可加 `CELL_ANCHORED` flag 区分 bedrock / 可破坏 rock。

## Dynamic rigid occupancy / velocity fields and material interaction

本轮在 `WorldGrid` 中加入自由刚体投影到网格后的有效 cell 信息：

```cpp
rigid_body_id[i]
rigid_velocity_x[i]
rigid_velocity_y[i]
```

这些字段不是材料本身，而是动态刚体覆盖在 MAC/powder/gas 网格上的 overlay：

- `rigid_body_id != 0` 表示该 cell 当前被一个自由刚体占据；
- `rigid_velocity_x/y` 是该刚体在这个 cell 位置的局部速度；
- 对旋转刚体，速度按点速度计算：

```text
v(cell) = linear_velocity + angular_velocity × (cell_position - body_center)
```

### 动态刚体和流体/粉末的关系

当前实现的优先级是“让 moving solid 对其他材料可见”：

1. `WorldGrid::blocks_velocity()` 会把 `rigid_body_id != 0` 当作 moving solid。
2. `MacFluidSolver::cell_material_def()` 对动态刚体 cell 返回 solid boundary。
3. `is_liquid_cell()` / `is_pressure_active_cell()` 会排除动态刚体覆盖的 cell。
4. `PowderSolver::can_powder_enter()` 不允许沙子进入动态刚体 cell。

### 刚体移动时的挤出/弹飞

`RigidBodySolver::populate_dynamic_cells()` 每帧把自由刚体 rasterize 到网格，并为每个有效 cell 写入：

```cpp
rigid_body_id
rigid_velocity_x/y
```

如果刚体 cell 覆盖了液体、粉末或气体，先尝试把该材料挤到邻近非 solid / 非 rigid cell：

- 粉末：按刚体局部速度方向弹飞，写入初速度；
- 液体：尝试挤到邻近空气/气体/液体格，并保留 `volume/oil/toxic/temperature`；
- 气体：尝试挤到邻近空气/气体格。

`PowderSolver` 也会检查 `rigid_body_id`：如果某个沙子 cell 仍被动态刚体覆盖，会按 `rigid_velocity_x/y` 尝试弹出到无 rigid 标签的邻居，并继承一部分刚体速度。

### 仍然没有完成的强耦合 TODO

目前动态刚体已经能作为 moving solid occupancy 影响流体 active set 和 powder CA，但还没有把刚体速度作为 MAC pressure projection 的移动边界速度。也就是说现在 moving solid 近似为 blocked cell，而不是严格的：

```text
u_face = rigid_velocity_at_face · normal
```

后续应在 `MacFluidSolver` 的 face boundary 中加入：

```text
if face touches dynamic rigid:
    boundary velocity = rigid body velocity sampled at face center
```

这样液体接触运动刚体时才会更接近动量守恒/弹性碰撞。

### RigidBodyChunk 不再自动合并回 WorldGrid

刚体系统已改成更接近独立 `RigidBodyChunk` 的语义：

```cpp
RigidBodyChunk
  pixels / local cells
  transform: x, y, angle
  velocity: vx, vy, angular_velocity
  raster collider: local cells + AABB broad phase
  sleeping flag
  never automatically merge into WorldGrid on contact
```

之前动态块撞到静态地形、速度足够小时会 `bake_body_to_grid()`，这会把已经分离的刚体重新并入静态地形，导致后续 flood fill 可能把它和地形当成一个连通块。现在改为：

```text
碰到 static grid collider -> 回退、阻尼/反弹
速度很小时 -> sleeping = true
仍然保留为独立 rigid body overlay
不自动写回 WorldGrid
```

只有显式破坏、拖拽等操作会唤醒 sleeping body。

### 动态刚体有效 cell 和速度场

`WorldGrid` 维护动态刚体投影：

```cpp
rigid_body_id[i]
rigid_velocity_x[i]
rigid_velocity_y[i]
```

每帧 `RigidBodySolver::populate_dynamic_cells()` 会 rasterize 所有自由刚体，写入这些字段。旋转刚体的 cell 速度用：

```text
v_cell = linear_velocity + angular_velocity × (cell_position - center)
```

所以粉末/液体看到的是每个接触 cell 的局部速度，而不是整块刚体单一速度。

### 与粉末/液体/气体的当前交互

当动态刚体 cell 覆盖其他材料时：

1. 先尝试把被覆盖的液体/粉末/气体挤到邻近无 rigid tag 的 cell；
2. 被挤出的材料继承刚体局部速度；
3. 粉末如果仍处于 `rigid_body_id != 0` 的 cell，会在 `PowderSolver` 中按近似弹性碰撞弹出：

```text
v_after ≈ v_sand + 2 * (v_rigid - v_sand)
```

然后加一个很小的方向偏置，避免继续留在刚体标签里。

### 单个 solid cell 不转刚体

`create_body_from_island()` 现在跳过 `island_cells.size() <= 1`，即单个孤立 solid cell 不生成 RigidBodyChunk，避免大量单像素刚体拖慢模拟。

### 仍然保留的 TODO

- 现在 collider 是 raster collider，不是 Godot Physics `RigidBody2D + CollisionPolygon2D`。
- 后续如果转 Godot physics，需要把 pixel island 轮廓转成 polygon / convex decomposition。
- MAC 流体压力投影仍未使用 moving boundary velocity：现在动态刚体是 blocked overlay + 挤出材料；下一步应实现：

```text
u_face = rigid_velocity_at_face · normal
```

让液体和运动刚体的接触更接近动量守恒边界。

## 刚体冲量碰撞与测试场景

刚体现在不再只用“碰撞后回退 + 阻尼”的粗略方法。`RigidBodySolver` 给每个自由刚体维护：

```cpp
mass, inv_mass
inertia, inv_inertia
velocity: vx, vy
angular_velocity
```

每个 pixel cell 近似为单位质量，转动惯量用：

```text
I = Σ(local_x^2 + local_y^2 + 1/6)
```

其中 `1/6` 是单位方格绕自身中心的近似转动惯量。

### 自由刚体撞静态地形

每帧积分后，solver rasterize 刚体 cell。如果 cell 落入静态 solid 或越界，就生成接触点：

```text
contact point = cell world center
normal = 从静态障碍指向刚体自由空间的方向
```

然后使用刚体冲量公式：

```text
r = contact - center
v_contact = V + ω × r
vn = dot(v_contact, n)

j = -(1 + restitution) * vn /
    (inv_mass + cross(r,n)^2 * inv_inertia)

V += j * n * inv_mass
ω += cross(r, j*n) * inv_inertia
```

并加入库仑摩擦近似：

```text
jt = -vt / (inv_mass + cross(r,t)^2 * inv_inertia)
jt = clamp(jt, -friction*j, friction*j)
```

最后做一小步 position correction，把已经重叠的 raster cell 沿 contact normal 推出静态地形。

### 自由刚体 vs 自由刚体

目前实现了第一版动态刚体互撞：

1. AABB broad phase；
2. 两个刚体的 raster cell 如果落在同一个 grid cell，生成 pair contact；
3. normal 从 B 指向 A；
4. 用双刚体冲量公式同时更新两个刚体：

```text
relative_velocity = vA_contact - vB_contact
j = -(1 + e) * dot(relative_velocity, n) /
    (inv_massA + inv_massB
     + cross(rA,n)^2 * inv_inertiaA
     + cross(rB,n)^2 * inv_inertiaB)

VA += j*n*inv_massA
ωA += cross(rA, j*n)*inv_inertiaA
VB -= j*n*inv_massB
ωB -= cross(rB, j*n)*inv_inertiaB
```

这仍然是 raster collider，不是连续 SAT/GJK，所以高速或薄物体可能穿透；但已经能表现自由刚体之间的线速度交换和角速度变化。

### 测试方式

新增 Godot/C++ 方法：

```gdscript
world.generate_rigid_collision_test()
```

`project/mac_main.gd` 现在启动时默认生成一个刚体碰撞测试场景：

- 地面和左右墙是静态 rock；
- 两个自由刚体块在空中相向运动；
- 两者带初始角速度；
- 它们会先互撞，然后落地撞静态地形。

按键：

```text
T   重新生成刚体碰撞测试
Esc 回到原来的 basin 场景
MMB 拖拽刚体
Shift + MMB 旋转刚体
```

### 当前 TODO

- 现在动态刚体互撞使用 raster overlap contact，后续可以换成轮廓 polygon + SAT，减少卡格子和穿透。
- 还没有把 moving rigid boundary velocity 严格写进 MAC pressure projection；目前流体只看到 moving solid overlay 和挤出速度。
- 刚体对液体/沙子的反作用仍是近似，之后应把液体压力/粉末碰撞冲量累计回刚体。

## 刚体碰撞优化：边界随机采样 + sleeping AABB 唤醒

### 代码位置

主要实现位于：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
```

关键函数：

```cpp
RigidBodySolver::rebuild_boundary_samples()
RigidBodySolver::collect_contacts()
RigidBodySolver::resolve_body_pair_contacts()
RigidBodySolver::wake_sleeping_bodies_near_active()
RigidBodySolver::step()
```

### 游戏内生命周期位置

刚体求解发生在每帧 simulation step 的最前面：

```cpp
MacSimulation::step()
  rigid_solver.step(grid)
  powder_solver.step(grid)
  fluid_solver.step()
  gas_solver.step(grid)
  fire_solver.step(grid)
  reaction_solver.step(grid)
```

也就是说刚体先更新，把动态刚体 raster overlay 写入：

```cpp
world.rigid_body_id
world.rigid_velocity_x
world.rigid_velocity_y
```

后续粉末、流体、气体都会把这些动态刚体 cell 当作 moving solid / blocked cell。

### 为什么之前会卡

旧版动态刚体互撞做的是近似：

```text
for cell in bodyA.cells:
  for cell in bodyB.cells:
    if raster(cellA) == raster(cellB): contact
```

复杂度大约是：

```text
O(bodyA_pixels * bodyB_pixels * body_pairs * collision_iterations)
```

几个大刚体时会非常慢。

### 新算法：完整像素保留，碰撞只采样边界

每个刚体仍然保留完整：

```cpp
std::vector<BodyCell> cells;
```

完整 cells 继续用于：

```text
显示
动态刚体 overlay
挤出液体/粉末/气体
```

但是碰撞不再遍历全部 cell，而是新增：

```cpp
std::vector<int32_t> boundary_indices;
std::vector<int32_t> collision_sample_indices;
```

生成过程：

```text
1. 刚体创建或破坏后，调用 rebuild_boundary_samples()
2. 在 local grid 中找边界 cell：
   如果上下左右任意一个邻居不存在，则该 cell 是 boundary
3. 从 boundary_indices 中按固定比例做确定性随机采样
4. 默认 boundary_sample_ratio = 0.50
5. collision_sample_indices 最多保留 max_collision_samples = 160 个点
```

所以碰撞成本从：

```text
所有像素 × 所有像素
```

变成：

```text
采样边界点 × 采样边界点
```

对于一个 2000 像素刚体，如果边界约 200 点，50% 采样后约 100 点；互撞从约 400 万次检查下降到约 1 万次检查。

### 静态地形碰撞

静态地形碰撞函数：

```cpp
RigidBodySolver::collect_contacts()
```

现在只遍历：

```cpp
collision_sample_indices
```

对每个采样点：

```text
1. 计算采样点世界坐标
2. raster 到 grid cell
3. 如果该 cell 是 solid 或越界，则生成 contact
4. 用周围 solid 梯度估计接触法线
5. 交给 resolve_contacts() 做冲量修正
```

### 动态刚体互撞

动态互撞函数：

```cpp
RigidBodySolver::resolve_body_pair_contacts()
```

现在流程是：

```text
1. AABB broad phase
2. 如果 AABB 不重叠，直接跳过
3. 只遍历 A.collision_sample_indices 和 B.collision_sample_indices
4. 如果两个采样点 raster 到同一个 grid cell，则生成 pair contact
5. 用双刚体冲量公式同时修改二者线速度和角速度
```

### sleeping / inactive 逻辑

刚体没有被删除，而是进入 sleeping：

```cpp
body.sleeping = true;
body.vx = 0;
body.vy = 0;
body.angular_velocity = 0;
```

sleeping body：

```text
不积分
不主动碰撞
仍然作为动态刚体 overlay 显示和阻挡流体/粉末
```

每个刚体维护扩展唤醒盒：

```cpp
wake_min_x = min_x - wake_aabb_padding;
wake_min_y = min_y - wake_aabb_padding;
wake_max_x = max_x + wake_aabb_padding;
wake_max_y = max_y + wake_aabb_padding;
```

当前默认：

```cpp
wake_aabb_padding = 10;
```

每帧 `RigidBodySolver::step()` 开头调用：

```cpp
wake_sleeping_bodies_near_active();
```

其逻辑是：

```text
for sleeping_body:
  for moving_body:
    if moving_body.AABB overlaps sleeping_body.expanded_wake_AABB:
      sleeping_body.sleeping = false
```

也就是：静止刚体不会浪费积分和碰撞求解；只有活跃刚体经过它附近时才唤醒。

### 当前参数

```cpp
boundary_sample_ratio = 0.50f;
max_collision_samples = 160;
wake_aabb_padding = 10;
settle_speed = 0.080f;
settle_angular_speed = 0.010f;
```

### 当前局限

- 这是随机边界采样，不是精确连续碰撞；高速小物体仍可能漏检。
- 采样点越少越快，但碰撞越软/越容易穿透。
- 未来可以改成：边界分段采样、空间哈希、SAT polygon、或把 boundary raster 到临时 occupancy grid，把互撞从 O(sampleA * sampleB) 进一步降到 O(sampleA + sampleB)。

## 连续绘制刚体：2 + 左键 stroke 延迟识别

### 修复的问题

之前用岩石笔刷创建刚体时，每次 `paint_circle()` 都会立刻：

```text
画一小块 rock
mark_dirty_rect()
process_dirty()
flood fill 这一个小块
立刻从 WorldGrid 拆出一个 RigidBodyChunk
```

所以当鼠标拖动时，第一帧画出的圆已经被转换成自由刚体，下一帧新画的 rock 就无法和上一帧 rock 在 WorldGrid 中连通，结果只能得到很多小刚体，而不是一条连续刚体。

### 新生命周期

现在新增了“刚体绘制 stroke”模式：

```text
按住 2 选择 Rock
按住 LMB 开始 stroke
  begin_rigid_paint_stroke()
  暂停 RigidBodySolver::process_dirty()
  每帧只把 rock 连续写入 WorldGrid
松开 LMB
  end_rigid_paint_stroke()
  恢复 process_dirty()
  对整条累计 dirty 区域做一次 flood fill
  一次性生成连续 RigidBodyChunk
```

### 代码位置

GDScript 输入层：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
```

新增状态：

```gdscript
var rigid_paint_stroke := false
```

在 `_handle_mouse_paint()` 中：

```gdscript
if material == 1 and lmb_down:
    if not rigid_paint_stroke:
        rigid_paint_stroke = true
        world.begin_rigid_paint_stroke()

# 松开左键或切换材料：
world.end_rigid_paint_stroke()
```

Godot C++ API 层：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.cpp
```

新增绑定：

```cpp
begin_rigid_paint_stroke()
end_rigid_paint_stroke()
```

Simulation 组装层：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.cpp
```

新增状态：

```cpp
bool rigid_paint_stroke_active = false;
```

新增函数：

```cpp
MacSimulation::begin_rigid_paint_stroke()
MacSimulation::end_rigid_paint_stroke()
```

Rigid solver 层：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
```

新增开关：

```cpp
bool auto_process_dirty = true;
set_auto_process_dirty(bool)
is_auto_process_dirty()
```

`RigidBodySolver::step()` 中：

```cpp
if (auto_process_dirty) {
    process_dirty(grid);
}
```

所以 stroke 期间虽然 simulation 仍然运行，但 dirty rock 不会被提前拆成小刚体；直到 stroke 结束才统一识别。

### 游戏内测试方式

```text
按住 2 选择 rock
按住鼠标左键拖出一条连续岩石结构
松开鼠标左键
整条连续结构会作为一个 RigidBodyChunk 掉落/碰撞
```

如果想切回普通 basin：

```text
Esc
```

如果想测试刚体互撞：

```text
T
```

## 刚体碰撞第二轮优化：sleep、低频 broad phase、static chunk collider、hash narrow phase

### 目标

本轮优化解决两个问题：

1. 落地后的刚体长期保持 active，继续每帧积分/碰撞，浪费性能；
2. 动态刚体和动态刚体之间原来是采样点两两比较，且 static 地形没有进入统一 AABB/pair 管线。

### 代码位置

主要代码：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
```

关键结构：

```cpp
RigidBody
BoundarySample
StaticCollisionChunk
BodyPair
StaticPair
```

关键函数：

```cpp
RigidBodySolver::rebuild_boundary_samples()
RigidBodySolver::rebuild_static_collision_chunks()
RigidBodySolver::rebuild_broad_phase_pairs()
RigidBodySolver::collect_static_chunk_contacts()
RigidBodySolver::resolve_body_pair_contacts()
RigidBodySolver::resolve_static_pair_contacts()
RigidBodySolver::resolve_dynamic_body_contacts()
RigidBodySolver::update_sleep_state()
RigidBodySolver::step()
```

### 游戏内生命周期位置

刚体仍然在每帧 simulation 的最前面更新：

```cpp
MacSimulation::step()
  rigid_solver.step(grid)
  powder_solver.step(grid)
  fluid_solver.step()
  gas_solver.step(grid)
  fire_solver.step(grid)
  reaction_solver.step(grid)
```

### 每帧刚体生命周期

现在 `RigidBodySolver::step()` 的逻辑变成：

```text
1. clear_rigid_fields()
2. 如果不是绘制 stroke，process_dirty()
3. 对所有 active 且非 sleeping 的刚体积分：
     vy += gravity
     x/y/angle += velocity
     damping
     update_body_aabb()
4. 每 broad_phase_interval_steps 帧：
     rebuild_static_collision_chunks()
     rebuild_broad_phase_pairs()
     wake sleeping bodies
5. 每帧对缓存 pair 做 narrow phase：
     active_body_pairs -> active-active collision
     active_static_pairs -> active-static collision
6. 根据连续低速帧数 update_sleep_state()
7. populate_dynamic_cells()
```

### 检测频率

频率现在分成两层：

```cpp
broad_phase_interval_steps = 6;
```

假设 60 simulation step/s：

```text
AABB broad phase / wake / static chunk rebuild: 每 6 step，约 10Hz / 0.1 秒
Narrow phase / 冲量响应: 对已缓存 pair 每 step，约 60Hz / 1/60 秒
```

也就是说：

```text
低频雷达扫描：找哪些刚体/静态块可能碰撞
高频接触响应：只对已缓存 pair 做边界查询和冲量
```

### sleep / inactive 逻辑

注意：代码中没有把静止刚体设为 `active=false`，因为 `active=false` 表示删除/不显示。

静止刚体现在使用：

```cpp
body.sleeping = true;
body.vx = 0;
body.vy = 0;
body.angular_velocity = 0;
```

它仍然：

```text
显示
保留 cells / transform
写入 rigid overlay
阻挡液体/粉末/气体
等待 active 刚体唤醒
```

新增参数：

```cpp
settle_speed = 0.25f;
settle_angular_speed = 0.045f;
sleep_after_frames = 12;
```

逻辑：

```text
如果刚体处于候选接触区域，并且线速度/角速度都小于阈值：
    still_frames++
否则：
    still_frames = 0

still_frames >= sleep_after_frames:
    sleeping = true
```

### sleeping 唤醒

每次 broad phase 会检查 active 刚体和 sleeping 刚体的扩展 AABB：

```cpp
wake_aabb_padding = 10;
```

逻辑：

```text
for active body A:
  for sleeping body B:
    if A.AABB overlaps B.wake_AABB:
        B.sleeping = false
        B.still_frames = 0
        add active pair(A,B)
```

所以 sleeping body 不做每帧积分/碰撞，只在 active body 接近时被唤醒。

### 动态刚体边界采样：spatial bucket 均匀采样

`RigidBody` 仍然保留完整像素：

```cpp
std::vector<BodyCell> cells;
```

但碰撞只用：

```cpp
boundary_indices
collision_sample_indices
```

`rebuild_boundary_samples()` 做：

```text
1. 在刚体 local grid 中找边界 cell：上下左右有空邻居即为边界
2. 把边界 cell 按 local 空间 bucket 分组
3. 每个 bucket 选一个代表采样点
4. 如果超过 max_collision_samples，则再 stride 降采样
```

参数：

```cpp
boundary_sample_ratio = 0.50f;
max_collision_samples = 160;
dynamic_bucket_size = 4;
```

这比纯随机采样稳定：不会出现某一整条边刚好没采样点。

### active-active narrow phase：occupancy hash + 3×3 查询

动态刚体互撞现在不再做：

```text
sampleA × sampleB 全比较
```

而是：

```text
1. 把 body B 的采样点 raster 到 grid cell key
2. 对 key 排序，形成临时 occupancy hash
3. 遍历 body A 的采样点
4. 查询 A 点周围 3×3 cell 是否存在 B 采样点
5. 命中则生成 contact
6. 用双刚体冲量更新两个刚体线速度/角速度
```

复杂度从近似：

```text
O(sampleA * sampleB)
```

降低为：

```text
O(sampleB log sampleB + sampleA * 9 * log sampleB)
```

在样本数不大时非常稳定，且比全比较快得多。

### static 地形也进入同一套机制

本轮新增了 static collider chunk：

```cpp
struct StaticCollisionChunk {
    int min_x, min_y, max_x, max_y;
    bool has_solid;
    vector<BoundarySample> samples;
    vector<int64_t> sample_keys;
};
```

`rebuild_static_collision_chunks()` 每次 broad phase 运行：

```text
1. 把 WorldGrid 按 static_chunk_size=32 切成 chunk
2. 对每个 chunk 扫描 solid cell
3. 找 solid-air 边界 cell
4. 估计边界法线
5. 按 static_bucket_size=4 做空间桶采样
6. 存 sample_keys 供 narrow phase 查询
```

所以 static 不再只是“直接查 WorldGrid solid”，而是也有：

```text
AABB
边界 samples
hash keys
cached active_static_pairs
```

### active-static broad/narrow

每次 broad phase：

```text
for active body:
  for static chunk:
    if body.AABB overlaps chunk.AABB:
        active_static_pairs.add(body, chunk)
```

每帧 narrow phase：

```text
for active_static_pair:
  active body samples 查询 static chunk sample hash 的 3×3 邻域
  命中则生成 contact
  用 active-static 冲量更新 body
```

static 的物理语义是：

```text
inv_mass = 0
inv_inertia = 0
不移动
不唤醒
不接收反向冲量
```

目前代码上 active-static 复用 `resolve_contacts()`，等价于只修改动态刚体。

### 当前局限

- static chunk 现在每次 broad phase 全量重建，未来应改成 dirty chunk 局部重建；
- active-static 的法线来自 static boundary sample，已经比直接 grid 梯度稳定，但仍然是像素级近似；
- high-speed 物体仍可能穿透，后续可加入 swept AABB / substep；
- active_body_pairs 每 0.1 秒刷新，如果高速刚体在两次 broad phase 之间相遇，可能晚几个 frame 才进入 pair；可通过增大 AABB padding 或速度扩展 AABB 改进。

## 刚体 broad phase 第三轮：固定世界 bucket、static/sleeping 复用、active 临时构建

### 回答：bucket 位置是否固定？

是。动态刚体 broad phase 的 bucket 是固定世界空间网格：

```cpp
broad_phase_cell_size = 64;
bucket_x = floor(world_x / broad_phase_cell_size);
bucket_y = floor(world_y / broad_phase_cell_size);
bucket_key = (bucket_x, bucket_y);
```

bucket 不跟随物体移动。物体只是根据自己的 AABB 覆盖哪些固定 bucket。

### 为什么只动态构建 active bucket？

本轮改成三套结构：

```text
static  : 固定 StaticCollisionChunk，dirty 后才重建
sleeping: 持久 sleeping bucket，sleep/wake 后才重建
active  : 每次 broad phase 临时构建 active bucket
```

原因：

```text
static 地形不动，没必要每次重建；
sleeping 刚体不动，没必要每次重建；
active 刚体每帧移动/旋转，所以每 0.1 秒临时重建最简单可靠。
```

### 代码位置

主要实现：

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
```

新增/修改字段：

```cpp
static_chunks_x, static_chunks_y
static_chunks_dirty
sleeping_bucket_keys
sleeping_bucket_body_ids
sleeping_buckets_dirty
broad_phase_cell_size = 64
sleep_after_seconds = 0.20f
fixed_step_seconds = 1.0f / 60.0f
RigidBody::still_time
```

关键函数：

```cpp
ensure_static_collision_chunks()
rebuild_static_collision_chunks()
rebuild_sleeping_body_buckets()
rebuild_broad_phase_pairs()
wake_sleeping_bodies_without_support_or_with_velocity()
body_has_static_support()
update_sleep_state()
```

### Static chunk 复用

static 现在不是每次 broad phase 无条件重建，而是：

```cpp
ensure_static_collision_chunks(grid)
```

只有在：

```text
static_chunks_dirty == true
world size / chunk grid 改变
```

才调用：

```cpp
rebuild_static_collision_chunks(grid)
```

`mark_dirty_cell()` 会在岩石/空气改变时设置：

```cpp
static_chunks_dirty = true;
```

这意味着 static 地形 bucket/chunk 是复用的，后续应进一步优化成只重建 dirty chunk，而不是 dirty 后全量重建。

### Sleeping bucket 复用

sleeping 刚体进入睡眠后，transform 不变，所以它的 wake AABB 覆盖的 bucket 也不变。

本轮新增持久 sleeping bucket：

```cpp
sleeping_bucket_keys
sleeping_bucket_body_ids
```

`rebuild_sleeping_body_buckets()` 只在：

```text
刚体 sleep
刚体 wake
刚体被破坏/dirty 唤醒
```

之后通过：

```cpp
sleeping_buckets_dirty = true
```

触发重建。

查询 sleeping body 时：

```text
active body 用自己的扩展 AABB 算覆盖哪些 fixed bucket
在 sleeping_bucket_keys 里 lower_bound 查询同 bucket 的 sleeping body id
再做 wake AABB 精确判断
命中则唤醒 sleeping body 并加入 active_body_pairs
```

### Active bucket 临时构建

每次 broad phase：

```cpp
std::vector<BucketEntry> active_entries;
```

只把 active 且非 sleeping 的刚体插入固定世界 bucket：

```text
for active body:
  for bucket overlapped by body.AABB + padding:
    active_entries.push(bucket_key, body_id)
```

然后：

```text
sort(active_entries by bucket_key)
for each bucket group:
  只比较同 bucket 内的 body pair
  如果 AABB overlap，加入 temp_pair_keys
sort + unique temp_pair_keys
生成 active_body_pairs
```

这把 active-active broad phase 从全局两两检查：

```text
O(N^2)
```

变成：

```text
O(N log N + local pairs)
```

### Static 查询不走 active bucket

static 已经是 32×32 chunk grid：

```cpp
static_chunk_size = 32;
```

所以 active-static broad phase 直接由 active body AABB 算 chunk 范围：

```cpp
min_cx = (body.min_x - padding) / static_chunk_size;
max_cx = (body.max_x + padding) / static_chunk_size;
```

然后直接访问：

```cpp
static_chunks[cy * static_chunks_x + cx]
```

不遍历所有 static chunk。

### Sleep 改成固定时间

之前是：

```text
still_frames >= sleep_after_frames
```

现在改成固定模拟时间：

```cpp
fixed_step_seconds = 1.0f / 60.0f;
sleep_after_seconds = 0.20f;
RigidBody::still_time += fixed_step_seconds;
```

逻辑：

```text
如果 near_contact && speed < settle_speed && angular_speed < settle_angular_speed:
    still_time += fixed_step_seconds
else:
    still_time = 0

if still_time >= sleep_after_seconds:
    sleeping = true
    velocity = 0
    sleeping_buckets_dirty = true
```

### Sleeping body 自动唤醒

本轮新增：

```cpp
wake_sleeping_bodies_without_support_or_with_velocity(grid)
```

每帧 broad phase 之前调用。

如果 sleeping body：

```text
速度/角速度意外大于阈值
或者 body_has_static_support(grid, body) == false
```

则：

```cpp
sleeping = false;
still_time = 0;
sleeping_buckets_dirty = true;
```

这样如果地面被挖掉，睡眠刚体会重新醒来并受重力下落。

### 碰撞法线和碰撞点是否用完整刚体？

不是。当前 narrow phase 使用的是：

```cpp
collision_sample_indices
```

也就是边界空间桶采样点。完整 `cells` 仍然用于：

```text
显示
rigid overlay
挤出液体/粉末/气体
AABB 计算
```

但碰撞接触点和法线来自：

```text
动态-动态：双方边界采样点的 3×3 occupancy hash 命中
动态-static：动态边界采样点查询 static chunk 边界 sample hash
```

所以碰撞不是用所有像素算，而是边界采样。

### 当前频率

```text
active bucket / sleeping query / static pair query: 每 6 step，约 0.1 秒
active_body_pairs / active_static_pairs narrow phase: 每 step，约 1/60 秒
sleep 支撑检测: 每 step
```

### 当前 TODO

- static_chunks_dirty 现在仍是 dirty 后全量重建，下一步可以改成 dirty chunk 局部重建；
- sleeping bucket 当前采用重建式缓存，不是精确 insert/remove；数量大时可以改成 lazy remove + 定期 compact；
- 高速刚体仍需要 swept AABB / substep 防穿透；
- active bucket cell size 现在固定 64，之后可以根据平均刚体尺寸自适应。

## Sleep 支撑检测合并进 broad phase

### 修改结论

`sleeping` 刚体的支撑检测现在不再每 step 单独运行，而是合并到 0.1 秒一次的 broad phase 维护阶段：

```text
每 broad_phase_interval_steps = 6 个 simulation step：
  ensure_static_collision_chunks()
  wake_sleeping_bodies_without_support_or_with_velocity()
  rebuild_sleeping_body_buckets() if dirty
  rebuild active temporary bucket
  rebuild active_body_pairs
  rebuild active_static_pairs
```

这样所有低频空间关系维护都集中在一个地方。

### 为什么不再设置 broad_phase_counter = 0

因为支撑检测现在就在 `rebuild_broad_phase_pairs()` 内部执行：

```cpp
RigidBodySolver::rebuild_broad_phase_pairs()
  ensure_static_collision_chunks(grid)
  wake_sleeping_bodies_without_support_or_with_velocity(grid)
  rebuild_sleeping_body_buckets()
  rebuild active bucket / pair cache
```

如果 sleeping body 在这一阶段失去支撑被唤醒，它会在同一轮 broad phase 里马上进入 active bucket 和 pair cache，所以不需要额外把 `broad_phase_counter` 清零。

`broad_phase_counter` 只负责控制低频 broad phase 的调用频率。

### 支撑检测只查下方

之前支撑检测曾检查：

```text
(x, y + 1)
(x - 1, y + 1)
(x + 1, y + 1)
(x, y)
```

现在移除了 `(x, y)`，因为 `(x, y)` 是当前采样点所在格，不是“下方支撑”。

当前只检查：

```text
(x, y + 1)
(x - 1, y + 1)
(x + 1, y + 1)
```

### 支撑来源

当前支撑定义是：

```text
support = static solid / world boundary OR other sleeping rigid overlay below
```

代码位于：

```cpp
RigidBodySolver::body_has_support(const WorldGrid &grid, const RigidBody &body)
```

逻辑：

```text
for boundary sample in body.collision_sample_indices:
  x,y = sample world cell
  check below-left / below / below-right:
    if static_solid_or_boundary(grid, sx, sy): support
    else if grid.rigid_body_id[below] belongs to another sleeping rigid body: support
```

注意：active rigid body 不作为稳定 sleep 支撑。原因是 active body 靠近 sleeping body 时会通过 active-vs-sleeping wake/AABB 逻辑唤醒它；只有 sleeping rigid body 才被视为稳定支撑，便于形成静止堆叠。

### Overlay 使用上一帧数据

为了让 support check 能查询 rigid overlay，`RigidBodySolver::step()` 不再在帧开头清空：

```cpp
grid.clear_rigid_fields()
```

因为 `populate_dynamic_cells()` 在帧末本来就会清空并重写 overlay。这样 broad phase 阶段可以读取上一帧的：

```cpp
grid.rigid_body_id
```

用于判断 below cell 是否有其他 sleeping rigid body。

### bucket 维护关系

当前 bucket 维护规则：

```text
static chunks:
  固定世界网格，static_chunks_dirty 时重建。

sleeping bucket:
  持久缓存。body sleep / wake / destroy / drag 时标记 sleeping_buckets_dirty。
  broad phase 中 lazy rebuild。

active bucket:
  不持久维护。每次 broad phase 临时从 active bodies 的 AABB 构建。
```

也就是说 static 和 sleeping 的空间 group 可以复用，active 由于一直移动，只低频临时构建。

## Rigid overlap sleep blocking and overlap response

### Code locations

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
```

Main functions:

```cpp
build_body_cell_hash()
boundary_overlaps_body_cells()
boundary_overlaps_static()
has_overlap_blocking_sleep()
add_body_overlap_contacts()
add_static_overlap_contacts()
update_sleep_state()
resolve_body_pair_contacts()
resolve_static_pair_contacts()
```

### Lifecycle position

In one simulation frame:

```text
1. move active rigid bodies by velocity / gravity
2. every ~0.1 s: rebuild broad phase pairs and wake unsupported sleepers
3. every frame: resolve cached active body/body and body/static pairs
4. after collision: update sleep state
5. repopulate rigid overlay into WorldGrid
```

The new overlap logic appears in steps 3 and 4:

- step 3: collision response adds robust overlap contacts;
- step 4: sleep timer is blocked if overlap still exists.

### Why boundary-vs-full-cells instead of all-vs-all

The expensive exact test would compare every occupied cell of body A with every occupied cell of body B. That is unnecessary. For penetration detection, it is enough to ask:

```text
Does any boundary cell of A rasterize into any occupied cell of B?
```

So the implemented test is:

```text
B hash = sorted unique world-cell keys of all B cells
A query = all boundary cells of A
if A_boundary_cell_key exists in B hash -> overlap
```

This is roughly:

```text
O(B_cells log B_cells + A_boundary log B_cells)
```

rather than:

```text
O(A_cells * B_cells)
```

For sleeping validation this uses the full boundary, not the random/sampled collision boundary, so a body cannot freeze while a sampled narrow phase missed an interpenetration.

### Sleep overlap blocking

`update_sleep_state(grid, body, had_contact)` now only accumulates `still_time` when all of these are true:

```text
near contact exists
linear speed < settle_speed
angular speed < settle_angular_speed
no overlap with static terrain
no overlap with relevant rigid bodies
```

Relevant rigid candidates are:

```text
1. counterpart bodies in active_body_pairs
2. nearby sleeping bodies from sleeping_bucket
```

Static overlap does not hash the whole static world. It directly checks the candidate body's full boundary against `WorldGrid`:

```text
for boundary cell of body:
  if rasterized cell is static solid or world boundary:
    overlap = true
```

If overlap exists, `still_time` is reset to 0, so the body stays active and collision resolution continues next frame.

### Collision overlap response

The normal narrow phase still uses the fast sampled-boundary contact test. After that, body/body collision additionally performs:

```text
A full-boundary vs B full-cell hash
B full-boundary vs A full-cell hash
```

For detected overlaps, it adds extra contacts. The overlap count is used as a cheap area estimate:

```cpp
penetration = clamp(0.65 + 0.045 * sqrt(overlap_count), 0.65, 1.65)
```

This mainly strengthens positional correction, instead of simply multiplying velocity impulse. That avoids explosive rebounds while still pushing embedded chunks apart more decisively.

Static collision also gets a full-boundary overlap supplement, but only inside the active static chunk's bounds. So it avoids repeating the same terrain contact for every static chunk.

### Important behavior change

A rigid body can now look visually still but remain active if it is still overlapping another rigid body or static terrain. This is intentional: sleep is only allowed after separation is confirmed by full-boundary overlap checks.

### Rigid static depenetration and tilted rendering note

After testing a tilted chunk embedded in the floor, static overlap response was strengthened in:

```cpp
RigidBodySolver::add_static_overlap_contacts()
```

Important change:

```text
if a rigid boundary pixel is inside static solid:
  search up to 10 cells for nearest open/non-solid cell
  use direction from solid to nearest open cell as escape normal
  add stronger overlap penetration, capped at 4 cells
```

This is still not a real force field. It is positional depenetration: deep static overlap is converted into stronger contact penetration so the solver moves the chunk out over collision iterations.

Rigid tilted rendering is handled in C++, not Godot. The pipeline is:

```text
RigidBodySolver::draw_overlay_rgba()
  -> MacSimulation::fill_rgba_pixels()
  -> MacWorld::update_texture()
  -> Godot ImageTexture update and draw_texture_rect()
```

Godot only displays the final RGBA texture. It does not rotate sprites or draw separate rigid-body nodes.

The old renderer painted each rigid cell only at `floor(rotated_center)`, which caused holes/flicker on tilted chunks. It now rasterizes each body cell as a rotated unit square into the RGBA buffer. Rock shading also uses local cell coordinates instead of world coordinates, so the texture pattern no longer crawls over the body while it moves.

### Frame timing probes

Added in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
```

The debug UI now reports:

```text
last_sim_ms
last_fill_ms
last_texture_ms
```

Meaning:

```text
last_sim_ms:
  Time spent in sim.step(). This is the actual simulation side: rigid bodies,
  powder, liquid pressure/advection, gas, fire, and reactions.

last_fill_ms:
  Time spent converting simulation state into the CPU RGBA pixel buffer via
  MacSimulation::fill_rgba_pixels(), including fluid_solver.fill_rgba_pixels()
  and rigid_solver.draw_overlay_rgba(). If conservative rigid rasterization is
  expensive, it shows up here.

last_texture_ms:
  Time spent resizing/copying the PackedByteArray, updating the Godot Image, and
  calling ImageTexture::update/create_from_image(). If the bottleneck is CPU->GPU
  texture upload or Godot image update, it shows up here.
```

Important limitation: this is the last executed simulation tick. If `simulation_speed` causes multiple `step()` calls in one rendered frame, each step currently also calls `update_texture()`, so the real frame cost can be roughly the sum of several sim/fill/texture updates plus Godot UI rendering and vsync wait.

### Render once per frame after multiple simulation catch-up steps

`MacWorld::_process()` previously called public `step()` inside the catch-up loop. Public `step()` runs:

```text
sim.step()
update_texture()
queue_redraw()
```

So when FPS dropped and the accumulator requested multiple simulation steps, one rendered frame could upload the texture several times.

Now `_process()` calls internal:

```cpp
run_sim_step()
```

inside the catch-up loop, and only after all simulation catch-up steps are complete does it call:

```cpp
update_texture();
queue_redraw();
```

The UI also reports:

```text
frame sim N steps X ms
```

This exposes spiral-of-death cases where a slow frame causes several simulation steps to be executed in the next rendered frame. If `N` stays above 1 for many frames, the simulation is behind real time.

### Rigid solver hotspot optimization pass: id map, O(1) AABB, contact flag

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
```

Changes:

1. `find_body_by_id()` no longer linearly scans `bodies`.

```text
Before:
  each lookup = O(number_of_bodies)
  hot paths: collision pair resolving, broad phase, sleeping support, overlap checks

Now:
  maintain lazy id -> body array index cache
  lookup is average O(1)
  cache is marked dirty when bodies are pushed or erased
```

2. `update_body_aabb()` no longer calls conservative covered-cell rasterization.

```text
Before:
  collect_body_covered_cells()
  transform many cells
  push vector
  sort by cell key
  unique
  scan min/max

Now:
  recompute_mass_properties() stores local body bounds once after topology/recenter
  update_body_aabb() rotates only four local AABB corners
  then adds a one-cell conservative pad
```

This removes an O(N log N) allocation-heavy pass from the per-body/per-collision-iteration hot path.

3. Sleep contact detection no longer scans all active pair lists per body.

```text
Before:
  for each body:
    scan active_static_pairs
    scan active_body_pairs

Now:
  each body owns had_contact
  resolve_static_pair_contacts() / resolve_body_pair_contacts() set had_contact=true when actual contacts are found
  sleep update reads body.had_contact directly
```

Expected impact:

```text
large rigid piles:
  less id lookup overhead
  less AABB recomputation overhead
  less per-body pair-list scanning
```

Remaining known heavyweight area:

```text
resolve_body_pair_contacts() still rebuilds covered-cell hashes per pair and per collision iteration.
The next optimization should cache covered cells / full cell keys once per body per simulation step and reuse them across pair solving, static overlap, populate overlay, and rendering.
```

### Rigid sleep logic fix: contact no longer resets sleep timer every frame

Problem found:

```cpp
resolve_body_pair_contacts()
```

previously did this whenever a body/body contact existed:

```cpp
r_a.sleeping = false;
r_b.sleeping = false;
r_a.still_time = 0.0f;
r_b.still_time = 0.0f;
```

That made piles almost impossible to sleep: stacked bodies are always touching, so their sleep timer was cleared every frame.

Current behavior:

```text
body/body contact sets had_contact = true
active-active low-speed contact no longer clears still_time
sleeping bodies are only woken by a contacting body if that body is moving faster than the wake threshold
broad phase no longer wakes sleeping bodies only because their wake AABB overlaps an active body
```

Also relaxed sleep overlap blocking:

```text
small conservative-raster overlaps are tolerated for sleep
only overlap_count > 12 blocks sleep
```

This avoids the conservative rigid raster turning tiny resting contacts into permanent sleep blockers.

Debug UI now reports:

```text
rigid bodies: total | awake: N | sleeping: M
```

Code locations:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\src\rigid_body_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.cpp/.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.cpp/.h
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
```

### Rigid sleep support fix

Problem:

Sleeping still stayed at 0 because `update_sleep_state()` only accumulated `still_time` when `had_contact == true`.

A rigid body resting on the floor can be pushed to a non-penetrating position, so the next frame may have no collision contact even though the body is clearly supported. That reset `still_time` every frame.

Fix:

```text
can_accumulate_sleep = had_contact OR body_has_support(grid, body)
```

Also changed support detection:

```text
body_has_support() now uses full boundary_indices, not collision_sample_indices
below rigid overlay from any other active/sleeping body counts as support
```

And relaxed sleep thresholds:

```text
settle_speed: 0.25 -> 0.45
settle_angular_speed: 0.045 -> 0.080
```

This lets resting bodies and stacked bodies enter sleep even when they are not producing solver contacts every frame.

### Semi-Lagrangian velocity advection for MAC fluid

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.cpp
```

New helpers:

```cpp
sample_u_bilinear()
sample_v_bilinear()
```

`predict_velocity_explicit()` now starts with a semi-Lagrangian advection pass for the MAC velocity field:

```text
for every u face at world position (x, y + 0.5):
  face_velocity = (u_old(x,y), average surrounding v_old)
  source = face_position - dt * face_velocity
  u_advected(face) = bilinear_sample_u_old(source)

for every v face at world position (x + 0.5, y):
  face_velocity = (average surrounding u_old, v_old(x,y))
  source = face_position - dt * face_velocity
  v_advected(face) = bilinear_sample_v_old(source)
```

Then viscosity and gravity are applied on top of the advected velocity:

```text
u_tmp = u_advected + dt * viscosity * laplacian(u_advected)
v_tmp = v_advected + dt * (viscosity * laplacian(v_advected) + gravity)
```

Then the existing pressure projection runs:

```text
build_pressure_system()
solve_pressure_pcg()
apply_pressure_projection()
advect_mass_finite_volume()
```

Meaning:

```text
volume is still advected by the projected velocity field,
but now the velocity field itself is also transported by the flow.
```

This should preserve horizontal momentum much better for water guns, parabolic jets, and falling streams. It is the stable numerical form of the convective term `(u · ∇)u`.

Note: semi-Lagrangian advection is intentionally dissipative. It is stable, but it can blur/decay velocity over time. If stronger momentum preservation is needed later, the next step is BFECC/MacCormack velocity advection or finite-volume momentum advection.

### Active liquid region, early-out, and fluid phase timings

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
```

Problem fixed: when there is no liquid, the fluid solver used to still execute most passes over the full grid. Now each `MacFluidSolver::step()` first computes a liquid effective region:

```text
scan cells once:
  if material is liquid and volume > epsilon and not covered by rigid body:
    update liquid bounding box
    update max cell-centered speed from MAC u/v

pad = ceil(max_speed * dt * 1.5) + 3, at least 3 cells
active rect = liquid bounds expanded by pad and clamped to the world
active_region_mask[i] = true inside that rect
```

The solver keeps both:

```text
active_region_mask[i]  // full-cell boolean table for O(1) effective-region queries
active_min/max x/y     // loop bounds, so passes do not sweep the whole grid
```

Important distinction:

```text
active_region_mask = fluid work region for this frame
active_cells       = pressure-matrix nonzero rows for PCG
```

`active_cells` is still kept because PCG dot products and stencil SpMV should only iterate the pressure-active rows. It is not used as the general fluid-region representation.

Early-out behavior:

```text
if no liquid is found:
  skip predict/build/PCG/project/advect/clamp
  clear stale velocity fields once when transitioning from liquid -> no liquid
  report all fluid phase timings as 0.00 ms
```

Region-narrowed passes:

```text
predict_velocity_explicit()      // u/v semi-Lagrangian advection + viscosity/gravity only on active face ranges
build_pressure_system()          // pressure mask, alpha, rhs, stencil rows only inside active rect
apply_pressure_projection()      // pressure gradient only on active face ranges
advect_mass_finite_volume()      // next volume/oil/toxic, fluxes, outflow, writeback only inside active rect
clamp_velocities()               // damping/clamp and world velocity update only inside active rect
```

The padding uses `1.5 * max_speed * dt` so the region includes cells that liquid can plausibly reach in one step. This is still a rectangle, not a sparse cell list, so it is simple and cache-friendly. Later, chunk activation can replace the one full scan with dirty/active chunks.

Timing instrumentation added:

```text
get_last_predict_ms()
get_last_build_ms()
get_last_pcg_ms()
get_last_project_ms()
get_last_advect_ms()
get_last_clamp_ms()
get_last_step_ms()
```

Godot UI now prints all these values with two decimals:

```text
fluid phases: predict | build | pcg | project | advect | clamp | total
```

The UI also prints the active fluid rectangle, padding, and max speed. This is intended to decide later whether expensive fluid phases should be scheduled across multiple rendered frames. The current implementation does not yet split pressure/decompression over a 0.05s cadence; it only exposes the per-phase timings needed to do that safely without changing the physical stepping blindly.

### Backtraced pressure-active threshold after velocity advection

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.cpp
```

Issue: after adding semi-Lagrangian velocity advection, the velocity field is transported before pressure projection, but `volume` is still advected later by `advect_mass_finite_volume()`. If pressure activity only tests the current cell volume,

```text
volume^n(x) > pressure_active_mass
```

then a fast horizontal stream can be time-layer shifted: velocity already says liquid is arriving at `x`, while the current volume at `x` is still below threshold. The pressure solve then misses that destination cell for one step.

Fix: pressure activity now uses an OR gate:

```text
current_active = current cell is pressure-solved and volume^n(x) > threshold
source_active  = volume^n(x - dt * u_tmp_cell(x)) > threshold
pressure_active = current_active OR source_active OR internal_liquid_current
```

The source sample is a cell-centered bilinear sample of pressure-solved liquid volume. Samples outside the grid, inside solids, or inside rigid-body-covered cells contribute zero.

Density consistency: if an air/low-volume destination cell becomes pressure-active only because its backtraced source contains liquid, `cell_density()` uses liquid density instead of air density. This prevents

```text
alpha = dt / rho_air
```

from becoming too large and creating an over-strong projection impulse at incoming fronts.

Volume correction consistency: the over-compression source term now uses

```text
pressure_volume = max(current_volume, backtraced_source_volume)
s = density_correction_strength * max(pressure_volume - target_mass, 0) / dt
```

while the internal underfill term still uses current cell volume and still requires `is_internal_liquid_cell()`. This avoids making free-surface destination cells artificially suck liquid inward.

Important dt note: the backtrace displacement currently uses the same solver `dt` as velocity advection:

```text
source = cell_center - dt * u_tmp_cell
```

If pressure/decompression is later split onto a separate 0.05s cadence or substep, this backtrace must use that projection/advection substep's effective dt rather than wall-clock frame time.

### Sparse active fluid indices for advection and pressure passes

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.h
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.cpp
```

The first active-region optimization used a padded bounding rectangle. That fixed full-grid passes, but it was still too coarse when liquid was sparse or split into several disconnected blobs: advection and velocity passes still visited every cell inside the rectangle, including large air gaps.

The fluid solver now uses a dual representation:

```cpp
active_region_mask[i]       // O(1) membership query for cells
active_region_indices[]     // compact traversal list for cells
u_active_face_mask[idx]     // O(1) membership query for u faces
v_active_face_mask[idx]     // O(1) membership query for v faces
active_u_faces[]            // compact traversal list for u faces
active_v_faces[]            // compact traversal list for v faces
```

Construction per fluid step:

```text
1. Clear previous masks by iterating previous indices, not by full-grid fill.
2. Scan material/volume once to find liquid source cells and max speed.
3. pad = ceil(max_speed * dt * 1.5) + 3, minimum 3.
4. For every liquid source cell, mark all cells in its local padded square.
5. When a cell is first marked, push it into active_region_indices.
6. For every active cell, mark its two adjacent u faces and two adjacent v faces.
7. When a face is first marked, push it into active_u_faces / active_v_faces.
```

This keeps both advantages:

```text
mask    -> fast random query: should this cell/face participate?
indices -> fast traversal: only visit active sparse cells/faces, not the whole padded rectangle
```

Passes changed to use indices instead of the coarse rectangle:

```text
predict_velocity_explicit()      -> active_u_faces / active_v_faces
build_pressure_system()          -> active_region_indices + active face lists
apply_pressure_projection()      -> active_u_faces / active_v_faces
advect_mass_finite_volume()      -> active_region_indices + active face lists
clamp_velocities()               -> active_u_faces / active_v_faces
update_world_velocity_field()    -> active_region_indices
```

Advection now also uses the cell mask as a safety gate on each face:

```text
for a u face between left/right cells:
  skip if left or right is outside active_region_mask

for a v face between up/down cells:
  skip if up or down is outside active_region_mask
```

Because the active region is already expanded by the maximum one-step travel distance, skipping flux beyond the mask should not remove reachable flow; it prevents stale, uninitialized `next_mass/outflow` cells from being touched outside the sparse region.

Pressure solve still keeps a separate list:

```cpp
active_cells[]
```

This is not the general fluid active list. It is the compact list of nonzero rows in the pressure matrix used by PCG dot products and stencil SpMV.

Current remaining full-grid cost: finding liquid source cells still scans all cells once per fluid step. This is the next candidate for chunk-level optimization: maintain dirty/active chunks or a world-level liquid count/list so zero-liquid and sparse-liquid scenes can skip even that scan.

### 30 Hz fluid dt and budgeted phase scheduler

Implemented in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_fluid_solver.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
```

The fluid timestep is now treated as real seconds instead of an arbitrary tick scale:

```text
default dt = 1 / 30 s
gravity unit = cells / s^2
velocity unit = cells / s
```

Default tuning changed accordingly:

```text
dt      = 1/30
gravity = 120 cells/s^2
max_velocity clamp = 30 cells/s
```

`MacWorld::_process()` now uses `sim.get_dt()` as the target fluid interval:

```text
target_fluid_interval = fluid dt
fluid_budget_ms = 5.0 ms per rendered frame
```

Frame scheduling:

```text
fluid_accumulator += frame_delta * simulation_speed

while this frame's fluid work time < 5 ms:
  if no pending budgeted simulation step:
    if accumulator < dt: stop
    begin_budgeted_step()
    accumulator -= dt

  advance_budgeted_step(remaining_budget_ms)

  if the step completes:
    count one completed sim step and allow another if budget remains
  else:
    stop and continue this same pending step next rendered frame
```

The fluid solver itself is now a small phase job state machine:

```text
PREDICT -> BUILD -> PCG -> PROJECT -> ADVECT -> CLAMP -> done
```

Each call to `advance_step_job(budget_ms)` executes phases in order until the budget is consumed or the job completes. A phase is not interrupted halfway; if one phase costs more than the remaining budget, it may slightly overshoot for that frame. This keeps the implementation simple and avoids exposing half-finished inner loops.

`MacSimulation` wraps the budgeted fluid job with the existing material order:

```text
begin_budgeted_step():
  rigid_solver.step(grid)
  powder_solver.step(grid) if needed
  fluid_solver.begin_step_job()

when fluid job completes:
  resolve fluid/powder pressure interaction
  gas_solver.step(grid) if needed
  fire_solver.step(grid) if needed
  reaction_solver.step(grid)
```

So a simulation step can span multiple rendered frames, but post-fluid solvers only run after the fluid phase has completed. The texture is updated only after a complete simulation step, not after every partial fluid phase, to avoid rendering intermediate pressure/advection states.

The UI now displays:

```text
target fluid: 30.0 steps/s
budget: 5.00 ms/frame
```

and the `dt` slider label reports both seconds and Hz.

Future refinement: split PCG into iteration batches if `pcg` alone becomes larger than the 5 ms frame budget. The current scheduler splits by major phase only.

### Budgeted fluid job: responsive post-solver timing

Updated in:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.cpp
```

The first budgeted scheduler version waited for the fluid job to fully finish before running post-fluid systems:

```text
fluid complete -> fluid/powder interaction -> gas -> fire -> reaction
```

This minimized half-step reads, but it also delayed material interaction when a fluid step was split across rendered frames.

The timing is now changed for responsiveness:

```text
begin_budgeted_step():
  rigid_solver.step(grid)
  powder_solver.step(grid) if needed
  fluid_solver.begin_step_job()
  resolve_fluid_powder_interactions()
  gas_solver.step(grid) if needed
  fire_solver.step(grid) if needed
  reaction_solver.step(grid)

later rendered frames:
  fluid_solver.advance_step_job(remaining_budget)
  when fluid completes, only mark the budgeted step complete
```

Meaning: post-fluid systems run once per 30 Hz simulation tick immediately. If the budgeted fluid job is still pending, they read the last committed fluid state. If there is no liquid or the fluid job is already complete, they read the updated state. This trades a small amount of temporal lag in fluid/material coupling for faster interaction response and avoids stalling gas/fire/reaction behind a multi-frame fluid solve.

The fluid solver still does not expose partially computed pressure/advection buffers as committed volume; material systems see a coherent `WorldGrid`, either from the previous completed fluid step or from a just-completed one.


### Fluid frame budget adjusted to 4 ms

Updated in:

`	ext
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.h
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
` 

The budgeted fluid scheduler now uses:

`	ext
fluid_budget_ms = 4.0 ms per rendered frame
` 

This leaves more room for Godot rendering, rigid bodies, texture upload, UI, and other material solvers while still allowing multi-frame fluid phase jobs to progress incrementally.


### Debug slider panel moved to lower-left

Updated in:

`	ext
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
` 

The parameter slider panel is now positioned near the lower-left of the viewport and reduced from 470/440 px wide to 340/320 px so it blocks less of the left-side debug text and simulation view.


### Debug slider panel raised to avoid bottom clipping

Updated in:

`	ext
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
` 

The lower-left parameter panel was too close to the bottom and could be partially clipped. Its y position now uses iewport_height - 430 instead of iewport_height - 285, and the panel/slider widths were reduced to 310/290 px.

## 13. 2026-07-05：fill fraction 语义、RHS 修正与刚体挤液解耦

### 13.1 `volume_fraction` 不是物理质量

当前世界格子的液体标量字段命名为：

```txt
WorldGrid::volume_fraction
```

它表示一个 cell 的填充率 / 占有率，而不是物理质量。NS/MAC 求解中的速度 `u,v` 也不是“质量速度”，而是以 `cells / s` 为单位的体积流动速度；有限体积 advection 搬运的是填充体积。不同液体的密度通过 `WorldGrid::density` 和压力投影中的 `alpha_f = dt / rho_f` 进入，而不应该把 `volume_fraction` 当成密度。

### 13.2 压力 RHS 的 `s` 目标固定为填充率 1

压力方程仍写成：

```txt
U_new = U* - alpha grad(p)
div(U_new) = s
A p = s - div(U*)
```

其中 `s` 的体积修正目标现在固定为：

```txt
target_fill_fraction = 1.0
```

也就是说，过满 / 欠填的判定只看 cell 填充率是否偏离 1：

```txt
pressure_fill_fraction = max(current volume_fraction, backtraced pressure volume_fraction)
excess  = max(pressure_fill_fraction - 1, 0)
deficit = max(1 - current volume_fraction, 0)

s = density_correction_strength * excess / dt
if internal_liquid:
    s -= underfill_correction_strength * deficit / dt
```

这样做的目的：

- `volume_fraction > 1`：要求净流出，修正被压缩的体积。
- 内部 `volume_fraction < 1`：要求净流入，填补水体内部空洞。
- 自由表面 `volume_fraction < 1`：不做 underfill，避免空气边界持续吸水。
- 油/水/毒液的密度差只通过 `rho` 影响 `alpha`，不改变填充率目标。

### 13.3 刚体运动与液体挤出已经解耦

刚体 solver 现在只负责刚体自身运动、碰撞、sleep、以及把自由刚体 raster 成动态刚体 overlay：

```txt
RigidBodySolver::step()
  -> populate_dynamic_cells()
       writes WorldGrid::rigid_body_id / rigid_velocity_x / rigid_velocity_y
       records RigidCellOccupancy list
```

它不再直接搬运水。刚体与液体的交互放在组装层：

```txt
MacSimulation::run_pre_fluid_solvers()
  1. rigid_solver.step(grid)
  2. resolve_rigid_liquid_overlaps()
  3. powder_solver.step(...)
  4. fluid_solver.begin_step_job()
```

`resolve_rigid_liquid_overlaps()` 读取 `RigidBodySolver::get_current_occupied_cells()`。如果某个本帧移动刚体覆盖了液体 cell，就把该 cell 的液体 payload 通过局部 BFS 挤到附近可进入 cell：

```txt
preferred directions:
  left/up-left, left, right, right/up-right, down, up, down-left, down-right
max visits: 96 cells
max temporary fill after displacement: 2.0
```

搬运内容包括：

```txt
volume_fraction
oil phase volume
toxic amount
temperature
velocity inherited from rigid contact velocity
```

目标 cell 如果已有液体，则按填充体积混合；如果是 air/gas，则转成液体。最终压力投影会继续把临时 `volume_fraction > 1` 的区域推平。因此刚体运动本身和“刚体挤开水”的近似交互已经分层：刚体 solver 只产出本帧 occupied cells，MacSimulation 负责解释这些 overlap 对液体意味着什么。

### 13.4 2026-07-05 update: boundary-touch rigid-liquid displacement

Rigid-liquid displacement was changed from per-overlapped-cell BFS to a cheaper boundary-touch method.

Frame/lifecycle location:

```txt
MacSimulation::run_pre_fluid_solvers()
  rigid_solver.step(grid)
    RigidBodySolver::populate_dynamic_cells()
      writes rigid overlay
      records RigidCellOccupancy { x, y, body_id, vx, vy, moving, boundary }
  MacSimulation::resolve_rigid_liquid_overlaps()
```

Algorithm:

```txt
1. Only moving rigid occupied cells are considered.
2. Only boundary occupied cells perform liquid-touch tests.
3. If a moving boundary cell sees liquid in its 3x3 neighborhood,
   mark that rigid body as liquid-touching.
4. For that boundary cell, collect non-rigid, non-solid neighbor targets
   in a 2-cell Manhattan neighborhood.  Air, gas, and liquid cells are valid.
5. Deduplicate targets per body.
6. For every occupied cell of marked bodies:
   if the underlying grid material is liquid:
       move that liquid directly into a pseudo-random order of the body's
       collected boundary targets.
```

This avoids maintaining liquid chunk indices and avoids rebuilding liquid chunk hashes.  The broad phase is simply the active/moving rigid boundary itself: if a body boundary never touches liquid, none of that body's interior occupied cells are tested for liquid displacement.

The displacement is intentionally approximate and Noita-like rather than physically exact:

```txt
max temporary target fill = 2.0
phase volumes moved = volume_fraction / oil / toxic
target velocity receives the rigid contact velocity by volume-weighted mixing
```

Pressure projection then removes the temporary overfill in later fluid phases.

### 13.5 2026-07-05 update: rigid-liquid splash impulse

Displaced liquid now receives an initial velocity impulse instead of only being moved volumetrically.

Parameter exposed to Godot/GDScript:

```text
rigid_liquid_impulse_strength
```

Code locations:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_simulation.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\src\mac_world.h/.cpp
D:\AICode\Emergence\Noita\NoitaCppExtension\project\mac_main.gd
```

For each displaced liquid parcel, the target velocity uses:

```text
displacement_dir = normalize(target_cell - overlapped_rigid_cell)
rigid_speed = length(rigid_cell_velocity)
splash_speed = rigid_speed * rigid_liquid_impulse_strength

displaced_velocity = rigid_cell_velocity + displacement_dir * splash_speed
```

Then the target cell velocity is volume-weighted with any liquid already there.  Physically this is an approximate momentum-transfer / splash-strength coefficient: larger values make water splash farther away from the rigid body; zero means pure displacement without extra outward impulse.

Godot UI adds a compact slider:

```text
rigid-liquid splash impulse: 0.00 - 3.00, default 0.45
```

The debug panel was also slightly shrunk to make room for the extra control.

### 13.6 2026-07-05 update: biased random target direction for rigid-liquid displacement

The rigid-liquid displacement target selection is now vertically biased:

```text
30% of displaced-liquid events prefer targets above the overlapped cell
70% of displaced-liquid events prefer targets below the overlapped cell
```

In screen coordinates this means:

```text
above: target_y < source_y
below: target_y > source_y
```

Implementation detail:

```text
prefer_above = hash(source_cell, body_id) % 10 < 3
```

The solver first tries targets on the preferred side in pseudo-random order.  If that side has no capacity, it falls back to any valid target so liquid is not deleted by a saturated preferred side.

## 14. 2026-07-05: threaded two-world simulation test

Before this change, `MacWorld` simulation was effectively single-threaded: `_process()` on the Godot main thread advanced `MacSimulation`, then filled RGBA pixels, then uploaded the texture.

A test-only threaded mode was added to `MacWorld`:

```text
MacWorld::set_threaded_simulation_enabled(true)
```

When enabled:

```text
worker std::thread:
  lock sim mutex
  sim.step()
  unlock
  sleep until next sim dt

Godot main thread:
  lock sim mutex briefly
  fill_rgba_pixels()
  unlock
  upload ImageTexture and draw
```

Important limitation: this is a coarse per-world thread test.  It does not yet split one large world into chunks.  Instead, it verifies that two independent simulation regions can run on two separate CPU threads while the main thread only renders their latest state.

New test scene:

```text
D:\AICode\Emergence\Noita\NoitaCppExtension\project\threaded_test.tscn
D:\AICode\Emergence\Noita\NoitaCppExtension\project\threaded_test.gd
```

The scene creates two `MacWorld` instances side by side and enables threaded simulation for both.  Each world owns its own `MacSimulation` and its own worker thread.  This is the first stepping stone toward future chunk-based multithreading:

```text
current test:  world A -> thread A, world B -> thread B
future design: chunk group A -> worker A, chunk group B -> worker B, with border exchange
```

### 14.1 2026-07-06: step(max_time_ms) wrapper and simpler MacWorld scheduling

The budgeted simulation API has been wrapped so callers no longer need to manually call:

```cpp
begin_budgeted_step();
while (has_pending_budgeted_step()) {
    advance_budgeted_step(...);
}
```

New shape:

```cpp
bool MacSimulation::step(double max_time_ms = 1000000.0);
bool MacSimulation::has_pending_step() const;

bool MacFluidSolver::step(double max_time_ms = 1000000.0);
```

Semantics:

```text
step(max_time_ms):
  if no tick is pending, start one
  run current tick until either:
    - tick is complete -> return true
    - max_time_ms is consumed -> return false
```

This works because `MacFluidSolver::advance_step_job()` already contains the inner phase loop, phase order, timing checks, and resume-from-interruption state.  If `max_time_ms` is huge, one call finishes the whole tick; if it is small, later calls continue from the saved phase.

`MacWorld::_process()` non-thread mode now only decides whether a tick is due or already pending, then calls:

```cpp
completed = sim.step(remaining_budget_ms);
```

Thread mode continues to use:

```cpp
sim.step(); // default huge budget: complete one full tick in the worker thread
```
