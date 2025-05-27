### 圆锥螺旋线任务的工具坐标系变换总结

#### 1. **核心变换链**
```
基坐标系{B} → 法兰坐标系{F} → 工具坐标系{T}
```

#### 2. **关键变换公式**
| 变换 | 数学表示 | 物理意义 |
|------|----------|----------|
| **工具位姿** | \( \mathbf{T}_{BT} = \begin{bmatrix} \mathbf{R}_{BT} & \mathbf{p}_{BT} \\ \mathbf{0} & 1 \end{bmatrix} \) | 工具末端在基坐标系中的位姿 |
| **法兰位姿** | \( \mathbf{T}_{BF} = \mathbf{T}_{BT} \cdot \mathbf{T}_{TF}^{-1} \) | 通过工具位姿反推法兰位姿 |
| **工具偏移** | \( \mathbf{T}_{TF} = \begin{bmatrix} \mathbf{I} & \mathbf{p}_{TF} \\ \mathbf{0} & 1 \end{bmatrix} \) | 工具坐标系在法兰系中的固定偏移 |

#### 3. **实际计算步骤**
1. **计算工具位姿**：
   ```python
   # 位置计算（圆锥螺旋线参数方程）
   tool_pos = [x0 + r*cosθ, y0 + r*sinθ, z0 + h*t]
   
   # 姿态计算（Z轴指向圆锥轴线）
   z_tool = normalize([vertex_x - x, vertex_y - y, -h*(1-t)])
   x_tool = cross(global_y, z_tool) if |z_tool.z|<0.99 else cross(global_x, z_tool)
   y_tool = cross(z_tool, x_tool)
   R_BT = matrix_from_basis(x_tool, y_tool, z_tool)
   ```

2. **反算法兰位姿**：
   ```python
   # 位置反算
   flange_pos = tool_pos - R_BT @ tool_offset
   
   # 姿态继承（假设工具无额外旋转）
   R_BF = R_BT  
   ```

#### 4. **特性验证条件**
| 验证项 | 检查方法 | 合格标准 |
|--------|----------|----------|
| 偏移量守恒 | \( \| \text{tool\_pos} - \text{flange\_pos} \| \equiv \| \text{tool\_offset} \| \) | 误差<0.1mm |
| 姿态连续性 | 检查相邻帧的 \( \Delta R = \| R_{t} - R_{t-1} \|_F \) | \( \Delta R < 0.05 \) |
| 奇异点规避 | 监控pitch角 \( \theta_p \) | \( |\theta_p| < 85^\circ \) |

#### 5. 可视化关键点
```python
# 绘制坐标系关系（示例帧）
draw_frame(tool_pos, R_BT, color='red')   # 工具坐标系（红）
draw_frame(flange_pos, R_BF, color='blue') # 法兰坐标系（蓝）
plot_connection_line(tool_pos, flange_pos) # 偏移量连线
```