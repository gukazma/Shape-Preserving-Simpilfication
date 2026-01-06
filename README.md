# Shape-Preserving Simplification

基于 CGAL 的高质量网格简化工具，使用 Garland-Heckbert (QEM) 算法，支持多材质纹理烘焙。

## 功能特性

- **高质量简化**: 使用 Garland-Heckbert 二次误差度量算法
- **形状保持**: 支持有界法向变化滤波器，保持模型特征
- **多格式支持**: 支持 OBJ、PLY、OFF、STL 格式
- **纹理处理**:
  - UV 坐标传递模式（保持原始纹理）
  - 纹理烘焙模式（生成新纹理图集）
- **多材质支持**: 自动处理多材质 OBJ 模型
- **并行加速**: 使用 OpenMP 加速纹理烘焙

## 编译

### 依赖项

- CMake 3.16+
- C++17 编译器
- vcpkg 包管理器
- 依赖库：CGAL、Eigen3、Boost、stb_image

### 编译步骤

```bash
# 配置
cmake -B Builds -S . -DCMAKE_TOOLCHAIN_FILE=[vcpkg路径]/scripts/buildsystems/vcpkg.cmake

# 编译
cmake --build Builds --config Release
```

## 使用方法

### 基本语法

```bash
sps_simplify <输入文件> [输出文件] [选项]
```

### 命令行选项

| 选项 | 说明 | 默认值 |
|------|------|--------|
| `-r <ratio>` | 简化比例 (0.0-1.0) | 0.1 |
| `-f <count>` | 目标面数（覆盖比例设置） | - |
| `-g` | 使用 Garland-Heckbert (QEM) 策略 | 启用 |
| `-l` | 使用 Lindstrom-Turk 策略 | - |
| `-b` | 启用有界法向变化滤波器 | 启用 |
| `-n` | 禁用有界法向变化滤波器 | - |
| `-t [path]` | 启用纹理烘焙（可指定纹理路径） | - |
| `-ts <size>` | 纹理烘焙输出尺寸 | 2048 |
| `-v` | 详细输出 | 启用 |
| `-h` | 显示帮助 | - |

### 使用示例

#### 1. 基本简化（保留10%面数）

```bash
sps_simplify model.obj simplified.obj -r 0.1
```

#### 2. 指定目标面数

```bash
sps_simplify model.obj simplified.obj -f 10000
```

#### 3. 简化并烘焙纹理

```bash
# 自动检测纹理文件
sps_simplify model.obj simplified.obj -r 0.1 -t

# 指定纹理文件
sps_simplify model.obj simplified.obj -r 0.1 -t texture.jpg
```

#### 4. 多材质模型处理

对于包含多个材质的 OBJ 模型（带 MTL 文件），程序会自动：
- 检测并加载所有材质纹理
- 将多个纹理烘焙为单一纹理图集
- 生成新的 MTL 文件

```bash
sps_simplify multi_material.obj simplified.obj -r 0.1 -t
```

#### 5. 自定义纹理尺寸

```bash
sps_simplify model.obj simplified.obj -r 0.1 -t -ts 4096
```

## 纹理处理模式

### UV 传递模式（默认）

不使用 `-t` 选项时，程序会将原始模型的 UV 坐标传递到简化后的模型：
- 保持原始纹理文件不变
- 适用于纹理质量要求高的场景
- 可能在简化后出现 UV 接缝问题

### 纹理烘焙模式

使用 `-t` 选项时，程序会：
1. 使用 xatlas 生成新的 UV 布局
2. 从原始模型采样颜色
3. 生成新的烘焙纹理（JPEG 格式，质量 75）
4. 创建对应的 MTL 文件

优点：
- 消除多材质合并问题
- 优化 UV 利用率
- 减少纹理接缝

## 输出文件

简化后会生成以下文件：

| 文件 | 说明 |
|------|------|
| `output.obj` | 简化后的网格模型 |
| `output.mtl` | 材质文件（如启用纹理烘焙） |
| `output_baked.jpg` | 烘焙后的纹理（如启用纹理烘焙） |

## 性能建议

1. **大模型处理**: 对于百万面级别的模型，建议使用纹理烘焙模式以获得更好的纹理质量
2. **内存使用**: 纹理烘焙会占用较多内存，建议有足够的 RAM
3. **并行加速**: 确保编译时启用 OpenMP 以加速纹理烘焙过程

## 示例工作流

```bash
# 1. 查看原始模型信息
sps_simplify model.obj -h

# 2. 简化到 10% 并烘焙纹理
sps_simplify model.obj model_lod1.obj -r 0.1 -t

# 3. 简化到 5%
sps_simplify model.obj model_lod2.obj -r 0.05 -t

# 4. 简化到 1%
sps_simplify model.obj model_lod3.obj -r 0.01 -t
```

## 常见问题

### Q: 纹理出现黑边怎么办？
A: 程序已内置纹理膨胀算法（16次迭代）来填充接缝。如果仍有问题，可能是原始 UV 布局问题。

### Q: 简化后模型变形严重？
A: 尝试使用 `-b` 选项启用有界法向变化滤波器，或提高简化比例。

### Q: 多材质模型纹理错乱？
A: 确保使用 `-t` 选项启用纹理烘焙模式，程序会自动处理多材质合并。

## 许可证

MIT License
