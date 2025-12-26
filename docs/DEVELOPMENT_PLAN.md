# Shape-Preserving Simplification 研发方案

## 项目概述

基于论文《A Shape-Preserving Simplification Method for Urban Building Models》实现一个针对城市建筑模型的形状保持简化系统。

---

## 一、技术架构设计

### 1.1 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Shape-Preserving Simplification                       │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Input     │  │   Core      │  │  Algorithm  │  │   Output    │        │
│  │   Layer     │  │   Layer     │  │   Layer     │  │   Layer     │        │
│  ├─────────────┤  ├─────────────┤  ├─────────────┤  ├─────────────┤        │
│  │ • OBJ       │  │ • HalfEdge  │  │ • Bilateral │  │ • LOD Gen   │        │
│  │ • PLY       │  │ • Quadric   │  │   Filter    │  │ • OBJ/PLY   │        │
│  │ • OFF       │  │ • Priority  │  │ • Region    │  │ • Texture   │        │
│  │ • FBX       │  │   Queue     │  │   Growing   │  │   Atlas     │        │
│  │             │  │ • Spatial   │  │ • QEM       │  │ • Metrics   │        │
│  │             │  │   Index     │  │   Simplify  │  │   Report    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 核心数据结构

```cpp
// 半边数据结构
struct HalfEdge {
    int vertex;           // 指向的顶点索引
    int face;             // 所属面索引
    int next;             // 下一条半边
    int prev;             // 上一条半边
    int opposite;         // 对边
};

// 顶点结构
struct Vertex {
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    Eigen::Vector2d texCoord;
    Eigen::Matrix4d quadric;    // QEM矩阵
    int region;                  // 所属区域
    VertexType type;            // V1/V2/V3
    double weight;              // 权重 1/10/100
};

// 面结构
struct Face {
    std::array<int, 3> vertices;
    Eigen::Vector3d normal;
    int region;                  // 所属平面区域
    double area;
};

// 区域结构
struct PlanarRegion {
    int id;
    Eigen::Vector3d avgNormal;
    std::vector<int> faces;
    std::set<int> boundaryVertices;
    std::set<int> neighborRegions;
};
```

### 1.3 依赖库选型

| 库 | 版本 | 用途 | 必需性 |
|---|-----|-----|-------|
| **Eigen3** | 3.4+ | 矩阵运算、线性代数 | 必需 |
| **CGAL** | 5.5+ | 几何算法、空间索引 | 必需 |
| **OpenMesh** | 9.0+ | 网格数据结构 | 推荐 |
| **libigl** | 2.4+ | 备选网格库 | 可选 |
| **tinyobjloader** | 2.0+ | OBJ文件解析 | 必需 |
| **happly** | latest | PLY文件解析 | 必需 |
| **spdlog** | 1.11+ | 日志系统 | 推荐 |
| **CLI11** | 2.3+ | 命令行解析 | 推荐 |
| **Google Test** | 1.12+ | 单元测试 | 推荐 |

---

## 二、模块详细设计

### 2.1 模块一：网格滤波 (Bilateral Mesh Filtering)

**文件**: `src/filter/bilateral_filter.h/cpp`

```cpp
class BilateralMeshFilter {
public:
    struct FilterParams {
        double sigmaDistance = 1.0;     // 空间距离方差
        double angleThreshold = 25.0;    // 角度阈值(度)
        int maxIterations = 10;          // 最大迭代次数
        double convergenceThreshold = 1e-6;
    };

    // 主入口：执行双边滤波
    void filter(Mesh& mesh, const FilterParams& params);

private:
    // 步骤1：计算面法向
    void computeFaceNormals(Mesh& mesh);

    // 步骤2：双边滤波法向
    void filterNormals(Mesh& mesh, const FilterParams& params);

    // 步骤3：更新顶点位置
    void updateVertexPositions(Mesh& mesh);

    // 辅助：计算空间距离权重
    double computeSpatialWeight(const Face& fi, const Face& fj, double sigma);

    // 辅助：计算法向相似性权重
    double computeNormalWeight(const Face& fi, const Face& fj, double threshold);

    // 辅助：获取自适应邻域
    std::vector<int> getAdaptiveNeighborhood(const Mesh& mesh, int faceIdx, double angleThresh);
};
```

**核心算法**：
```cpp
// 法向滤波核心
Eigen::Vector3d BilateralMeshFilter::filterFaceNormal(
    const Mesh& mesh, int faceIdx, const FilterParams& params)
{
    const Face& fi = mesh.faces[faceIdx];
    auto neighbors = getAdaptiveNeighborhood(mesh, faceIdx, params.angleThreshold);

    Eigen::Vector3d weightedNormal = Eigen::Vector3d::Zero();
    double totalWeight = 0.0;

    for (int neighborIdx : neighbors) {
        const Face& fj = mesh.faces[neighborIdx];

        double alpha = computeSpatialWeight(fi, fj, params.sigmaDistance);
        double beta = computeNormalWeight(fi, fj, params.angleThreshold);
        double weight = fi.area * alpha * beta;

        weightedNormal += weight * fi.normal;
        totalWeight += weight;
    }

    return (weightedNormal / totalWeight).normalized();
}
```

---

### 2.2 模块二：形状检测 (Shape Detection)

**文件**: `src/detection/region_growing.h/cpp`, `src/detection/region_optimizer.h/cpp`

```cpp
class RegionGrowing {
public:
    struct GrowingParams {
        double angleThreshold = 30.0;    // 生长角度阈值
        int minRegionSize = 10;          // 最小区域面数
    };

    // 主入口：区域生长
    std::vector<PlanarRegion> detectRegions(const Mesh& mesh, const GrowingParams& params);

private:
    // 种子选择
    int selectSeedFace(const Mesh& mesh, const std::vector<bool>& grouped);

    // 区域扩展
    void growRegion(const Mesh& mesh, PlanarRegion& region,
                    std::vector<bool>& grouped, double angleThresh);

    // 检查面是否可合并
    bool canMergeFace(const Face& seed, const Face& candidate, double angleThresh);
};

class RegionOptimizer {
public:
    // 主入口：优化区域
    void optimize(Mesh& mesh, std::vector<PlanarRegion>& regions);

private:
    // 处理未分组面
    void handleUngroupedFaces(Mesh& mesh, std::vector<PlanarRegion>& regions);

    // 合并小区域
    void mergeSmallRegions(std::vector<PlanarRegion>& regions, int minSize);

    // 平滑边界
    void smoothBoundaries(Mesh& mesh, std::vector<PlanarRegion>& regions);
};
```

**区域生长算法**：
```cpp
std::vector<PlanarRegion> RegionGrowing::detectRegions(
    const Mesh& mesh, const GrowingParams& params)
{
    std::vector<PlanarRegion> regions;
    std::vector<bool> grouped(mesh.faces.size(), false);
    int regionId = 0;

    while (true) {
        int seedIdx = selectSeedFace(mesh, grouped);
        if (seedIdx < 0) break;  // 所有面已分组

        PlanarRegion region;
        region.id = regionId++;
        region.faces.push_back(seedIdx);
        grouped[seedIdx] = true;

        growRegion(mesh, region, grouped, params.angleThreshold);

        // 计算区域平均法向
        region.avgNormal = computeAverageNormal(mesh, region);

        regions.push_back(std::move(region));
    }

    return regions;
}
```

---

### 2.3 模块三：约束简化 (Constrained QEM Simplification)

**文件**: `src/simplify/qem_simplifier.h/cpp`, `src/simplify/constraint_manager.h/cpp`

```cpp
class ConstraintManager {
public:
    // 顶点分类
    void classifyVertices(Mesh& mesh, const std::vector<PlanarRegion>& regions);

    // 获取顶点权重
    double getVertexWeight(VertexType type) const {
        switch (type) {
            case VertexType::PLANAR:   return 1.0;    // V1
            case VertexType::EDGE:     return 10.0;   // V2
            case VertexType::CORNER:   return 100.0;  // V3
        }
    }

    // 检查边折叠是否有效
    bool isCollapseValid(const Mesh& mesh, int edgeIdx,
                         const std::vector<PlanarRegion>& regions);
};

class QEMSimplifier {
public:
    struct SimplifyParams {
        double targetRatio = 0.1;        // 目标简化率 (保留10%)
        int targetFaceCount = -1;        // 或指定目标面数
        bool preserveTexture = true;     // 保持纹理坐标
        bool preserveBoundary = true;    // 保持边界
    };

    // 主入口：执行简化
    void simplify(Mesh& mesh, const std::vector<PlanarRegion>& regions,
                  const SimplifyParams& params);

private:
    ConstraintManager constraintMgr_;

    // 初始化QEM矩阵
    void initializeQuadrics(Mesh& mesh);

    // 计算边折叠代价
    double computeEdgeCost(const Mesh& mesh, int edgeIdx);

    // 计算最优折叠位置
    Eigen::Vector3d computeOptimalPosition(const Mesh& mesh, int edgeIdx);

    // 执行边折叠
    void collapseEdge(Mesh& mesh, int edgeIdx, const Eigen::Vector3d& newPos);

    // 更新受影响的边代价
    void updateAffectedEdges(Mesh& mesh, int vertexIdx);
};
```

**QEM核心算法**：
```cpp
void QEMSimplifier::simplify(Mesh& mesh, const std::vector<PlanarRegion>& regions,
                             const SimplifyParams& params)
{
    // 1. 顶点分类并设置权重
    constraintMgr_.classifyVertices(mesh, regions);

    // 2. 初始化QEM矩阵
    initializeQuadrics(mesh);

    // 3. 构建优先队列
    std::priority_queue<EdgeCost, std::vector<EdgeCost>, std::greater<>> pq;
    for (int e = 0; e < mesh.edges.size(); e++) {
        double cost = computeEdgeCost(mesh, e);
        pq.push({e, cost});
    }

    // 4. 计算目标面数
    int targetCount = (params.targetFaceCount > 0)
        ? params.targetFaceCount
        : static_cast<int>(mesh.faces.size() * params.targetRatio);

    // 5. 迭代折叠
    while (mesh.faceCount() > targetCount && !pq.empty()) {
        auto [edgeIdx, cost] = pq.top();
        pq.pop();

        // 验证边折叠有效性
        if (!mesh.isEdgeValid(edgeIdx)) continue;
        if (!constraintMgr_.isCollapseValid(mesh, edgeIdx, regions)) continue;

        // 执行折叠
        Eigen::Vector3d newPos = computeOptimalPosition(mesh, edgeIdx);
        collapseEdge(mesh, edgeIdx, newPos);

        // 更新受影响的边
        updateAffectedEdges(mesh, edgeIdx);
    }
}

double QEMSimplifier::computeEdgeCost(const Mesh& mesh, int edgeIdx) {
    const Edge& edge = mesh.edges[edgeIdx];
    const Vertex& v1 = mesh.vertices[edge.v1];
    const Vertex& v2 = mesh.vertices[edge.v2];

    // 合并QEM矩阵
    Eigen::Matrix4d Q = v1.quadric + v2.quadric;

    // 计算最优位置
    Eigen::Vector3d optPos = computeOptimalPosition(mesh, edgeIdx);
    Eigen::Vector4d v(optPos.x(), optPos.y(), optPos.z(), 1.0);

    // 基础代价
    double baseCost = v.transpose() * Q * v;

    // 应用顶点权重约束
    double weight = std::max(v1.weight, v2.weight);

    return baseCost * weight;
}
```

---

### 2.4 模块四：纹理重映射 (Texture Remapping)

**文件**: `src/texture/texture_remap.h/cpp`

```cpp
class TextureRemapper {
public:
    struct RemapParams {
        int atlasSize = 2048;            // 纹理图集尺寸
        int padding = 2;                  // patch间距
        bool generateMipmaps = true;     // 生成mipmap
    };

    // 主入口：纹理重映射
    void remap(Mesh& mesh, const RemapParams& params);

private:
    // Patch分解
    std::vector<MeshPatch> decomposeToPatchs(const Mesh& mesh);

    // UV展开
    void parameterizePatch(MeshPatch& patch);

    // 纹理图集打包
    TextureAtlas packAtlas(const std::vector<MeshPatch>& patches, int atlasSize);

    // 重采样纹理
    void resampleTexture(MeshPatch& patch, const TextureAtlas& atlas);
};
```

---

## 三、项目目录结构

```
Shape-Preserving-Simplification/
├── CMakeLists.txt                    # 主CMake配置
├── README.md                         # 项目说明
├── LICENSE
│
├── docs/                             # 文档
│   ├── DEVELOPMENT_PLAN.md          # 本研发方案
│   ├── API.md                        # API文档
│   ├── ARCHITECTURE.md               # 架构文档
│   └── pdca/                         # PDCA文档
│       └── mesh-simplify/
│           ├── plan.md
│           ├── do.md
│           ├── check.md
│           └── act.md
│
├── include/                          # 公共头文件
│   └── sps/                          # Shape-Preserving-Simplification
│       ├── core/
│       │   ├── mesh.h
│       │   ├── half_edge.h
│       │   └── quadric.h
│       ├── filter/
│       │   └── bilateral_filter.h
│       ├── detection/
│       │   ├── region_growing.h
│       │   └── region_optimizer.h
│       ├── simplify/
│       │   ├── qem_simplifier.h
│       │   └── constraint_manager.h
│       ├── texture/
│       │   └── texture_remap.h
│       └── io/
│           ├── mesh_io.h
│           └── obj_loader.h
│
├── src/                              # 源代码
│   ├── core/
│   │   ├── mesh.cpp
│   │   ├── half_edge.cpp
│   │   └── quadric.cpp
│   ├── filter/
│   │   └── bilateral_filter.cpp
│   ├── detection/
│   │   ├── region_growing.cpp
│   │   └── region_optimizer.cpp
│   ├── simplify/
│   │   ├── qem_simplifier.cpp
│   │   └── constraint_manager.cpp
│   ├── texture/
│   │   └── texture_remap.cpp
│   ├── io/
│   │   ├── mesh_io.cpp
│   │   └── obj_loader.cpp
│   └── main.cpp                      # 命令行入口
│
├── tests/                            # 测试
│   ├── CMakeLists.txt
│   ├── test_mesh.cpp
│   ├── test_filter.cpp
│   ├── test_detection.cpp
│   ├── test_simplify.cpp
│   └── test_data/                    # 测试数据
│       ├── cube.obj
│       ├── building_simple.obj
│       └── building_complex.obj
│
├── examples/                         # 示例
│   ├── basic_simplify.cpp
│   ├── batch_process.cpp
│   └── lod_generation.cpp
│
├── third_party/                      # 第三方库
│   ├── eigen/
│   ├── tinyobjloader/
│   └── happly/
│
├── tools/                            # 工具脚本
│   ├── download_dependencies.py
│   ├── run_benchmarks.py
│   └── visualize_results.py
│
└── data/                             # 测试数据
    ├── input/
    └── output/
```

---

## 四、开发阶段规划

### Phase 1: 基础框架 [预估工作量: 中等]

**目标**: 搭建项目框架，实现基础数据结构

| 任务 | 详细内容 | 验收标准 |
|-----|---------|---------|
| 1.1 项目初始化 | CMake配置、依赖管理 | 能成功编译空项目 |
| 1.2 网格数据结构 | Vertex/Face/HalfEdge | 单元测试通过 |
| 1.3 模型I/O | OBJ/PLY读写 | 读取后再写出，内容一致 |
| 1.4 基础QEM | 标准QEM算法 | 能简化cube模型 |

**关键代码 - CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.16)
project(ShapePreservingSimplification VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(Eigen3 3.4 REQUIRED)
find_package(CGAL 5.5 REQUIRED)

# 头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 主程序
add_executable(sps_simplify ${SOURCES})
target_link_libraries(sps_simplify
    Eigen3::Eigen
    CGAL::CGAL
)

# 测试
enable_testing()
add_subdirectory(tests)
```

---

### Phase 2: 网格滤波 [预估工作量: 中等]

**目标**: 实现双边网格滤波

| 任务 | 详细内容 | 验收标准 |
|-----|---------|---------|
| 2.1 面法向计算 | 叉积计算、归一化 | 法向方向正确 |
| 2.2 双边权重 | 空间权重、法向权重 | 权重值范围正确 |
| 2.3 自适应邻域 | 角度阈值筛选 | 不跨越折痕 |
| 2.4 顶点更新 | 迭代更新位置 | 收敛且保持特征 |

**验证方法**:
- 输入带噪声的平面模型
- 滤波后平面区域法向应趋于一致
- 边缘和角点应保持锐利

---

### Phase 3: 形状检测 [预估工作量: 中等偏高]

**目标**: 实现区域生长和优化

| 任务 | 详细内容 | 验收标准 |
|-----|---------|---------|
| 3.1 区域生长 | 种子选择、扩展合并 | 平面区域正确分组 |
| 3.2 未分组面处理 | 合并到最近区域 | 无孤立面 |
| 3.3 小区域合并 | 阈值判断、合并策略 | 小区域被吸收 |
| 3.4 边界平滑 | 边界面重新分配 | 边界更平滑 |

**验证方法**:
- 简单建筑模型应检测出墙面、屋顶等区域
- 区域边界应清晰可辨
- 可视化区域分配结果

---

### Phase 4: 约束简化 [预估工作量: 高]

**目标**: 实现约束QEM简化

| 任务 | 详细内容 | 验收标准 |
|-----|---------|---------|
| 4.1 顶点分类 | V1/V2/V3分类逻辑 | 分类正确 |
| 4.2 权重系统 | 1/10/100权重 | 折叠顺序符合预期 |
| 4.3 代价函数 | 加权QEM代价 | 平面区域优先简化 |
| 4.4 折叠验证 | 拓扑/区域约束检查 | 无非法折叠 |

**验证方法**:
- 99%简化率时建筑轮廓仍清晰
- Hausdorff距离优于标准QEM
- 边缘和角点保持到最后

---

### Phase 5: 纹理与输出 [预估工作量: 中等]

**目标**: 纹理重映射和LOD生成

| 任务 | 详细内容 | 验收标准 |
|-----|---------|---------|
| 5.1 UV保持 | 简化时保持UV | UV坐标有效 |
| 5.2 纹理重映射 | Patch分解、图集打包 | 纹理正确显示 |
| 5.3 LOD生成 | 多级别输出 | 支持LOD0-LOD4 |
| 5.4 批处理 | 多模型并行处理 | 支持文件夹处理 |

---

### Phase 6: 优化与测试 [预估工作量: 中等]

**目标**: 性能优化和完整测试

| 任务 | 详细内容 | 验收标准 |
|-----|---------|---------|
| 6.1 性能优化 | 空间索引、并行计算 | 大模型处理时间可接受 |
| 6.2 单元测试 | 各模块测试覆盖 | 覆盖率 > 80% |
| 6.3 集成测试 | 端到端测试 | 各种模型均可处理 |
| 6.4 对比测试 | vs MeshLab QEM | 指标优于基准 |

---

## 五、质量指标

### 5.1 功能指标

| 指标 | 目标值 |
|-----|-------|
| 支持简化率 | 10% - 99.9% |
| 支持面数 | 10M+ 三角面 |
| 输入格式 | OBJ, PLY, OFF |
| 输出格式 | OBJ, PLY |

### 5.2 性能指标

| 指标 | 目标值 |
|-----|-------|
| 1M面模型简化到1%用时 | < 30秒 |
| 内存占用 | < 模型大小 × 5 |
| 并行效率 | 4核 > 3倍加速 |

### 5.3 质量指标

| 指标 | 目标值 |
|-----|-------|
| Hausdorff距离 | 优于标准QEM 50%+ |
| 视觉保真度 | 99%简化率仍可辨识 |
| 拓扑正确性 | 无非流形、自交 |

---

## 六、风险与应对

| 风险 | 可能性 | 影响 | 应对策略 |
|-----|-------|-----|---------|
| 区域检测不准确 | 中 | 高 | 增加参数调节、多次迭代 |
| 大模型内存溢出 | 中 | 高 | 实现out-of-core处理 |
| 纹理重映射失真 | 中 | 中 | 保守策略、手动修正接口 |
| 算法效率低 | 低 | 中 | 空间索引、GPU加速 |

---

## 七、命令行接口设计

```bash
# 基本用法
sps_simplify input.obj -o output.obj -r 0.1

# 完整参数
sps_simplify input.obj \
    --output output.obj \
    --ratio 0.1 \                    # 简化率 (保留10%)
    --target-faces 1000 \            # 或指定目标面数
    --filter-iterations 5 \          # 滤波迭代次数
    --angle-threshold 25 \           # 区域生长角度阈值
    --preserve-texture \             # 保持纹理
    --generate-lod 4 \               # 生成4级LOD
    --output-format obj \            # 输出格式
    --verbose                        # 详细输出

# 批处理
sps_simplify --batch input_dir/ --output-dir output_dir/ -r 0.1

# 生成评估报告
sps_simplify input.obj -o output.obj -r 0.1 --evaluate --report metrics.json
```

---

## 八、下一步行动

1. **立即开始**: 创建项目目录结构，配置CMake
2. **第一个里程碑**: 完成基础框架，能读取和写出OBJ文件
3. **验证点**: 实现标准QEM，与MeshLab对比确认正确性
4. **持续记录**: 在 `docs/pdca/` 中记录开发过程

是否需要我现在开始创建项目的初始代码框架？
