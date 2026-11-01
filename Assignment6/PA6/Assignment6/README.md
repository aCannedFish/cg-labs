# 作业6：BVH 加速结构

## 实现内容

- `Renderer::Render()`：按像素生成主光线并调用 `Scene::castRay`。
- `Triangle::getIntersection()`：用 Möller–Trumbore 计算交点，填充 `Intersection` 的坐标、法线、距离和材质。
- `Bounds3::IntersectP()`：在三个轴向上求射线进入和离开包围盒的 t 值，取区间交集判断是否相交。
- `BVHAccel::getIntersection()`：先做包围盒求交剪枝，叶子节点与其中物体求交，内部节点递归查询两个子树并返回距离最近的交点。
- 加分项：SAH 划分。

## SAH 实现

构建每个节点时，对 x、y、z 三个轴分别按图元包围盒中心排序，然后从左到右累加前缀包围盒、从右到左累加后缀包围盒。对每个划分位置 i，用

```
cost = area(left) * i + area(right) * (n - i)
```

估计左右子树的遍历代价，取三个轴中代价最小的划分。图元数量为 1 时成为叶子；由于构建参数 `maxPrimsInNode` 为 1，每个叶子只包含一个图元。场景 BVH 和 `MeshTriangle` 内部的 BVH 都使用 SAH 构建。

## 时间对比

测试场景为 bunny（约 2000 个三角形），分辨率 1280×960，未开启编译器优化：

| 构建方式 | BVH 构建 | 总耗时（含构建） |
| --- | --- | --- |
| NAIVE（按最长轴中位数划分） | 小于 1 秒 | 约 5.9 秒 |
| SAH | 约 1 秒 | 约 5.5 秒 |

SAH 需要额外排序和面积计算，单帧渲染时构建开销分摊不掉；它减少的是每条光线的节点访问次数，图元和场景规模更大时收益才明显。两种构建方式得到的图像逐像素一致。

## 运行

```bash
mkdir build && cd build && cmake .. && make
./RayTracing
```

渲染结果写入当前目录的 `binary.ppm`。
