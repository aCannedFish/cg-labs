# 作业7：路径追踪

## 实现内容

`Scene::castRay()` 实现了路径追踪：

- 光线没有击中物体时返回黑色；击中光源时直接返回自发光，使光源在画面中可见。
- 直接光照用 `sampleLight` 按面积采样光源上的点 x，先判断 p 到 x 之间是否被遮挡，再按 BRDF、两个余弦项和距离平方计算贡献。
- 间接光照以 `RussianRoulette` 的概率继续追踪，按材质采样出射方向，递归累加后用 pdf 做重要性采样校正。
- `Triangle::getIntersection`、`Bounds3::IntersectP`、`BVH::getIntersection` 从作业6迁移并适配新框架。

## 加分项

### 多线程

- `Renderer::Render` 按行把画面分配给 `std::thread::hardware_concurrency()` 个线程，每个线程只写自己负责的行，帧缓冲元素互不重叠。
- `get_random_float` 改为 `thread_local` 的 `mt19937`，避免多线程共享同一随机数状态；同时去掉了原来每次调用都构造 `random_device` 的开销。
- 进度条更新用互斥锁保护，繁忙时按固定行数跳过一次刷新。

### Microfacet

- 材质使用 Cook-Torrance BRDF：GGX 法线分布 D、Smith 几何项 G、Schlick 菲涅尔项 F，F0 取自材质的 `Ks`，`roughness` 控制高光宽度。
- `sample` 按 GGX 分布采样半程向量再反射入射方向，`pdf` 返回对应的重要性采样概率密度，`eval` 返回 BRDF 值，三者保持一致。
- 默认场景保持标准 Cornell Box；运行时加 `--microfacet` 参数，会把前面的盒子换成一个金属球（roughness 0.2），用于展示微表面反射。

## 渲染参数与耗时

- 分辨率 784×784，SPP 16，8 线程。
- 本机默认构建（未开启编译器优化）墙钟约 78 秒，8 线程累计 CPU 时间约 7 分钟。
- 未做多线程时同一场景需要半小时以上，多线程和随机数生成器的修复带来了明显加速。

## 运行

```bash
mkdir build && cd build && cmake .. && make
./RayTracing                # 标准 Cornell Box
./RayTracing --microfacet   # 金属球微表面演示
```

渲染结果写入当前目录的 `binary.ppm`。
