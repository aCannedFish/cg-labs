#include <algorithm>
#include <cassert>
#include <limits>
#include "BVH.hpp"

// 取包围盒中心在指定轴上的分量（非 const 的 operator[] 只有声明，不能用）
static double centroidAxis(const Vector3f& centroid, int axis)
{
    if (axis == 0)
        return centroid.x;
    if (axis == 1)
        return centroid.y;
    return centroid.z;
}

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else if (splitMethod == SplitMethod::SAH) {
        // SAH：在三个轴向上分别尝试所有划分位置，
        // 用左右子树包围盒面积乘以其中图元数量估计遍历开销，取开销最小的划分。
        const int n = static_cast<int>(objects.size());
        int bestAxis = 0;
        int bestSplit = n / 2;
        double bestCost = std::numeric_limits<double>::infinity();

        for (int axis = 0; axis < 3; ++axis) {
            std::sort(objects.begin(), objects.end(), [axis](Object* a, Object* b) {
                return centroidAxis(a->getBounds().Centroid(), axis) <
                       centroidAxis(b->getBounds().Centroid(), axis);
            });

            // 前缀包围盒，leftBounds[i] 覆盖前 i+1 个图元
            std::vector<Bounds3> leftBounds(n);
            Bounds3 running;
            for (int i = 0; i < n; ++i) {
                running = Union(running, objects[i]->getBounds());
                leftBounds[i] = running;
            }

            // 后缀包围盒，rightBounds[i] 覆盖从 i 到末尾的图元
            std::vector<Bounds3> rightBounds(n);
            running = Bounds3();
            for (int i = n - 1; i >= 0; --i) {
                running = Union(running, objects[i]->getBounds());
                rightBounds[i] = running;
            }

            for (int split = 1; split < n; ++split) {
                double cost = leftBounds[split - 1].SurfaceArea() * split +
                              rightBounds[split].SurfaceArea() * (n - split);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = split;
                }
            }
        }

        // 按取得最小开销的轴重新排序，再在 bestSplit 处切分
        std::sort(objects.begin(), objects.end(), [bestAxis](Object* a, Object* b) {
            return centroidAxis(a->getBounds().Centroid(), bestAxis) <
                   centroidAxis(b->getBounds().Centroid(), bestAxis);
        });

        auto beginning = objects.begin();
        auto middling = objects.begin() + bestSplit;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    Intersection inter;

    // 先做包围盒求交测试，不相交的子树直接剪枝
    // dirIsNeg 的约定是 int(x > 0)，即方向分量为正时为 1
    std::array<int, 3> dirIsNeg = {ray.direction.x > 0, ray.direction.y > 0,
                                   ray.direction.z > 0};
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return inter;
    }

    // 叶子节点：与其中唯一的物体求交
    if (node->left == nullptr && node->right == nullptr)
    {
        if (node->object != nullptr)
        {
            inter = node->object->getIntersection(ray);
        }
        return inter;
    }

    // 内部节点：递归查询两个子树，取距离最近的交点
    Intersection leftInter = getIntersection(node->left, ray);
    Intersection rightInter = getIntersection(node->right, ray);
    return leftInter.distance < rightInter.distance ? leftInter : rightInter;
}