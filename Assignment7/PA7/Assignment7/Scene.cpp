//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // 光线没有击中任何物体，返回黑色
    Intersection inter = intersect(ray);
    if (!inter.happened)
    {
        return Vector3f(0.0f);
    }

    // 击中的是光源，直接返回其自发光，这样光源在画面中可见
    if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }

    Vector3f L_dir(0.0f), L_indir(0.0f);
    Vector3f p = inter.coords;
    Vector3f N = normalize(inter.normal);
    Vector3f wo = normalize(-ray.direction);

    // 直接光照：在光源上按面积均匀采样一个点 x
    Intersection lightInter;
    float pdf_light = 0.0f;
    sampleLight(lightInter, pdf_light);
    if (pdf_light > 0.0f)
    {
        Vector3f x = lightInter.coords;
        Vector3f ws = normalize(x - p);
        Vector3f NN = normalize(lightInter.normal);
        Vector3f emit = lightInter.emit;
        float distance2 = dotProduct(x - p, x - p);

        // 从 p 向 x 发射阴影光线，检查中间是否有遮挡
        Ray shadowRay(p + N * EPSILON, ws);
        Intersection block = intersect(shadowRay);
        bool visible = !block.happened ||
                       (block.coords - p).norm() >= (x - p).norm() - EPSILON;
        if (visible)
        {
            L_dir = emit * inter.m->eval(wo, ws, N) * dotProduct(ws, N) *
                    dotProduct(-ws, NN) / distance2 / pdf_light;
        }
    }

    // 间接光照：俄罗斯轮盘赌决定是否继续追踪
    if (get_random_float() < RussianRoulette)
    {
        Vector3f wi = normalize(inter.m->sample(wo, N));
        Ray r(p + N * EPSILON, wi);
        Intersection q = intersect(r);
        if (q.happened && !q.m->hasEmission())
        {
            float pdf = inter.m->pdf(wo, wi, N);
            if (pdf > 0.0f)
            {
                L_indir = castRay(r, depth + 1) * inter.m->eval(wo, wi, N) *
                          dotProduct(wi, N) / pdf / RussianRoulette;
            }
        }
    }

    return L_dir + L_indir;
}