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
    Intersection p_inter = intersect(ray);
    if(!p_inter.happened) return Vector3f();
    if(p_inter.m->hasEmission()) return p_inter.m->getEmission();

    Vector3f L_dir;
    Vector3f L_indir;
    float pdf_light = 1.0f; Intersection x_inter;
    sampleLight(x_inter,pdf_light);
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f N = p_inter.normal.normalized();
    Vector3f NN = x_inter.normal.normalized();
    Vector3f ws = (x - p).normalized();
    float ws_distance = (x - p).norm();
    Vector3f emit = x_inter.emit;

    Ray p2x(p,ws);
    Intersection p2x_inter = intersect(p2x);

    if(p2x_inter.distance - ws_distance > -0.0001)
    {
        L_dir = emit * p_inter.m->eval(ray.direction,p2x.direction,N) * dotProduct(p2x.direction,N) * dotProduct(-p2x.direction,NN) / std::pow(ws_distance,2) / pdf_light;
    }

    if(get_random_float() > RussianRoulette)
    {
        return L_dir;
    }
    
    Vector3f wi = p_inter.m->sample(ray.direction,N).normalized();
    Ray p2wi(p_inter.coords,wi);
    Intersection p2wi_inter;
    p2wi_inter = intersect(p2wi);

    if(p2wi_inter.happened && (!p2wi_inter.m->hasEmission()))
    {
        L_indir = castRay(p2wi,depth+1) * p_inter.m->eval(ray.direction,p2wi.direction,N) * dotProduct(p2wi.direction,N) / p_inter.m->pdf(ray.direction,p2wi.direction,N) / RussianRoulette;
    }

    return L_dir + L_indir;
}