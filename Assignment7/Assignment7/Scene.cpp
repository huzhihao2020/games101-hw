//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
  printf(" - Generating BVH...\n\n");
  this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
  return this->bvh->Intersect(ray);
}

// 在场景中所有光源中，以表面积为权重，随机挑选一个光源进行采样
// 取一个随机数p
// 按面积随机挑一个出射方向，计算交点的dir、coords、emission
void Scene::sampleLight(Intersection &pos, float &pdf) const {
  float emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
    }
  }
  float p = get_random_float() * emit_area_sum;
  emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
      if (p <= emit_area_sum) {
        objects[k]->Sample(pos, pdf);
        break;
      }
    }
  }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) {
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
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  auto L_indir = Vector3f(0);
  auto L_dir = Vector3f(0);

  // ray 与场景 bvh 的交点 inter
  Intersection inter = intersect(ray);

  if (!inter.happened)
    return Vector3f(0);

  // ray打中光源，直接渲染光源
  if (inter.m->hasEmission()) {
    return inter.m->getEmission();
  }

  auto p = inter.coords;
  auto N = normalize(inter.normal);
  auto wo = -ray.direction;

  Intersection inter_on_light;
  float pdf_light = 0.0f; // 1/A

  // 1. 对光源采样计算直接光照
  sampleLight(inter_on_light, pdf_light);
  auto x = inter_on_light.coords;
  auto ws = x - p;
  auto distance_2 = dotProduct(ws, ws);
  ws = normalize(ws);
  auto NN = normalize(inter_on_light.normal);

  auto fr = inter.m->eval(ws, wo, N);
  auto cos_theta = dotProduct(ws, N);
  auto cos_theta1 = dotProduct(-ws, NN);

  auto inter_p_light = intersect(Ray(p, ws));
  if (inter_p_light.distance - sqrt(distance_2) > -5 * EPSILON) {
    // 交点与光源中间没有遮挡
    L_dir = inter_on_light.emit * fr * cos_theta * cos_theta1 / distance_2 /
            pdf_light;
  }

  // 2. 半球面均匀采样计算间接光照
  if (get_random_float() > RussianRoulette) {
    return L_dir;
  }

  auto wi = inter.m->sample(wo, N); // 采用某种分布采样，返回一个方向
  L_indir = castRay(Ray(p, wi), depth + 1) * inter.m->eval(wi, wo, N) *
            dotProduct(wi, N) / inter.m->pdf(wi, wo, N) / RussianRoulette;

  return L_dir + L_indir;
}