//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, MICROFACET };

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    // GGX 法线分布函数：roughness 越小，高光越集中
    inline float DistributionGGX(const Vector3f &N, const Vector3f &H, float roughness) const
    {
        float a = std::max(roughness * roughness, 1e-4f);
        float a2 = a * a;
        float NdotH = std::max(0.0f, dotProduct(N, H));
        float denom = NdotH * NdotH * (a2 - 1.0f) + 1.0f;
        return a2 / (M_PI * denom * denom);
    }

    // Schlick-GGX 几何项的单方向形式
    inline float GeometrySchlickGGX(float NdotV, float roughness) const
    {
        float r = roughness + 1.0f;
        float k = (r * r) / 8.0f;
        return NdotV / (NdotV * (1.0f - k) + k);
    }

    // Smith 几何项：同时考虑入射和出射方向的自遮挡
    inline float GeometrySmith(const Vector3f &N, const Vector3f &wo, const Vector3f &wi, float roughness) const
    {
        float NdotO = std::max(0.0f, dotProduct(N, wo));
        float NdotI = std::max(0.0f, dotProduct(N, wi));
        return GeometrySchlickGGX(NdotO, roughness) * GeometrySchlickGGX(NdotI, roughness);
    }

    // Schlick 近似的菲涅尔项，F0 为垂直入射时的反射率
    inline Vector3f FresnelSchlick(float cosTheta, const Vector3f &F0) const
    {
        float f = std::pow(1.0f - cosTheta, 5.0f);
        return F0 + (Vector3f(1.0f) - F0) * f;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    float roughness;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
    roughness = 0.2f;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {
            // 按 GGX 分布采样半程向量，再关于半程向量反射入射方向
            float a = std::max(roughness * roughness, 1e-4f);
            float a2 = a * a;
            for (int attempt = 0; attempt < 8; ++attempt)
            {
                float x_1 = get_random_float(), x_2 = get_random_float();
                float phi = 2.0f * M_PI * x_1;
                float cosTheta = std::sqrt((1.0f - x_2) / (1.0f + (a2 - 1.0f) * x_2));
                float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
                Vector3f localH(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
                Vector3f H = normalize(toWorld(localH, N));
                Vector3f out = normalize(2.0f * dotProduct(wi, H) * H - wi);
                // 采样结果落到表面以下时重新采样
                if (dotProduct(out, N) > 0.0f)
                {
                    return out;
                }
            }
            return N;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MICROFACET:
        {
            // GGX 采样半程向量的概率密度，转换到出射方向
            Vector3f H = normalize(wi + wo);
            float NdotH = std::max(0.0f, dotProduct(N, H));
            float OdotH = std::max(0.0f, dotProduct(wo, H));
            if (OdotH < EPSILON)
            {
                return 0.0f;
            }
            float D = DistributionGGX(N, H, roughness);
            return D * NdotH / (4.0f * OdotH);
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            // Cook-Torrance BRDF：D * G * F / (4 * (N·wo) * (N·wi))
            float NdotO = dotProduct(N, wo);
            float NdotI = dotProduct(N, wi);
            if (NdotO <= 0.0f || NdotI <= 0.0f)
            {
                return Vector3f(0.0f);
            }

            Vector3f H = normalize(wi + wo);
            float OdotH = std::max(0.0f, dotProduct(wo, H));

            float D = DistributionGGX(N, H, roughness);
            float G = GeometrySmith(N, wo, wi, roughness);
            Vector3f F = FresnelSchlick(OdotH, Ks);

            float denom = 4.0f * NdotO * NdotI;
            if (denom < EPSILON)
            {
                return Vector3f(0.0f);
            }
            return F * (D * G / denom);
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
