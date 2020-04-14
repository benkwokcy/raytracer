#pragma once

#include <atlas/math/Math.hpp>
#include <atlas/math/Ray.hpp>
#include <atlas/math/Solvers.hpp>
#include <atlas/core/Timer.hpp>

#include <stb_image.h>
#include <stb_image_write.h>

#include <vector>
#include <iostream>
#include <algorithm> 
#include <random> 
#include <math.h>
#include <thread>
#include <mutex>
#include <memory>
#include <queue>
#include <chrono>

using namespace atlas;
using Colour = math::Vector;
using Ray = math::Ray<math::Vector>;

Colour BLACK { 0,0,0 };
Colour WHITE { 1,1,1 };

// class forward declarations
struct World;
class Object;
class Material;
class Light;
class AmbientOccluder; 
class PinholeCamera;
class Sampler;
class Tracer;

// constants
const float kEpsilon = 0.0001 * 150; // tiny value used to avoid floating point imprecision 
const float kHugeValue = 1.0E10; 
const float piInverse { 0.31830988618 };
const float pi { 3.14159265359 };

// function declarations
void saveToBMP(std::string const& filename, std::size_t width, std::size_t height, std::vector<Colour> const& image);
void buildScene(World& w);
void render(int width, int height, std::vector<Colour>* image);
void renderParallel(int width, int height, std::vector<Colour>* image);

/********************
 *      UTILITY     *
 ********************/
float rand_float(void) {
    return (float) rand() / (float) RAND_MAX;
}

float rand_float_multi(int l, float h) {
    return (rand_float() * (h - l) + l);
}

int rand_int_multi(int l, int h) {
    return ((int) (rand_float_multi(0, h - l + 1) + l));
}

/********************
 *      SLAB        *
 ********************/
// represents a subsection of the image grid to be rendered by a thread worker
struct Slab {
    int startY;
    int startX;
    int endY;
    int endX;
};

/********************
 *      WORLD       *
 ********************/

struct World {
    int width;
    int height;
    Colour background { 0,0,0 };
    std::vector<Colour>* image;
    std::vector<std::shared_ptr<Object>> objects;
    std::shared_ptr<AmbientOccluder> ambient;
    std::vector<std::shared_ptr<Light>> lights;
    int maxDepth { 1 };
    std::shared_ptr<Tracer> tracer;
    int numSamples { 1 };
};

/********************
 *        HIT       *
 ********************/

// Used to return information about a ray-object intersection
struct Hit {
    float root; // the "t" in r(t) = o + dt at the point of intersection
    math::Vector normal; // normal to the surface at point of intersection
    std::shared_ptr<Material> material;
    math::Vector hitpoint;
    math::Ray<math::Vector>* ray;
    int depth { 0 };
};

/********************
 *      SAMPLER     *
 ********************/

class Sampler {
public:
    std::vector<math::Point2> samples;
    std::vector<math::Vector> hemisphereSamples;
    int numSamples { 0 };
    int numSets { 83 };
    int index { 0 };
    std::vector<int> shuffledIndices;		
    int jump { 0 };

    Sampler(int numSamples, int exp) {
        if (numSamples == 1) {
            generateRegularSamples(numSamples);
        } else {
            generateMultiJitterSamples(numSamples);
        }
        generateHemisphereSamples(exp);
        setupShuffledIndices();
    }

    // Fetches the next sample
    math::Point2 getSample() {
        if (index % numSamples == 0) {
            jump = (rand() % numSets) * numSamples;				
        }

        return (samples[jump + shuffledIndices[jump + index++ % numSamples]]);  
    }

    math::Vector getHemisphereSample() {
        if (index % numSamples == 0) {
            jump = (rand() % numSets) * numSamples;				
        }

        return (hemisphereSamples[jump + shuffledIndices[jump + index++ % numSamples]]);  
    }

    // e: cosine density distribution, ex. e=1
    void generateHemisphereSamples(double e) {
        hemisphereSamples.clear();
        int size { (int) samples.size() };
        
        for (int i = 0; i < size; i++) {
            double cos_phi { cos(2.0 * pi * samples[i].x) };
            double sin_phi = sin(2.0 * pi * samples[i].x);	
            double cos_theta { pow((1.0 - samples[i].y), 1.0 / (e + 1.0)) };
            double sin_theta { sqrt (1.0 - cos_theta * cos_theta) };
            double pu { sin_theta * cos_phi };
            double pv { sin_theta * sin_phi };
            double pw { cos_theta };
            hemisphereSamples.push_back(math::Vector(pu, pv, pw)); 
        }
    }

    void generateRegularSamples(int numSamp) {
        samples.clear();
        numSamples = numSamp;
        int n { (int) sqrt((float)numSamples) };

        for (int j = 0; j < numSets; j++)
            for (int p = 0; p < n; p++)		
                for (int q = 0; q < n; q++)
                    samples.push_back(math::Point2((q + 0.5) / n, (p + 0.5) / n));
    }

    void generateMultiJitterSamples(int numSamp) {
        samples.clear();
        numSamples = numSamp;
        int dim { (int) sqrt((float)numSamples) };

        float subcell_width { 1.0f / ((float) numSamples) };

        math::Point2 fillPoint;
        for (int j = 0; j < numSamples * numSets; j++) {
            samples.push_back(fillPoint);
        }

        // distribute points in the initial patterns
        for (int p = 0; p < numSets; p++)
            for (int i = 0; i < dim; i++)		
                for (int j = 0; j < dim; j++) {
                    samples[i * dim + j + numSamples * p].x = (i * dim + j) * subcell_width + rand_float_multi(0, subcell_width);
                    samples[i * dim + j + numSamples * p].y = (j * dim + i) * subcell_width + rand_float_multi(0, subcell_width);
                }

        // shuffle x coordinates
        for (int p = 0; p < numSets; p++)
            for (int i = 0; i < dim; i++)		
                for (int j = 0; j < dim; j++) {
                    int k { rand_int_multi(j, dim - 1) };
                    float t { samples[i * dim + j + numSamples * p].x };
                    samples[i * dim + j + numSamples * p].x = samples[i * dim + k + numSamples * p].x;
                    samples[i * dim + k + numSamples * p].x = t;
                }

        // shuffle y coordinates
        for (int p = 0; p < numSets; p++)
            for (int i = 0; i < dim; i++)		
                for (int j = 0; j < dim; j++) {
                    int k { rand_int_multi(j, dim - 1) };
                    float t { samples[j * dim + i + numSamples * p].y };
                    samples[j * dim + i + numSamples * p].y = samples[k * dim + i + numSamples * p].y;
                    samples[k * dim + i + numSamples * p].y = t;
                }
    }

    void setupShuffledIndices(void) {
        shuffledIndices.reserve(numSamples * numSets);
        std::vector<int> indices;
        
        for (int j = 0; j < numSamples; j++)
            indices.push_back(j);
        
        for (int p = 0; p < numSets; p++) { 
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(indices.begin(), indices.end(), g);	
            
            for (int j = 0; j < numSamples; j++) {
                shuffledIndices.push_back(indices[j]);
            }
        }	
    }
};

/********************
 *      OBJECTS      *
 ********************/

// parent class for all objects to allow looping over multiple different objects in a vector
class Object {
public:
    Object(std::shared_ptr<Material> m):
        material {m}
    {}

    // calculates ray-object intersection and saves information about the intersection in the hit struct
    virtual bool intersect(const math::Ray<math::Vector>& ray, Hit& hit) = 0;

    // reverse normal if it's facing the wrong direction
    math::Vector getNormal(const math::Ray<math::Vector>& ray, math::Vector normal) {
        if (glm::dot(-ray.d, normal) < 0) {
            return -normal;
        }
        
        return normal;
    }
protected:
    std::shared_ptr<Material> material;
};

class Sphere : public Object {
public:
    Sphere(math::Point cn, float r, std::shared_ptr<Material> m) :
        Object(m),
        center{ cn },
        radius{ r },
        radius_squared{ r * r }
    {}

    bool intersect(const math::Ray<math::Vector>& ray, Hit& hit) {
        float a { glm::dot(ray.d, ray.d) };
        float b { glm::dot(ray.d, ray.o - center) * 2 };
        float c { glm::dot(ray.o - center, ray.o - center) - radius_squared };
        float discriminant { b * b - (4.0f * a * c) };

        if (discriminant < 0.0f) {
            return false;
        }

        float e = std::sqrt(discriminant); // save this result because computation is expensive

        // I duplicate some code below to return early for better performance
        float smallerRoot { (-b - e) / (2.0f * a) };
        if (smallerRoot > kEpsilon) {
            hit.root = smallerRoot;
            hit.material = material;
            hit.normal = glm::normalize(ray.o + (ray.d * hit.root) - center); // TODO - assumes camera is outside the sphere?
            hit.hitpoint = ray.o + (ray.d * smallerRoot);
            return true;
        }
        
        float largerRoot = (-b + e) / (2.0f * a);	
        if (largerRoot > kEpsilon) {
            hit.root = largerRoot;
            hit.material = material;
            hit.normal = glm::normalize(ray.o + (ray.d * hit.root) - center); // TODO - assumes camera is outside the sphere?
            hit.hitpoint = ray.o + (ray.d * largerRoot);
            return true;
        }

        return false;
    }

private:
    math::Point center;
    float radius;
    float radius_squared;
};

class Plane : public Object {
public:
    Plane(math::Vector n, math::Point p, std::shared_ptr<Material> m) :
        Object(m),
        normal{ n },
        point{ p }
    {}

    bool intersect(const math::Ray<math::Vector>& ray, Hit& hit) {
        auto root{ -glm::dot(ray.o - point, normal) / glm::dot(ray.d, normal) };

        if (root >= 0.0f) {
            hit.root = root;
            hit.material = material;
            hit.normal = getNormal(ray, normal);
            hit.hitpoint = ray.o + (ray.d * root);
            return true;
        }

        return false;
    }

private:
    math::Vector normal;
    math::Point point;
};

class Triangle : public Object {
public:
    Triangle(math::Point aa, math::Point bb, math::Point cc, std::shared_ptr<Material> m) :
        Object(m),
        v0 {aa},
        v1 {bb},
        v2 {cc},
        normal{ glm::normalize(glm::cross(bb - aa,cc - aa)) }
    {}

    // credit to textbook for intersection calculation
    bool intersect(const math::Ray<math::Vector>& ray, Hit& hit) {
        float a = v0.x - v1.x, b = v0.x - v2.x, c = ray.d.x, d = v0.x - ray.o.x;
        float e = v0.y - v1.y, f = v0.y - v2.y, g = ray.d.y, h = v0.y - ray.o.y;
        float i = v0.z - v1.z, j = v0.z - v2.z, k = ray.d.z, l = v0.z - ray.o.z;

        float m = f * k - g * j, n = h * k - g * l, p = f * l - h * j;
        float q = g * i - e * k, s = e * j - f * i;

        float inv_denom = 1.0 / (a * m + b * q + c * s);

        float e1 = d * m - b * n - c * p;
        float beta = e1 * inv_denom;

        if (beta < 0.0) {
            return false;
        }

        float r = e * l - h * i;
        float e2 = a * n + d * q + c * r;
        float gamma = e2 * inv_denom;

        if (gamma < 0.0) {
            return false;
        }

        if (beta + gamma > 1.0) {
            return false;
        }

        float e3 = a * p - b * r + d * s;
        float t = e3 * inv_denom;

        if (t < kEpsilon) {
            return false;
        }

        hit.root = t;
        hit.material = material;
        hit.normal = getNormal(ray, normal);
        hit.hitpoint = ray.o + (ray.d * t);

        return true;
    }

private:
    math::Point v0;
    math::Point v1;
    math::Point v2;
    math::Vector normal;
};

// TODO - Implement torus normal
class Torus : public Object {
public:
    Torus(float s, float t, std::shared_ptr<Material> m) :
        Object(m),
        sweptRadius{ s },
        tubeRadius{ t }
    {}

    // credit to textbook for intersection calculation
    bool intersect(const math::Ray<math::Vector>& ray, Hit& hit) {
        float x1 = ray.o.x; float y1 = ray.o.y; float z1 { ray.o.z };
        float d1 = ray.d.x; float d2 = ray.d.y; float d3 { ray.d.z };

        std::vector<float> coeffs;
        std::vector<float> roots;

        float sum_d_sqrd { d1 * d1 + d2 * d2 + d3 * d3 };
        float e { x1 * x1 + y1 * y1 + z1 * z1 - sweptRadius * sweptRadius - tubeRadius * tubeRadius };
        float f { x1 * d1 + y1 * d2 + z1 * d3 };
        float four_a_sqrd { 4.0f * sweptRadius * sweptRadius };

        // reverse co-efficients because Mauricio's function expects that
        coeffs.push_back(sum_d_sqrd * sum_d_sqrd);
        coeffs.push_back(4.0 * sum_d_sqrd * f);
        coeffs.push_back(2.0 * sum_d_sqrd * e + 4.0 * f * f + four_a_sqrd * d2 * d2);
        coeffs.push_back(4.0 * f * e + 2.0 * four_a_sqrd * y1 * d2);
        coeffs.push_back(e * e - four_a_sqrd * (tubeRadius * tubeRadius - y1 * y1));

        int num_real_roots { (int) math::solveQuartic(coeffs, roots) };

        bool intersected { false };
        float t { kHugeValue };

        if (num_real_roots == 0) {
            return false;
        }

        for (int j = 0; j < num_real_roots; j++) {
            if (roots[j] > kEpsilon) {
                intersected = true;
                if (roots[j] < t) {
                    t = roots[j];
                }
            }
        }

        if (!intersected) {
            return false;
        }

        hit.root = t;
        hit.material = material;
        // TODO - hit.normal?
        hit.hitpoint = ray.o + (ray.d * t);

        return true;
    }

private:
    float sweptRadius; // swept radius, aka the size of the donut hole
    float tubeRadius; // tube radius, aka the thickness of the donut
};

// TODO - implement box normals
class Box : public Object {
public:
    Box(math::Point a, math::Point b, std::shared_ptr<Material> m) :
        Object(m),
        A { a },
        B { b }
    {}

    bool intersect(const math::Ray<math::Vector>& ray, Hit& hit) {
        // giving these a short variable name for readability
        math::Vector O { ray.o };
        math::Vector D { ray.d };

        // int faceIn;
        // int faceOut;
    
        // intersect with 6 planes, each corresponding to a side of the box
        float t_ax { (A.x - O.x) / D.x };
        float t_ay { (A.y - O.y) / D.y };
        float t_az { (A.z - O.z) / D.z };
        float t_bx { (B.x - O.x) / D.x };
        float t_by { (B.y - O.y) / D.y };
        float t_bz { (B.z - O.z) / D.z };

        // find the biggest ta
        float t_a_max { t_ax };
        if (t_ay > t_a_max) {
            t_a_max = t_ay;
        }
        if (t_az > t_a_max) {
            t_a_max = t_az;
        }

        // find the smallest tb
        float t_b_min { t_bx };
        if (t_by < t_b_min) {
            t_b_min = t_by;
        }
        if (t_bz < t_b_min) {
            t_b_min = t_bz;
        }

        // check for intersection
        if (t_a_max < t_b_min && t_b_min > kEpsilon) {
            if (t_a_max > kEpsilon) {
                hit.root = t_a_max;
            } else {
                hit.root = t_b_min;
            }
            hit.material = material;
            // TODO - hit.normal?
            hit.hitpoint = ray.o + (ray.d * hit.root);

            return true;
        }

        return false;
    }

    // math::Vector getNormal(int face) {
    // 	switch (face) {
    // 		case 0:	return (math::Vector(-1, 0, 0));
    // 		case 1:	return (math::Vector(0, -1, 0));
    // 		case 2:	return (math::Vector(0, 0, -1));
    // 		case 3:	return (math::Vector(1, 0, 0));	
    // 		case 4:	return (math::Vector(0, 1, 0));	
    // 		case 5:	return (math::Vector(0, 0, 1));	
    // 	}
    // }


private:
    math::Point A; // minimum point of the box, where A.x < B.x, A.y < B.y, A.z < B.z
    math::Point B; // maximum point of the box
};


/********************
 *      LIGHTS      *
 ********************/

class Light {
public:
    Colour colour;
    float scale; // radiance, controls the brightness of the light

    Light(Colour c, float s):
        colour {c},
        scale {s}
    {}

    virtual math::Vector getDirection(math::Vector p) = 0;
    virtual float getDistance(math::Vector p) = 0;

    bool inShadow(const Hit& hit, World& w) {
        Hit tempHit {};
        math::Ray<math::Vector> ray { hit.hitpoint, getDirection(hit.hitpoint) };
        float hit_min { getDistance(hit.hitpoint) };
        for (auto o : w.objects) {
            if (o->intersect(ray, tempHit) && tempHit.root < hit_min && tempHit.root > kEpsilon) {
                return true;
            }
        }
        return false;
    }
};

class AmbientLight : public Light {
public:
    AmbientLight(Colour c, float s):
        Light(c, s)
    {}

    math::Vector getDirection(math::Vector p) { // unused function
        (void) p; // avoid unused parameter warning
        return math::Vector {0, 0, 0};
    }

    float getDistance(math::Vector p) {
        (void) p;
        return kEpsilon;
    }

    Colour getLight() {
        return scale * colour;
    }
};

class AmbientOccluder : public Light {
public:
    std::shared_ptr<Sampler> sampler;
    math::Vector u, v, w;
    float minAmount;

    AmbientOccluder(Colour c, float s, std::shared_ptr<Sampler> smp, float m):
        Light(c, s),
        sampler { smp },
        minAmount { m }
    {}

    math::Vector getDirection(math::Vector p) {
        (void) p;
        math::Vector v { sampler->getHemisphereSample() };
        return ((v.x * u) + (v.y * v) + (v.z * w));
    }

    float getDistance(math::Vector p) {
        (void) p;
        return kEpsilon;
    }

    bool inShadow(const Hit& hit, World& w) {
        Hit tempHit {};
        math::Ray<math::Vector> ray { hit.hitpoint, getDirection(hit.hitpoint) };
        for (auto o : w.objects) {
            if (o->intersect(ray, tempHit) && tempHit.root > kEpsilon) {
                return true;
            }
        }
        return false;
    }

    Colour getLight(const Hit& hit, World& world) {	
        w = hit.normal;	
        v = glm::normalize(glm::cross(w, math::Vector(0.0072, 1.0, 0.0034)));
        u = glm::cross(v,w);
        
        if (inShadow(hit, world)) {
            return minAmount * scale * colour;
        } else {
            return scale * colour;
        }
    }
};

class PointLight : public Light {
public:
    math::Vector point;

    PointLight(Colour c, float s, math::Vector p):
        Light(c, s),
        point {p}
    {}

    math::Vector getDirection(math::Vector p) {
        return glm::normalize(point - p);
    }

    float getDistance(math::Vector origin) {
        math::Vector d { getDirection(origin) };
        return (point - origin).x / d.x;
    }
};

class DirectionalLight : public Light {
public:
    math::Vector direction; // direction the light is point towards

    DirectionalLight(Colour c, float s, math::Vector d):
        Light(c, s),
        direction { glm::normalize(d) }
    {}

    math::Vector getDirection(math::Vector p) {
        (void) p; // avoid unused parameter warning

        // jitter the direction to create soft shadows
        math::Vector jitterDirection { -direction };
        jitterDirection.x += (rand_float() / 20.0);
        jitterDirection.y += (rand_float() / 20.0);
        jitterDirection.z += (rand_float() / 20.0);

        return jitterDirection;
    }

    float getDistance(math::Vector p) {
        (void) p;
        return std::numeric_limits<float>::infinity();
    }
};

/********************
 *       BRDF       *
 ********************/

class GlossySpecular {
    public:
        float specularScale;
        int phongExponent;
        Colour specularColour;
        std::shared_ptr<Sampler> sampler;

        GlossySpecular(float s, int p, std::shared_ptr<Sampler> sp, Colour c):
            specularScale {s},
            phongExponent {p},
            specularColour {c},
            sampler {sp}
        {}

        Colour f(const Hit& hit, const math::Vector& wo, const math::Vector& wi) const {
            Colour colour = BLACK;  
            float ndotwi { glm::dot(glm::normalize(hit.normal), wi) };
            math::Vector r { -wi + ((2.0f * ndotwi) * hit.normal) };
            float rdotwo { glm::dot(r, wo) };

            if (rdotwo > 0.0) {
                colour = specularScale * specularColour * (float) std::pow(rdotwo, phongExponent);
            }
            return colour;
        }

        Colour sample_f(const Hit& hit, const math::Vector& wo, math::Vector& wi, float& pdf) const {
            float ndotwo { glm::dot(hit.normal, wo) };
            math::Vector r = -wo + 2.0f * hit.normal * ndotwo;     // direction of mirror reflection

            math::Vector w = r;								
            math::Vector u = glm::normalize(glm::cross(math::Vector(0.00424, 1, 0.00764), w)); 
            math::Vector v { glm::cross(u, w) };

            math::Vector sp { sampler->getHemisphereSample() };
            wi = sp.x * u + sp.y * v + sp.z * w;			// reflected ray direction

            if (glm::dot(hit.normal, wi) < 0.0f) 						// reflected ray is below tangent plane
                wi = -sp.x * u - sp.y * v + sp.z * w;

            float phong_lobe { (float) pow(glm::dot(r, wi), phongExponent) };
            pdf = phong_lobe * glm::dot(hit.normal, wi);

            return (specularScale * specularColour * phong_lobe);
        }
};

class PerfectSpecular {
    public:
        float reflectionCoefficient;	// reflection coefficient
        Colour reflectionColour;	// the reflection colour

        PerfectSpecular(float k, Colour c):	
            reflectionCoefficient {k}, 
            reflectionColour {c}
        {}

        // reflective and transparent materials
        Colour sample_f(const Hit& hit, math::Vector& wo, math::Vector& wi) {
            float ndotwo { glm::dot(hit.normal, wo) };
            wi = -wo + (2.0f * hit.normal * ndotwo); 
            return ((reflectionCoefficient * reflectionColour) / glm::dot(hit.normal, wi));
        }

        // path tracing
        // Colour sample_f(const Hit& hit, math::Vector& wo, math::Vector& wi, float& pdf) {
        // 	float ndotwo { glm::dot(hit.normal, wo) };
        // 	wi = -wo + (2.0f * hit.normal * ndotwo); 
        // 	pdf = fabs(glm::dot(hit.normal, wi));
        // 	return (reflectionCoefficient * reflectionColour);
        // }
};

class PerfectTransmitter {
    public:
        float	kt;			// transmission coefficient
        float	indexOfRefraction;		// index of refraction

        PerfectTransmitter(float k, float i):
            kt {k},
            indexOfRefraction {i}
        {}

        bool tir(const Hit& hit) const {
            math::Vector wo {-hit.ray->d}; 
            float cos_thetai = glm::dot(hit.normal, wo);  
            float eta { indexOfRefraction };

            if (cos_thetai < 0.0) 
                eta = 1.0 / eta; 

            return (1.0 - (1.0 - cos_thetai * cos_thetai) / (eta * eta) < 0.0);
        }	

        Colour sample_f(const Hit& sr, const math::Vector& wo, math::Vector& wt) const {

            math::Vector n {sr.normal};
            float cos_thetai {glm::dot(n, wo) };
            float eta { indexOfRefraction };	

            if (cos_thetai < 0.0) {			// transmitted ray is outside     
                cos_thetai = -cos_thetai;
                n = -n;  					// reverse direction of normal
                eta = 1.0 / eta; 			// invert indexOfRefraction 
            }

            float temp { 1.0f - (1.0f - cos_thetai * cos_thetai) / (eta * eta) };
            float cos_theta2 { sqrt(temp) };
            wt = -wo / eta - (cos_theta2 - cos_thetai / eta) * n;   

            return (kt / (eta * eta) * WHITE / fabs(glm::dot(sr.normal, wt)));
        }

};
/********************
 *     Tracers      *
 ********************/

class Tracer {
    public:
        World* world;

        Tracer(World* w) { world = w; }
        ~Tracer(void) {}
    
        virtual Colour trace_ray(math::Ray<math::Vector> ray, int depth) const;
};

/********************
 *     MATERIALS    *
 ********************/

// ambient
class Material {
public:
    Colour colour;
    float ambientScale;

    Material(Colour c, float ka):
        colour {c},
        ambientScale {ka}
    {}

    virtual Colour shade(const Hit& hit, World& w) = 0;
};

// ambient + diffuse
class Matte : public Material {
public:
    float diffuseScale;

    Matte(Colour c, float ka, float d):
        Material(c, ka),
        diffuseScale {d}
    {}

    Colour shade(const Hit& hit, World& w) {
        Colour c { (ambientScale * colour) * (w.ambient->getLight(hit, w)) };
        for (auto l : w.lights) {
            float dotproduct { glm::dot(hit.normal, l->getDirection(hit.hitpoint)) };
            if (dotproduct > 0.0f && !l->inShadow(hit, w)) {
                c += (diffuseScale * colour * piInverse) * (l->scale * l->colour) * dotproduct;
            }
        }

        return c;
    }
};

// ambient + diffuse + specular
class Specular : public Material {
public:
    float diffuseScale;
    std::shared_ptr<GlossySpecular> glossySpecular;

    Specular(Colour c, float ka, float d, std::shared_ptr<GlossySpecular> gs):
        Material(c, ka),
        diffuseScale {d},
        glossySpecular {gs}
    {}
 
    Colour shade(const Hit& hit, World& w) {
        math::Vector outgoingLight { glm::normalize(-(hit.ray->d)) };
        Colour c { (ambientScale * colour) * (w.ambient->getLight(hit, w)) };
        for (auto l : w.lights) {
            math::Vector incomingLight { glm::normalize(l->getDirection(hit.hitpoint)) };
            float dotproduct { glm::dot(hit.normal, incomingLight) };
            if (dotproduct > 0.0f && !l->inShadow(hit, w)) {
                c += (diffuseScale * colour * piInverse);
                c += glossySpecular->f(hit, outgoingLight, incomingLight) * (l->scale * l->colour) * dotproduct;
            }
        }

        return c; 
    }
};

// glossy reflective
class GlossyReflective : public Specular {
public:
    std::shared_ptr<GlossySpecular> glossySpecularIndirect;

    GlossyReflective(Colour c, float ka, float d, std::shared_ptr<GlossySpecular> gs, std::shared_ptr<GlossySpecular> gs2):
        Specular(c, ka, d, gs ),
        glossySpecularIndirect {gs2}
    {}

    Colour shade(const Hit& hit, World& w) {
        Colour colour { Specular::shade(hit, w) };

        math::Vector wo { glm::normalize(-(hit.ray->d)) };
        math::Vector wi;	
        float pdf;
        Colour fr = glossySpecularIndirect->sample_f(hit, wo, wi, pdf); 
        Ray reflected_ray(hit.hitpoint, wi); 

        colour += fr * w.tracer->trace_ray(reflected_ray, hit.depth + 1) * glm::dot(hit.normal, wi) / pdf;

        return colour;
    }
};

// mirror reflective
class Reflective : public Specular {
    public:
        std::shared_ptr<PerfectSpecular> perfectSpecular;

        Reflective(Colour c, float ka, float d, std::shared_ptr<GlossySpecular> gs, std::shared_ptr<PerfectSpecular> ps):
            Specular(c, ka, d, gs ),
            perfectSpecular {ps}
        {}
 
        Colour shade(const Hit& hit, World& w) {
            Colour colour { Specular::shade(hit, w) };

            math::Vector wo { glm::normalize(-(hit.ray->d)) };
            math::Vector wi;	
            Colour fr = perfectSpecular->sample_f(hit, wo, wi); 
            Ray reflected_ray(hit.hitpoint, wi); 

            colour += fr * w.tracer->trace_ray(reflected_ray, hit.depth + 1) * glm::dot(hit.normal, wi);

            return colour;
        }
};

class Transparent : public Specular {
    public:
        std::shared_ptr<PerfectSpecular> 	perfectSpecular;
        std::shared_ptr<PerfectTransmitter> perfectTransmitter;	

    Transparent(Colour c, float ka, float d, std::shared_ptr<GlossySpecular> gs, std::shared_ptr<PerfectSpecular> ps, std::shared_ptr<PerfectTransmitter> pt):
        Specular(c, ka, d, gs ),
        perfectSpecular { ps },
        perfectTransmitter { pt }
    {}

    Colour shade(const Hit& hit, World& w) {
        Colour colour { Specular::shade(hit, w) };

        math::Vector wo { glm::normalize(-(hit.ray->d)) };
        math::Vector wi;
        Colour fr = perfectSpecular->sample_f(hit, wo, wi);
        Ray reflectedRay { hit.hitpoint, wi };

        if(perfectTransmitter->tir(hit)) {
            colour += w.tracer->trace_ray(reflectedRay, hit.depth + 1);
        } else {
            math::Vector wt;
            Colour ft = perfectTransmitter->sample_f(hit, wo, wt); // computes wt
            Ray transmittedRay(hit.hitpoint, wt);
            colour += fr * w.tracer->trace_ray(reflectedRay, hit.depth + 1) * fabs(glm::dot(hit.normal, wi));
            colour += ft * w.tracer->trace_ray(transmittedRay, hit.depth + 1) * fabs(glm::dot(hit.normal, wt));
        }
        return (colour);
    }
};

/**********************
 *      TRACERS 2     *
 **********************/

// need to declare this after materials because of circular dependencies
Colour Tracer::trace_ray(math::Ray<math::Vector> ray, int depth) const {
    if (depth > world->maxDepth)
        return BLACK;
    else {			
        // get colour of the closest intersection
        bool isHit { false };
        Hit tempHit{};
        tempHit.root = std::numeric_limits<float>::infinity();
        Hit hit_min{};
        hit_min.root = std::numeric_limits<float>::infinity();
        Colour sample_colour { 0,0,0 };
        for (auto s : world->objects) {
            if (s->intersect(ray, tempHit) && tempHit.root < hit_min.root && tempHit.root > kEpsilon) {
                isHit = true;
                hit_min = tempHit;
                tempHit = Hit();
                tempHit.root = std::numeric_limits<float>::infinity();
            }
        }

        // if no intersection
        if (!isHit) {
            sample_colour = world->background;
        } else {
            hit_min.depth = depth;
            hit_min.ray = &ray;
            sample_colour = hit_min.material->shade(hit_min, *world);
        }

        return sample_colour;
    }																																			
}

/********************
 *      CAMERA     *
 ********************/

class Camera {
public:
    // camera position and orientation
    math::Vector eye;
    math::Vector lookat;
    math::Vector up {0, 1, 0};
    float distance; // controls FoV

    // orthonormal basis for camera space
    math::Vector w;
    math::Vector u;
    math::Vector v;

    // world
    World* world;

    Camera(void) {}

    Camera(math::Point e, math::Point l, float d, World* wd):
        eye {e},
        lookat {l},
        distance {d},
        w {glm::normalize(eye - lookat)},
        u {glm::normalize(glm::cross(up, w))},
        v {glm::cross(w, u)},
        world {wd}
    {}

    // Max-to-one
    Colour handleOutOfGamut(Colour c) {
        if (c.x > 1.0f || c.y > 1.0f || c.z > 1.0f) {
            float m { std::max(std::max(c.x, c.y), c.z) };
            return c / m;
        }

        return c;
    }
};

class PinholeCamera : public Camera {
    public:
        PinholeCamera(void) {}

        PinholeCamera(math::Point e, math::Point l, float d, World* wd):
            Camera(e, l, d, wd)
        {}

        // Converts a point on the viewplane to world coordintes
        math::Vector getRayDirection(math::Point2 point) {
            math::Vector direction { (point.x * u) + (point.y * v) - (distance * w) };
            return glm::normalize(direction);
        }

        void renderSlab(std::mutex& image_mutex, std::mutex& workMutex, std::queue<Slab>& workQueue) {
            while (true) {
                workMutex.lock();
                if (!workQueue.empty()) {
                    Slab slab = workQueue.front();
                    workQueue.pop();
                    workMutex.unlock();
                    math::Ray<math::Vector> ray { eye, {0,0,0} }; // initialize to any direction, doesn't matter since it gets set later
                    math::Point2 sample; // position of a pixel on the viewplane in camera space
                    Sampler sampler { world->numSamples, 1 }; 

                    // loop over pixels
                    for (int y{ slab.startY }; y < slab.endY; y++) {
                        for (int x{ slab.startX }; x < slab.endX; x++) {

                            Colour pixel_colour{ 0,0,0 };

                            // loop over samples
                            for (int i = 0; i < sampler.numSamples; i++) {
                                math::Point2 p { sampler.getSample() };

                                // set up camera ray
                                sample.x = x - (world->width * 0.5f) + p.x;
                                sample.y = y - (world->height * 0.5f) + p.y;
                                ray.d = getRayDirection(sample);

                                Colour sample_colour { world->tracer->trace_ray(ray, 0) };

                                pixel_colour += sample_colour; // summing the colour of each sample so we can average later
                            }

                            // set pixel colour
                            pixel_colour = pixel_colour / (float) (sampler.numSamples); // calculate average colour of all samples
                            pixel_colour = handleOutOfGamut(pixel_colour);
                            std::lock_guard<std::mutex> guard(image_mutex);
                            (*(world->image))[x + y * world->height] = pixel_colour;
                        }
                    }
                } else {
                    workMutex.unlock();
                    return;
                }
                workMutex.unlock();
            }
        }

        void render() {
            math::Ray<math::Vector> ray { eye, {0,0,0} }; // initialize to any direction, doesn't matter since it gets set later
            math::Point2 sample; // position of a pixel on the viewplane in camera space
            Sampler sampler { world->numSamples, 1 }; 

            // loop over pixels
            for (int y{ 0 }; y < world->height; y++) {
                for (int x{ 0 }; x < world->width; x++) {

                    Colour pixel_colour{ 0,0,0 };

                    // loop over samples
                    for (int i = 0; i < sampler.numSamples; i++) {
                        math::Point2 p { sampler.getSample() };

                        // set up camera ray
                        sample.x = x - (world->width * 0.5f) + p.x;
                        sample.y = y - (world->height * 0.5f) + p.y;
                        ray.d = getRayDirection(sample);

                        Colour sample_colour { world->tracer->trace_ray(ray, 0) };

                        pixel_colour += sample_colour; // summing the colour of each sample so we can average later
                    }

                    // set pixel colour
                    pixel_colour = pixel_colour / (float) (sampler.numSamples); // calculate average colour of all samples
                    pixel_colour = handleOutOfGamut(pixel_colour);
                    (*(world->image))[x + y * world->height] = pixel_colour;
                }
            }
        }
};

class FishEye : public Camera {
    public:
        float psi_max;

        FishEye(math::Point e, math::Point l, float d, World* wd, float psi):
            Camera(e, l, d, wd),
            psi_max {psi} // in degrees
        {}

        // Converts a point on the viewplane to world coordinates
        math::Vector getRayDirection(math::Point2 point, float& r_squared) {
            math::Point2 pn { 2.0 / world->width * point.x, 2.0 / world->height * point.y};
            r_squared = pn.x * pn.x + pn.y * pn.y;
            if (r_squared <= 1.0f) {
                float r { sqrt(r_squared) };
                float psi { r * psi_max * 0.017453292519943295f };
                float sin_psi { sin(psi) };
                float cos_psi { cos(psi) };
                float sin_alpha { pn.y / r };
                float cos_alpha { pn.x / r };
                math::Vector dir { sin_psi * cos_alpha * u +  sin_psi * sin_alpha * v - cos_psi * w };
                return (glm::normalize(dir));
            }
            return math::Vector {0,0,0};
        }

        void render() {
            math::Ray<math::Vector> ray { eye, {0,0,0} }; // initialize to any direction, doesn't matter since it gets set later
            math::Point2 sample; // position of a pixel on the viewplane in camera space
            float r_squared;
            Sampler sampler { world->numSamples, 1 }; 

            // loop over pixels
            for (int y{ 0 }; y < world->height; y++) {
                for (int x{ 0 }; x < world->width; x++) {

                    Colour pixel_colour{ 0,0,0 };

                    // loop over samples
                    for (int i = 0; i < sampler.numSamples; i++) {
                        math::Point2 p { sampler.getSample() };

                        // set up camera ray
                        sample.x = x - (world->width * 0.5f) + p.x;
                        sample.y = y - (world->height * 0.5f) + p.y;
                        ray.d = getRayDirection(sample, r_squared);

                        if (r_squared <= 1.0) {
                            Colour sample_colour { world->tracer->trace_ray(ray, 0) };
                            pixel_colour += sample_colour; // summing the colour of each sample so we can average later
                        }
                    }

                    // set pixel colour
                    pixel_colour = pixel_colour / (float) (sampler.numSamples); // calculate average colour of all samples
                    pixel_colour = handleOutOfGamut(pixel_colour);
                    (*(world->image))[x + y * world->height] = pixel_colour;
                }
            }
        }
};