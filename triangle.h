#ifndef TRIANGLE
#define TRIANGLE

#include "hitable.h"
float getmin(float a, float b, float c);
float getmax(float a, float b, float c);

class triangle: public hitable  {
    public:
        triangle() {}
        triangle(vec3 A, vec3 B, vec3 C, material *m) : A(A), B(B), C(C), mat_ptr(m)  {};
        virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
        virtual bool bounding_box(float t0, float t1, aabb& box) const;
        vec3 A;
        vec3 B;
        vec3 C;
        material *mat_ptr;
};

float getmin(float a, float b, float c){
    float min = a;
    if(b < min)
        min = b;
    if(c < min)
        min = c;
    return min;
}

float getmax(float a, float b, float c){
    float max = a;
    if(b > max)
        max = b;
    if(c > max)
        max = c;
    return max;
}

bool triangle::bounding_box(float t0, float t1, aabb& box) const {
    //to find min and max bound
    vec3 MINv = vec3( getmin(A[0],B[0],C[0]), getmin(A[1],B[1],C[1]), getmin(A[2],B[2],C[2]));
    vec3 MAXv = vec3( getmax(A[0],B[0],C[0]), getmax(A[1],B[1],C[1]), getmax(A[2],B[2],C[2]));
    box = aabb(MINv, MAXv);
    return true;
}

bool triangle::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
    vec3 E = r.origin();
    vec3 D = r.direction();
    //
    if(dot(D,cross(B-A,C-B)) <= 0)
        return false;
    float a = A[0] - B[0];
    float b = A[1] - B[1];
    float c = A[2] - B[2];
    float d = A[0] - C[0];
    float e = A[1] - C[1];
    float f = A[2] - C[2];
    float g = D[0];
    float h = D[1];
    float i = D[2];
    float j = A[0] - E[0];
    float k = A[1] - E[1];
    float l = A[2] - E[2];
    //Cramer's rule
    float M = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
    float t = -(f*(a*k-j*b) + e*(j*c -a*l) + d*(b*l - k*c))/M;

    if(t < t_min || t > t_max)
        return false;
    float lambda = (i*(a*k -j*b) + h*(j*c - a*l) + g*(b*l - k*c))/M;
    if(lambda < 0 || lambda > 1)
        return false;
    float beta =( j*(e*i - h*f) + k*(g*f - d*i) + l*(d*h - e*g))/M;
    if(beta < 0 || beta > 1 - lambda)
        return false;
    rec.t = t;
    rec.p = r.point_at_parameter(rec.t);
    vec3 normalt = cross(C-B,B-A);
    normalt.make_unit_vector();
    rec.normal = normalt;
    rec.mat_ptr = mat_ptr;
    return true;
}

#endif