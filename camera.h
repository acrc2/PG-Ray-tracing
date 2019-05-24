//==================================================================================================
// Written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is distributed
// without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication along
// with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==================================================================================================

#ifndef CAMERAH
#define CAMERAH
#include "tinyraytracer.cpp"
#include "geometry.h"

Vec3f random_in_unit_disk() {
    Vec3f p;
    do {
        p = 2.0*Vec3f(drand48(),drand48(),0) - Vec3f(1,1,0);
    } while (dot(p,p) >= 1.0);
    return p;
}

class camera {
    public:
        camera(Vec3f lookfrom, Vec3f lookat, Vec3f vup, float vfov, float aspect, float aperture, float focus_dist) { // vfov is top to bottom in degrees
            lens_radius = aperture / 2;
            float theta = vfov*M_PI/180;
            float half_height = tan(theta/2);
            float half_width = aspect * half_height;
            origin = lookfrom;
            w = unit_vector(lookfrom - lookat);
            u = unit_vector(cross(vup, w));
            v = cross(w, u);
            lower_left_corner = origin  - half_width*focus_dist*u -half_height*focus_dist*v - focus_dist*w;
            horizontal = 2*half_width*focus_dist*u;
            vertical = 2*half_height*focus_dist*v;
        }
        ray get_ray(float s, float t) {
            Vec3f rd = lens_radius*random_in_unit_disk();
            Vec3f offset = u * rd.x() + v * rd.y();
            return ray(origin + offset, lower_left_corner + s*horizontal + t*vertical - origin - offset); 
        }

        Vec3f origin;
        Vec3f lower_left_corner;
        Vec3f horizontal;
        Vec3f vertical;
        Vec3f u, v, w;
        float lens_radius;
};
#endif



