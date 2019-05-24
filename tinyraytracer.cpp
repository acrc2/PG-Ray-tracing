#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "geometry.h"

using namespace std;
struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};
 
struct Material {
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec, string nome) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec),nome(nome) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() ,nome(){}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
    string nome;
};
 
struct Sphere {
    Vec3f center;
    float radius;
    Material material;
 
    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}
 
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};
 
Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}
 
Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}
 
bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
 
    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(dir.y)>1e-3)  {
        float d = -(orig.y+4)/dir.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + dir*d;
        if (d>0 && fabs(pt.x)<10 && pt.z<-10 && pt.z>-30 && d<spheres_dist) {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0,1,0);
            material.diffuse_color = (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 ? Vec3f(0, 0, 0) : Vec3f(1, 1, 1);
        }
    }
    return std::min(spheres_dist, checkerboard_dist)<1000;
}
 
Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth=0) {
    Vec3f point, N;
    Material material;
 
    if (depth>4 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }
 
    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);
 
    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();
 
        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;
 
        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}
 
Vec3f random_in_unit_disk() {
    Vec3f p;
    do {
        p = Vec3f(drand48(),drand48(),0)*2.0 - Vec3f(1,1,0);
    } while (p.norm()*p.norm() >= 1.0);
    return p;
}
 
void render(Vec3f lookfrom , Vec3f lookat,Vec3f vup, float vfov, float aspect,float aperture, float focus_dist,const std::vector<Sphere> &spheres, const std::vector<Light> &lights,float nx,float ny) {
    float lens_radius = aperture/2;
    const int   width    = 1024;
    const int   height   = 768;
    const int   ns       = 4;
    const float fov      = M_PI/180;
    std::vector<Vec3f> framebuffer(width*height);
 
    #pragma omp parallel for
    for (size_t j = 0; j<height; j++) { // actual rendering loop
        for (size_t i = 0; i<width; i++) {
           
            Vec3f u,v,w;
            Vec3f col = Vec3f(0,0,0);
            for (size_t h = 0; h < ns; h++){
                float theta =  vfov*fov;
                float half_height = tan(theta/2);
                float half_width =  aspect * half_height;
                Vec3f origin = lookfrom;
                w = (lookfrom -  lookat).normalize();
                u = cross(vup,w).normalize();
   
                v = cross(w,u);
                v = -v;
                Vec3f lower_left_corner = Vec3f(-half_width,-half_height,-1.0);
                lower_left_corner = origin - (u * half_width * focus_dist) - (v * half_height * focus_dist) - w * focus_dist;
                Vec3f horizontal =   u * half_width * 2 * focus_dist;
                Vec3f vertical = v * half_height * 2 * focus_dist;
               
                float s = (i+drand48())/width;
                float t = (j+drand48())/height;
                Vec3f rd = random_in_unit_disk() * lens_radius;
                Vec3f offset = (u * rd.x) + (v * rd.y);
               
   
                Vec3f result =  lower_left_corner +  (horizontal * s) + (vertical * t) - origin - offset;
                col = col + (cast_ray(lookfrom + offset, result.normalize(), spheres, lights));
                //result =  - result;
                /*float dir_x =  (i + 0.5) -  width/2.;
                float dir_y = -(j + 0.5) + height/2.;    // this flips the image at the same time
                float dir_z = -height/(2.*tan(fov/2.));*/    
           
   
                //framebuffer[i+j*width] = cast_ray(Vec3f(13,2,3), Vec3f(dir_x, dir_y , dir_z).normalize(), spheres, lights);
            }
            col = col * (1/(float)ns);
            //col = Vec3f(sqrt(col[0]),sqrt(col[1]),sqrt(col[2]));
            framebuffer[i+j*width] = col;
 
        }
    }
 
    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out.ppm",std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}
 
int main() {
    std::ifstream myfile;
    myfile.open("input.txt");
    string s;
    int nx, ny;
    float px ,py ,pz, tx, ty, tz, ux, uy, uz,fov;
    std::vector<Sphere> spheres;
    vector<Material> materiais;
    string trash;
    
    bool r = false, c = false, m = false, o = false;;
 
    while (getline(myfile, s)) {

            
        //cout<<s<<endl;
        if (s[0] == '#') {
            if (s[1] == 'r' || s[1] == 'R') {
                r = true;
            } else if (s[1] == 'c' && s[2] != 'o') {
                c = true;   
            } else if (s[1] == 'm' || s[1] == 'M') {
                m = true;
            } else if (s[1] == 'o' || s[1] == 'O') {
                o = true;
            }
        }
        if (r) { //ok!
            r = false;
            myfile >>trash>> nx >> ny;
            //cout<<nx<<" "<<ny<<endl;
        } else if (c) {
            c = false;
            myfile >>trash>>px >> py >> pz >> tx >> ty >> tz >> ux >> uy >> uz >>fov;
            //cout<<trash<<" "<<px <<" "<< py <<" "<< pz <<" "<< tx <<" "<< ty <<" "<< tz <<" "<< ux <<" "<< uy <<" "<< uz<<" "<<fov;
        } else if (m) {
            m = false;
            while (myfile >> s && s[0] == 'm') {                
                float r,g,b,kd,ks,ke,alpha,IR,kr;
                string nome;
                myfile >>nome >>r>>g>>b >>kd>>ks>>ke>>alpha>>IR>>kr;
                materiais.push_back(Material(IR,Vec4f(kd,ke,ks,kr),Vec3f(r,g,b),alpha,nome));
            }
            
        } else if (o) {
            o = false;
            
            while (myfile>> s && s[0] == 's') {
                float cx,cy,cz, r;
                string materialName;
                myfile>>cx>>cy>>cz>>r>>materialName;
                int pos = -1;
                for(int index = 0 ; index <materiais.size();index++){
                    if(materialName == materiais[index].nome){
                        pos = index;
                    }
                }
                //cout<<materiais[pos].nome;
                spheres.push_back(Sphere(Vec3f( cx, cy,cz), r, materiais[pos]));
                
            }

        }
    }
    myfile.close();

 
    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));
    lights.push_back(Light(Vec3f( 30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f( 30, 20,  30), 1.7));
 
 
    Vec3f lookfrom = Vec3f(px,py,pz);
    Vec3f lookat = Vec3f(tx, ty, tz);
    float dist_to_focus = (lookat - lookfrom).norm();
    float aperture = 0.05;
    //Parametros da camera.
    render(lookfrom,lookat,Vec3f(ux,uy,uz),fov,float(nx)/float(ny),aperture, dist_to_focus, spheres, lights,nx,ny);
 
    return 0;
}