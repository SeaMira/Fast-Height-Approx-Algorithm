#include <cmath>

#include "PerlinNoise.hpp"
#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/IO/OBJ/File_writer_wavefront.h>
#include <CGAL/IO/OBJ.h>

#include "BoundField_2/RasterFileField_2.h"
#include "BoundField_2/FunctionField_2.h"


// This example creates a SIZE*SIZE square grid from a BoundField using a random terrain function,
// and then with some image.png stored in the same directory.
// It then saves the generated meshes in OBJ format.

const int SIZE = 64;

using K = CGAL::Simple_cartesian<double>;
using Point = K::Point_3;
using Mesh = CGAL::Surface_mesh<Point>;
using FT = K::FT;

const siv::PerlinNoise::seed_type seed = 123456u;
const siv::PerlinNoise perlin{seed};


FT simple_terrain_func(FT x, FT y) {
    FT z = 1.0  * perlin.noise2D_01(x, y)
             + 0.5  * perlin.noise2D_01(2.0*x, 2.0*y)
             + 0.25 * perlin.noise2D_01(4.0*x, 4.0*y);
    z = z / (1.0 + 0.5 + 0.25);
    return pow(z, 3.0);
}


FT lerp(FT a, FT b, FT t) {
    if (t == 0.0) return a;
    if (t == 1.0) return b;
    return a + t * (b - a);
}


void gen_and_save(BoundField_2<FT> &H, std::string fname) {
    Mesh m;

    // vertices
    for (int idx_x = 0; idx_x < (SIZE - 1); idx_x++) {
        for (int idx_y = 0; idx_y < (SIZE-1); idx_y++) {
            FT x = lerp(H.minX(), H.maxX(), (FT)idx_x / (FT)SIZE);
            FT y = lerp(H.minY(), H.maxY(), (FT)idx_y / (FT)SIZE);

            FT xr = lerp(H.minX(), H.maxX(), (FT)(idx_x + 1) / (FT)SIZE);
            FT yu = lerp(H.minY(), H.maxY(), (FT)(idx_y + 1) / (FT)SIZE);

            auto p = m.add_vertex(Point(x, y, H(x,y)));
            auto pr = m.add_vertex(Point(xr, y, H(xr,y)));
            auto pu = m.add_vertex(Point(x, yu, H(x,yu)));
            auto pd = m.add_vertex(Point(xr, yu, H(xr,yu)));

            m.add_face(p, pr, pd);
            m.add_face(p, pd, pu);
        }
    }

    for (const auto &v : m.vertices()) {
        std::cout << m.point(v) << std::endl;
    }

    // save to .obj
    std::ofstream os(fname);
    CGAL::IO::write_OBJ(fname, m);
}


int main(int argc, char **argv) {
    FunctionField_2<FT> HF(simple_terrain_func, -10.0, -10.0, 10.0, 10.0);
    RasterFileField_2<FT> HR("image.png", 1.0, RasterInterpolationType::BILINEAR);

    gen_and_save(HF, "functionfield.obj");
    gen_and_save(HR, "rasterfield.obj");

    return 0;
}