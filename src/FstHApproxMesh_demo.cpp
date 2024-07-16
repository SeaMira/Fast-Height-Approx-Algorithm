#include <cmath>

#include "PerlinNoise.hpp"
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/OBJ/File_writer_wavefront.h>
#include <CGAL/IO/OBJ.h>

#include "BoundField_2/RasterFileField_2.h"
#include "BoundField_2/FunctionField_2.h"

#include "MeshFunctions/mesh_functions.hpp"


// This demo is the same as BoundField_demo, except it uses FstHApproxMesh (our implementation) to compute the mesh.

const int SIZE = 64;

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
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


int main(int argc, char **argv) {
    FunctionField_2<FT> HF(simple_terrain_func, -10.0, -10.0, 10.0, 10.0);
    RasterFileField_2<FT> HR("image.png", 1.0, RasterInterpolationType::BILINEAR);

    FstHApproxMesh fham_field(HF);
    FstHApproxMesh fham_raster(HR);

    CGAL::Surface_mesh<Point> function_mesh = fham_field(5000, 0.01, 24);
    CGAL::Surface_mesh<Point> raster_mesh = fham_raster(5000, 0.01, 24);

    CGAL::IO::write_OBJ("fst_function_mesh.obj", function_mesh);
    CGAL::IO::write_OBJ("fst_raster_mesh.obj", raster_mesh);

    return 0;
}