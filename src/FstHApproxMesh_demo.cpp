#include <cmath>
#include <random>
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


FT simple_terrain_func(FT x, FT y, const siv::PerlinNoise& perlin) {
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
    // Generate a random seed
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<siv::PerlinNoise::seed_type> dis;
    siv::PerlinNoise::seed_type seed = dis(gen);

    // Initialize Perlin noise with the random seed
    const siv::PerlinNoise perlin{seed};

    int mode = std::stoi(argv[1]);
    int MAX_POINTS = std::stoi(argv[2]);
    switch (mode) {
        case 1: {
            std::cout << "Random noise function" << std::endl;
            FT hlf_grid_size = std::stod(argv[3]);
            const char* out_file_name = argv[4];
            FunctionField_2<FT> HF([&perlin](FT x, FT y) { return simple_terrain_func(x, y, perlin); }, -hlf_grid_size, -hlf_grid_size, hlf_grid_size, hlf_grid_size);
            FstHApproxMesh fham_field(HF);
            CGAL::Surface_mesh<Point> function_mesh = fham_field(MAX_POINTS, 0.01, 6);
            CGAL::IO::write_OBJ(out_file_name, function_mesh);
            break;
        }
        case 2: {
            std::cout << "Raster function" << std::endl;
            float MAX_Z = std::stof(argv[3]);
            const char* in_file_name = argv[4];
            const char* out_file_name = argv[5];
            RasterFileField_2<FT> HR(in_file_name, MAX_Z, RasterInterpolationType::BILINEAR);
            FstHApproxMesh fham_raster(HR);
            CGAL::Surface_mesh<Point> raster_mesh = fham_raster(MAX_POINTS, 0.01, 6);
            CGAL::IO::write_OBJ(out_file_name, raster_mesh);
            break;
        }
    }
    return 0;
}