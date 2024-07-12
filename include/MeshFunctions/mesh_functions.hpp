#ifndef _MESH_FNCTS_H_
#define _MESH_FNCTS_H_

// delaunay 2d
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef K::Point_3   Point;
typedef Delaunay::Vertex_handle Vertex_handle;
typedef Delaunay::Face_handle Face_handle;

#include "BoundField_2/RasterFileField_2.h"
#include "BoundField_2/FunctionField_2.h"
#include "PerlinNoise.hpp"
// msh y lectores
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/OBJ/File_writer_wavefront.h>
#include <CGAL/IO/OBJ.h>

#include "BoundField_2/RasterFileField_2.h"
#include "BoundField_2/FunctionField_2.h"

const int SIZE = 64;

using Mesh = CGAL::Surface_mesh<Point>;
using FT = K::FT;

const siv::PerlinNoise::seed_type seed = 123456u;
const siv::PerlinNoise perlin{seed};

// Incluye otros encabezados de C++ y Boost
#include <cassert>
#include <cfloat>
#include <vector>
#include <cmath>
#include <cstdlib> 
#include <ctime> 
#include <iostream>
#include <fstream>
#include <map>
#include <random>
#include <unordered_map>
#include <unordered_set> 
#include <algorithm> 

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





// Función para interpolar la altura
double InterpolateHeight(double A, double B, double C, double D, Point p);

// Función para escanear el triángulo y encontrar el punto con mayor error


class FstHAproxMesh {
    private:
        Delaunay m;
        BoundField_2<FT> &H;

        // grid corners
        double left_x, left_y, right_x, right_y;

        std::unordered_map<Face_handle, Vertex_handle> candPos; 
        // Mesh::Property_map<face_descriptor, vertex_descriptor> candPos;
        // std::unordered_map<face_descriptor, std::vector<vertex_descriptor>> pointsInTriangle; 
        // Mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>> pointsInTriangle;

    public:
        void scanTriangle(Face_handle triangle);
        Vertex_handle meshInsert(Point p);
        void insert(Point p);
        void greedyInsert():
        // void sortTrianglePoints(face_descriptor f0, face_descriptor f1, face_descriptor f2, std::vector<vertex_descriptor> points);

        FstHAproxMesh(double left_x, double left_y, double right_x, double right_y);
        FstHAproxMesh(const std::string &path, FT max_scalar, RasterInterpolationType interp = RasterInterpolationType::BILINEAR);
};

#endif // _MESH_FNCTS_H_