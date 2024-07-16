#ifndef _MESH_FNCTS_H_
#define _MESH_FNCTS_H_

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

// delaunay 2d
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include "BMHeap.h"
#include "BoundField_2.h"

// Función para interpolar la altura
//double InterpolateHeight(double A, double B, double C, double D, Point p);

// Función para escanear el triángulo y encontrar el punto con mayor error

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

class FstHApproxMesh {
    public:
        typedef CGAL::Projection_traits_xy_3<K>             Geom_traits;
        typedef typename Geom_traits::FT                    FT;
        typedef typename Geom_traits::Point                 Point;
        typedef CGAL::Delaunay_triangulation_2<Geom_traits> Delaunay;
        typedef typename Delaunay::Face_handle              Face_handle;
        typedef BoundField_2<FT>                            BoundField;
        typedef CGAL::Surface_mesh<Point>                   Mesh;

        FstHApproxMesh(BoundField &height_field): H(height_field) {}
        Mesh operator()(size_t goal_vertices, float goal_error, unsigned int steps) {
            m.clear();
            err_heap.clear();
            face_node.clear();
            INV_STEPS = 1.0/steps;
            VERTEX_GOAL = goal_vertices;
            ERROR_GOAL = goal_error;
            total_error = 0.0;
            // iniciar grid con las esquinas
            m.insert(Point(H.minX(), H.minY(), H(H.minX(), H.minY())));
            m.insert(Point(H.maxX(), H.minY(), H(H.maxX(), H.minY())));
            m.insert(Point(H.maxX(), H.maxY(), H(H.maxX(), H.maxY())));
            m.insert(Point(H.minX(), H.maxY(), H(H.minX(), H.maxY())));
            // se escanea cada triangulo inicial 
            for (Delaunay::Face_handle face : m.finite_face_handles()) {
                scanTriangle(face);
            }

            // mientra no se llega a la meta se revisa el nuevo candidato
            while (!goalMet()) {
                std::pair<Delaunay::Face_handle, Point> t = err_heap.top().content;
                float err = err_heap.top().key(); // Recuperar la llave, que corresponde al error de este punto
                err_heap.pop();
                total_error -= err;
                insert(t.second, t.first); // Insertar el duo punto/cara con mayor error
            }

            Mesh sm; // crear surface mesh a partir de delaunay
            for (Delaunay::Face_handle f : m.finite_face_handles()) {
                Mesh::Vertex_index p0 = sm.add_vertex(f->vertex(0)->point());
                Mesh::Vertex_index p1 = sm.add_vertex(f->vertex(1)->point());
                Mesh::Vertex_index p2 = sm.add_vertex(f->vertex(2)->point());
                sm.add_face(p0, p1, p2);
            }

            return sm;
        };

    private:
        Delaunay m;
        BoundField &H;
        BMHeap<float, std::pair<Face_handle, Point>> err_heap;
        std::unordered_map<Face_handle, HeapNode<float, std::pair<Face_handle, Point>>*> face_node; // each face handle is linked to a node on the heap
        size_t VERTEX_GOAL;
        float ERROR_GOAL, total_error;
        FT INV_STEPS;

        std::unordered_map<Delaunay::Face_handle, std::pair<Point, double>> candPos; 
        // Mesh::Property_map<face_descriptor, vertex_descriptor> candPos;
        // std::unordered_map<face_descriptor, std::vector<vertex_descriptor>> pointsInTriangle; 
        // Mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>> pointsInTriangle;

        // points to check from centroid to every direction

        void scanTriangle(Delaunay::Face_handle triangle) {
            // Triangles are only scanned once, so we create the heap node here
            Point best;
            float maxError = 0.0;

            Point p0 = triangle->vertex(0)->point();
            Point p1 = triangle->vertex(1)->point();
            Point p2 = triangle->vertex(2)->point();

            Point centroid = CGAL::centroid(p0, p1, p2);
            /*
            std::vector<Point> searchPoints = generateSearchPoints(centroid, p0, p1, p2);
            double A = (p1.y() - p0.y()) * (p2.z() - p0.z()) - (p1.z() - p0.z()) * (p2.y() - p0.y());
            double B = (p1.z() - p0.z()) * (p2.x() - p0.x()) - (p1.x() - p0.x()) * (p2.z() - p0.z());
            double C = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p1.y() - p0.y()) * (p2.x() - p0.x());
            double D = -(A * p0.x() + B * p0.y() + C * p0.z());

            // cambiar por uso de aproximacion dentro de triangulo
            for (const auto& pt : searchPoints) {
                double heightFieldZ = H(pt.x(), pt.y());
                double interpolatedHeight = -(A * pt.x() + B * pt.y() + D) / C;
                float error = std::abs(interpolatedHeight - pt.z());
                if (error > maxError) {
                    maxError = error;
                    best = pt;
                }
            }
            */

            maxError = abs(centroid.z() - H(centroid.x(), centroid.y()));
            best = centroid;
            total_error += maxError;

            // insertar el triangulo en el heap y asociarlo a este triangulo
            HeapNode<float, std::pair<Face_handle, Point>> *this_triangle_node = &(err_heap.push(maxError, std::make_pair(triangle, best)));
            face_node.emplace(triangle, this_triangle_node);

            return;
        }

        void insert(Point p, Delaunay::Face_handle f_hint) {
            Delaunay::Vertex_handle vh = m.insert(p);
            // Obtener las nuevas caras generadas por la inserción y escanearlas
            Delaunay::Face_circulator new_f_start = m.incident_faces(vh);
            Delaunay::Face_circulator new_f = new_f_start;
            do {
                if (!m.is_infinite(new_f)) {
                    scanTriangle(new_f);
                }
            } while (++new_f != new_f_start);
            return;
        }

        bool isPointInTriangle(double x, double y, const Point& p0, const Point& p1, const Point& p2) {
            K::Point_2 p(x, y);
            K::Point_2 p0d(p0.x(), p0.y());
            K::Point_2 p1d(p1.x(), p1.y());
            K::Point_2 p2d(p2.x(), p2.y());
            double a0 = CGAL::area(p0d, p1d, p2d);
            double a1 = CGAL::area(p, p1d, p2d);
            double a2 = CGAL::area(p0d, p, p2d);
            double a3 = CGAL::area(p0d, p1d, p);
            return abs(a0 - (a1 + a2 + a3)) < 0.00001;
        }

        /*
        std::vector<Point> generateSearchPoints(const Point& centroid, const Point& p0, const Point& p1, const Point& p2) {
            std::vector<Point> points;

            auto addPointsInDirection = [&](const Point& start, const Point& direction) {
                for (double i = 0.0; i < 1.0; i+=INV_STEPS) {
                    double newX = start.x() + i * (direction.x() - start.x());
                    double newY = start.y() + i * (direction.y() - start.y());
                    if (isPointInTriangle(newX, newY, p0, p1, p2)) {
                        points.emplace_back(newX, newY, H(newX, newY)); // z-value will be determined by the height field H
                    }
                }
            };

            // Generate points towards each vertex
            addPointsInDirection(centroid, p0);
            addPointsInDirection(centroid, p1);
            addPointsInDirection(centroid, p2);

            // Generate points towards the midpoints of each side
            Point midpoint0((p0.x() + p1.x()) / 2, (p0.y() + p1.y()) / 2, 0);
            Point midpoint1((p1.x() + p2.x()) / 2, (p2.y() + p2.y()) / 2, 0);
            Point midpoint2((p2.x() + p0.x()) / 2, (p0.y() + p0.y()) / 2, 0);

            addPointsInDirection(centroid, midpoint0);
            addPointsInDirection(centroid, midpoint1);
            addPointsInDirection(centroid, midpoint2);

            return points;
        }
        */


        bool goalMet() {
            return (m.number_of_vertices() >= VERTEX_GOAL);// || (total_error < ERROR_GOAL);
        }
};

#endif // _MESH_FNCTS_H_