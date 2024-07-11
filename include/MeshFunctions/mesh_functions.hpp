#include <CGAL/boost/graph/generators.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <vector>
#include <cmath>
#include <set>
#include <unordered_map>

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;


// Función para interpolar la altura
double InterpolateHeight(double A, double B, double C, double D, K::Point_3 p);

// Función para escanear el triángulo y encontrar el punto con mayor error




class FstHAproxMesh {
    private:
        Mesh m;
        
        std::unordered_map<face_descriptor, vertex_descriptor> candPos; 
        // Mesh::Property_map<face_descriptor, vertex_descriptor> candPos;
        // std::unordered_map<face_descriptor, std::vector<vertex_descriptor>> pointsInTriangle; 
        // Mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>> pointsInTriangle;

    public:
        void scanTriangle(face_descriptor triangle);
        void meshInsert(vertex_descriptor p, face_descriptor triangle);
        void sortTrianglePoints(face_descriptor f0, face_descriptor f1, face_descriptor f2, std::vector<vertex_descriptor> points);

        FstHAproxMesh();
};