#include <CGAL/boost/graph/generators.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <vector>
#include <cmath>
#include <set>

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;


// Función para interpolar la altura
double InterpolateHeight(double A, double B, double C, double D, K::Point_3 p) {
    return -(A * p.x() + B * p.y() + D) / C;
}

// Función para escanear el triángulo y encontrar el punto con mayor error
void scanTriangle(Mesh* m, face_descriptor triangle,
                  Mesh::Property_map<face_descriptor, vertex_descriptor>& candPos,
                  Mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>>& pointsInTriangle) {
    vertex_descriptor best = Mesh::null_vertex();
    double maxError = 0.0;

    Mesh::Halfedge_index h = m->halfedge(triangle);
    vertex_descriptor vd0 = m->source(h);
    vertex_descriptor vd1 = m->source(m->next(h));
    vertex_descriptor vd2 = m->source(m->next(m->next(h)));

    K::Point_3 p0 = m->point(vd0);
    K::Point_3 p1 = m->point(vd1);
    K::Point_3 p2 = m->point(vd2);

    double A = (p1.y() - p0.y()) * (p2.z() - p0.z()) - (p1.z() - p0.z()) * (p2.y() - p0.y());
    double B = (p1.z() - p0.z()) * (p2.x() - p0.x()) - (p1.x() - p0.x()) * (p2.z() - p0.z());
    double C = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p1.y() - p0.y()) * (p2.x() - p0.x());
    double D = -(A * p0.x() + B * p0.y() + C * p0.z());

    for (vertex_descriptor vd : pointsInTriangle[triangle]) {
        K::Point_3 p = m->point(vd);
        double interpolatedHeight = InterpolateHeight(A, B, C, D, p);
        double error = std::abs(interpolatedHeight - p.z());
        if (error > maxError) {
            maxError = error;
            best = vd;
        }
    }

    candPos[triangle] = best;
    // lógica para actualizar "heap"
}

void meshInsert(Mesh* m, vertex_descriptor p, face_descriptor triangle, Mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>>& pointsInTriangle) {
    // Obtener las medias aristas del triángulo
    Mesh::Halfedge_index h = m->halfedge(triangle);
    Mesh::Halfedge_index h1 = m->next(h);
    Mesh::Halfedge_index h2 = m->next(h1);

    // Obtener los vértices del triángulo
    vertex_descriptor vd0 = m->source(h);
    vertex_descriptor vd1 = m->source(h1);
    vertex_descriptor vd2 = m->source(h2);

    // Eliminar el triángulo original
    m->remove_face(triangle);

    // Añadir tres nuevas caras formadas por el nuevo vértice y los vértices del triángulo original
    face_descriptor f0 = m->add_face(p, vd0, vd1);
    face_descriptor f1 = m->add_face(p, vd1, vd2);
    face_descriptor f2 = m->add_face(p, vd2, vd0);

    if (f0 == Mesh::null_face() || f1 == Mesh::null_face() || f2 == Mesh::null_face()) {
        std::cerr << "Error: Failed to add new faces." << std::endl;
    }

    // Actualizar pointsInTriangle para los nuevos triángulos
    std::set<vertex_descriptor> original_points(pointsInTriangle[triangle].begin(), pointsInTriangle[triangle].end());
    original_points.insert(p); // Añadir el nuevo vértice a los puntos originales

    // Asignar puntos a los nuevos triángulos
    pointsInTriangle[f0] = {p, vd0, vd1};
    pointsInTriangle[f1] = {p, vd1, vd2};
    pointsInTriangle[f2] = {p, vd2, vd0};

    // Repartir los puntos originales en los nuevos triángulos
    for (vertex_descriptor vd : original_points) {
        if (vd == vd0 || vd == vd1 || vd == vd2 || vd == p) {
            continue; // Saltar los vértices del triángulo original y el nuevo vértice
        }
        K::Point_3 point = m.point(vd);
        if (CGAL::collinear(m.point(vd0), m.point(vd1), point)) {
            pointsInTriangle[f0].push_back(vd);
        } else if (CGAL::collinear(m.point(vd1), m.point(vd2), point)) {
            pointsInTriangle[f1].push_back(vd);
        } else if (CGAL::collinear(m.point(vd2), m.point(vd0), point)) {
            pointsInTriangle[f2].push_back(vd);
        }
    }

    // Eliminar la entrada del triángulo original en pointsInTriangle
    pointsInTriangle.remove(triangle);
}

void sortTrianglePoints(Mesh* m, face_descriptor f0, face_descriptor f1, face_descriptor f2, std::vector<vertex_descriptor> points) {

}