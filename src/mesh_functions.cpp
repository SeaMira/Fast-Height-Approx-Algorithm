#include <MeshFunctions/mesh_functions.hpp>

// Función para interpolar la altura
double InterpolateHeight(double A, double B, double C, double D, K::Point_3 p) {
    return -(A * p.x() + B * p.y() + D) / C;
}

// Función para escanear el triángulo y encontrar el punto con mayor error
void FstHAproxMesh::scanTriangle(face_descriptor triangle) {
    vertex_descriptor best = Mesh::null_vertex();
    double maxError = 0.0;

    Mesh::Halfedge_index h = m.halfedge(triangle);
    vertex_descriptor vd0 = m.source(h);
    vertex_descriptor vd1 = m.source(m.next(h));
    vertex_descriptor vd2 = m.source(m.next(m.next(h)));

    K::Point_3 p0 = m.point(vd0);
    K::Point_3 p1 = m.point(vd1);
    K::Point_3 p2 = m.point(vd2);

    double A = (p1.y() - p0.y()) * (p2.z() - p0.z()) - (p1.z() - p0.z()) * (p2.y() - p0.y());
    double B = (p1.z() - p0.z()) * (p2.x() - p0.x()) - (p1.x() - p0.x()) * (p2.z() - p0.z());
    double C = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p1.y() - p0.y()) * (p2.x() - p0.x());
    double D = -(A * p0.x() + B * p0.y() + C * p0.z());

    for (vertex_descriptor vd : pointsInTriangle[triangle]) {
        K::Point_3 p = m.point(vd);
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

void FstHAproxMesh::meshInsert(vertex_descriptor p, face_descriptor triangle) {
    // Obtener las medias aristas del triángulo
    Mesh::Halfedge_index h = m.halfedge(triangle);
    Mesh::Halfedge_index h1 = m.next(h);
    Mesh::Halfedge_index h2 = m.next(h1);

    // Obtener los vértices del triángulo
    vertex_descriptor vd0 = m.source(h);
    vertex_descriptor vd1 = m.source(h1);
    vertex_descriptor vd2 = m.source(h2);

    // Eliminar el triángulo original
    m.remove_face(triangle);

    // Añadir tres nuevas caras formadas por el nuevo vértice y los vértices del triángulo original
    face_descriptor f0 = m.add_face(p, vd0, vd1);
    face_descriptor f1 = m.add_face(p, vd1, vd2);
    face_descriptor f2 = m.add_face(p, vd2, vd0);

    if (f0 == Mesh::null_face() || f1 == Mesh::null_face() || f2 == Mesh::null_face()) {
        std::cerr << "Error: Failed to add new faces." << std::endl;
    }

    // Actualizar pointsInTriangle para los nuevos triángulos
    std::vector<vertex_descriptor> original_points(pointsInTriangle[triangle].begin(), pointsInTriangle[triangle].end());
    original_points.push_back(p); // Añadir el nuevo vértice a los puntos originales

    // Asignar puntos a los nuevos triángulos
    pointsInTriangle[f0] = {p, vd0, vd1};
    pointsInTriangle[f1] = {p, vd1, vd2};
    pointsInTriangle[f2] = {p, vd2, vd0};

    // Repartir los puntos originales en los nuevos triángulos
    sortTrianglePoints(f0, f1, f2, original_points);

    // Eliminar la entrada del triángulo original en pointsInTriangle
    pointsInTriangle.erase(triangle);
}

void FstHAproxMesh::sortTrianglePoints(face_descriptor f0, face_descriptor f1, face_descriptor f2, std::vector<vertex_descriptor> points) {
    // Función auxiliar para verificar si un punto está dentro de un triángulo
    auto isPointInTriangle = [this](K::Point_3 p, K::Point_3 a, K::Point_3 b, K::Point_3 c) -> bool {
        if (c == p || b == p || a == p) return false;
        // Calcula los vectores de los lados del triángulo
        K::Vector_3 v0 = c - a;
        K::Vector_3 v1 = b - a;
        K::Vector_3 v2 = p - a;

        // Calcula los productos escalares
        double dot00 = v0 * v0;
        double dot01 = v0 * v1;
        double dot02 = v0 * v2;
        double dot11 = v1 * v1;
        double dot12 = v1 * v2;

        // Calcula las coordenadas baricéntricas
        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        // Verifica si el punto está dentro del triángulo
        return (u >= 0) && (v >= 0) && (u + v < 1);
    };

    // Obtener los vértices de los triángulos
    Mesh::Halfedge_index h0 = m.halfedge(f0);
    Mesh::Halfedge_index h1 = m.halfedge(f1);
    Mesh::Halfedge_index h2 = m.halfedge(f2);

    vertex_descriptor vd0_0 = m.source(h0);
    vertex_descriptor vd0_1 = m.source(m.next(h0));
    vertex_descriptor vd0_2 = m.source(m.next(m.next(h0)));

    vertex_descriptor vd1_0 = m.source(h1);
    vertex_descriptor vd1_1 = m.source(m.next(h1));
    vertex_descriptor vd1_2 = m.source(m.next(m.next(h1)));

    vertex_descriptor vd2_0 = m.source(h2);
    vertex_descriptor vd2_1 = m.source(m.next(h2));
    vertex_descriptor vd2_2 = m.source(m.next(m.next(h2)));

    K::Point_3 p0_0 = m.point(vd0_0);
    K::Point_3 p0_1 = m.point(vd0_1);
    K::Point_3 p0_2 = m.point(vd0_2);

    K::Point_3 p1_0 = m.point(vd1_0);
    K::Point_3 p1_1 = m.point(vd1_1);
    K::Point_3 p1_2 = m.point(vd1_2);

    K::Point_3 p2_0 = m.point(vd2_0);
    K::Point_3 p2_1 = m.point(vd2_1);
    K::Point_3 p2_2 = m.point(vd2_2);

    // Clasificar los puntos en los triángulos correspondientes
    for (vertex_descriptor vd : points) {
        K::Point_3 point = m.point(vd);
        if (isPointInTriangle(point, p0_0, p0_1, p0_2)) {
            pointsInTriangle[f0].push_back(vd);
        } else if (isPointInTriangle(point, p1_0, p1_1, p1_2)) {
            pointsInTriangle[f1].push_back(vd);
        } else if (isPointInTriangle(point, p2_0, p2_1, p2_2)) {
            pointsInTriangle[f2].push_back(vd);
        }
    }
}