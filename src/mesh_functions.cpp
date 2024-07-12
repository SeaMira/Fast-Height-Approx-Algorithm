#include <MeshFunctions/mesh_functions.hpp>

// constructor para el function field
FstHAproxMesh::FstHAproxMesh(double left_x, double left_y, double right_x, double right_y) {
    H = FunctionField_2<FT>(simple_terrain_func, left_x, left_y, right_x, right_y);
    this->left_x = left_x;
    this->left_y = left_y;
    this->right_x = right_x;
    this->right_y = right_y;
}


FstHAproxMesh::FstHAproxMesh(const std::string &path, FT max_scalar, RasterInterpolationType interp = RasterInterpolationType::BILINEAR) {
    H = RasterFileField_2<FT>(path, max_scalar, interp);
    this->left_x = H.minX();
    this->left_y = H.minY();
    this->right_x = H.maxX();
    this->right_y = H.maxY();
}


// Función para interpolar la altura
double InterpolateHeight(double A, double B, double C, double D, Point p) {
    return -(A * p.x() + B * p.y() + D) / C;
}

// Función para escanear el triángulo y encontrar el punto con mayor error
void FstHAproxMesh::scanTriangle(Face_handle triangle) {
    Vertex_handle best;
    double maxError = 0.0;

    Point p0 = triangle->vertex(0)->point();
    Point p1 = triangle->vertex(1)->point();
    Point p2 = triangle->vertex(2)->point();

    double A = (p1.y() - p0.y()) * (p2.z() - p0.z()) - (p1.z() - p0.z()) * (p2.y() - p0.y());
    double B = (p1.z() - p0.z()) * (p2.x() - p0.x()) - (p1.x() - p0.x()) * (p2.z() - p0.z());
    double C = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p1.y() - p0.y()) * (p2.x() - p0.x());
    double D = -(A * p0.x() + B * p0.y() + C * p0.z());


    // cambiar por uso de aproximacion dentro de triangulo
    for (Vertex_handle vh : pointsIntriangle[triangle]) {
        Point p = vh->point();
        double interpolatedHeight = -(A * p.x() + B * p.y() + D) / C;
        double error = std::abs(interpolatedHeight - p.z());
        if (error > maxError) {
            maxError = error;
            best = vh;
        }
    }
    if 

    MAX_ERROR += maxError;
    candPos[triangle] = std::make_pair(best, maxError);
    // lógica para actualizar "heap"
}

Vertex_handle FstHAproxMesh::meshInsert(Point p) {
    // Insertar el nuevo punto en la triangulación de Delaunay
    // y mantener la condición de triangulación
    return m.insert(p);
}

void FstHAproxMesh::insert(Point p) {
    Vertex_handle vh = meshInsert(p);

    // Obtener las nuevas caras generadas por la inserción
    std::vector<Face_handle> new_faces;
    m.incident_faces(vh, std::back_inserter(new_faces));
    for (Face_handle new_face : new_faces) {
        scanTriangle(new_face);
    }
}


void FstHAproxMesh::greedyInsert() {
    // iniciar grid con las esquinas
    m.insert(Point(left_x, left_y, H(left_x, left_y)));
    m.insert(Point(right_x, left_y, H(right_x, left_y)));
    m.insert(Point(right_x, right_y, H(right_x, right_y)));
    m.insert(Point(left_x, right_y, H(left_x, right_y)));

    // se escanea cada triangulo inicial 
    for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
        scanTriangle(face);
    }

    // mientra no se llega a la meta se revisa el nuevo candidato
    while (!goalMet()) {
        // T <- Heap_Delete_Max :: tiene que retornar un Face_handle
        // std::pair cand = candPos[T];
        // MAX_ERROR -= cand.second;
        // insert(cand.first);
    }
}

bool FstHAproxMesh::goalMet() {
    return m.number_of_vertices() >= VERTEX_GOAL || MAX_ERROR < MAX_ERROR_GOAL;
}













// void FstHAproxMesh::sortTrianglePoints(face_descriptor f0, face_descriptor f1, face_descriptor f2, std::vector<vertex_descriptor> points) {
//     // Función auxiliar para verificar si un punto está dentro de un triángulo
//     auto isPointInTriangle = [this](K::Point_3 p, K::Point_3 a, K::Point_3 b, K::Point_3 c) -> bool {
//         if (c == p || b == p || a == p) return false;
//         // Calcula los vectores de los lados del triángulo
//         K::Vector_3 v0 = c - a;
//         K::Vector_3 v1 = b - a;
//         K::Vector_3 v2 = p - a;

//         // Calcula los productos escalares
//         double dot00 = v0 * v0;
//         double dot01 = v0 * v1;
//         double dot02 = v0 * v2;
//         double dot11 = v1 * v1;
//         double dot12 = v1 * v2;

//         // Calcula las coordenadas baricéntricas
//         double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
//         double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
//         double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

//         // Verifica si el punto está dentro del triángulo
//         return (u >= 0) && (v >= 0) && (u + v < 1);
//     };

//     // Obtener los vértices de los triángulos
//     Mesh::Halfedge_index h0 = m.halfedge(f0);
//     Mesh::Halfedge_index h1 = m.halfedge(f1);
//     Mesh::Halfedge_index h2 = m.halfedge(f2);

//     vertex_descriptor vd0_0 = m.source(h0);
//     vertex_descriptor vd0_1 = m.source(m.next(h0));
//     vertex_descriptor vd0_2 = m.source(m.next(m.next(h0)));

//     vertex_descriptor vd1_0 = m.source(h1);
//     vertex_descriptor vd1_1 = m.source(m.next(h1));
//     vertex_descriptor vd1_2 = m.source(m.next(m.next(h1)));

//     vertex_descriptor vd2_0 = m.source(h2);
//     vertex_descriptor vd2_1 = m.source(m.next(h2));
//     vertex_descriptor vd2_2 = m.source(m.next(m.next(h2)));

//     K::Point_3 p0_0 = m.point(vd0_0);
//     K::Point_3 p0_1 = m.point(vd0_1);
//     K::Point_3 p0_2 = m.point(vd0_2);

//     K::Point_3 p1_0 = m.point(vd1_0);
//     K::Point_3 p1_1 = m.point(vd1_1);
//     K::Point_3 p1_2 = m.point(vd1_2);

//     K::Point_3 p2_0 = m.point(vd2_0);
//     K::Point_3 p2_1 = m.point(vd2_1);
//     K::Point_3 p2_2 = m.point(vd2_2);

//     // Clasificar los puntos en los triángulos correspondientes
//     for (vertex_descriptor vd : points) {
//         K::Point_3 point = m.point(vd);
//         if (isPointInTriangle(point, p0_0, p0_1, p0_2)) {
//             pointsInTriangle[f0].push_back(vd);
//         } else if (isPointInTriangle(point, p1_0, p1_1, p1_2)) {
//             pointsInTriangle[f1].push_back(vd);
//         } else if (isPointInTriangle(point, p2_0, p2_1, p2_2)) {
//             pointsInTriangle[f2].push_back(vd);
//         }
//     }
// }