#include <MeshFunctions/mesh_functions.hpp>

///////////////// auxilliary functions //////////////////////////////
bool isPointInTriangle(double x, double y, const Point& p0, const Point& p1, const Point& p2) {
    K::Point_2 pt(x, y);
    K::Triangle_2 tri(K::Point_2(p0.x(), p0.y()), K::Point_2(p1.x(), p1.y()), K::Point_2(p2.x(), p2.y()));
    return tri.has_on(pt);
}

Point calculateCentroid(const Point& p0, const Point& p1, const Point& p2) {
    return Point((p0.x() + p1.x() + p2.x()) / 3, (p0.y() + p1.y() + p2.y()) / 3, (p0.z() + p1.z() + p2.z()) / 3);
}

std::vector<Point> generateSearchPoints(const Point& centroid, const Point& p0, const Point& p1, const Point& p2, int steps) {
    std::vector<Point> points;

    auto addPointsInDirection = [&](const Point& start, const Point& direction) {
        double delta = 1.0/(double)steps;
        for (double i = 0.0; i < 1.0; i+=delta) {
            double newX = start.x() + i * (direction.x() - start.x());
            double newY = start.y() + i * (direction.y() - start.y());
            if (isPointInTriangle(newX, newY, p0, p1, p2)) {
                points.emplace_back(newX, newY, 0); // z-value will be determined by the height field H
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

//////////////////// end auxilliary funcitons //////////////////



// constructor para el function field
FstHAproxMesh::FstHAproxMesh(double left_x, double left_y, double right_x, double right_y, int steps, double MAX_ERROR_GOAL) {
    H = FunctionField_2<FT>(simple_terrain_func, left_x, left_y, right_x, right_y);
    this->left_x = left_x;
    this->left_y = left_y;
    this->right_x = right_x;
    this->right_y = right_y;
    this->steps = steps;
    this->MAX_ERROR_GOAL = MAX_ERROR_GOAL;
}


FstHAproxMesh::FstHAproxMesh(const std::string &path, FT max_scalar, RasterInterpolationType interp = RasterInterpolationType::BILINEAR, int steps, double MAX_ERROR_GOAL) {
    H = RasterFileField_2<FT>(path, max_scalar, interp);
    this->left_x = H.minX();
    this->left_y = H.minY();
    this->right_x = H.maxX();
    this->right_y = H.maxY();
    this->steps = steps;
    this->MAX_ERROR_GOAL = MAX_ERROR_GOAL;
}


// Función para interpolar la altura
double InterpolateHeight(double A, double B, double C, double D, Point p) {
    return -(A * p.x() + B * p.y() + D) / C;
}

// Función para escanear el triángulo y encontrar el punto con mayor error
void FstHAproxMesh::scanTriangle(Face_handle triangle) {
    Point best;
    double maxError = 0.0;

    Point p0 = triangle->vertex(0)->point();
    Point p1 = triangle->vertex(1)->point();
    Point p2 = triangle->vertex(2)->point();

    Point centroid = calculateCentroid(p0, p1, p2);
    std::vector<Point> searchPoints = generateSearchPoints(centroid, p0, p1, p2, steps);

    double A = (p1.y() - p0.y()) * (p2.z() - p0.z()) - (p1.z() - p0.z()) * (p2.y() - p0.y());
    double B = (p1.z() - p0.z()) * (p2.x() - p0.x()) - (p1.x() - p0.x()) * (p2.z() - p0.z());
    double C = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p1.y() - p0.y()) * (p2.x() - p0.x());
    double D = -(A * p0.x() + B * p0.y() + C * p0.z());


    // cambiar por uso de aproximacion dentro de triangulo
    for (const auto& pt : searchPoints) {
        double heightFieldZ = H(pt.x(), pt.y());
        double interpolatedHeight = -(A * pt.x() + B * pt.y() + D) / C;
        double error = std::abs(interpolatedHeight - heightFieldZ);
        if (error > maxError) {
            maxError = error;
            best = pt;
        }
    }
    

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