#pragma once

#include <vector>
#include <utility>
#include <cmath>

namespace routing {

// Representação de ponto 2D
struct Point {
    float x;
    float y;

    Point() : x(0), y(0) {}
    Point(float x_, float y_) : x(x_), y(y_) {}

    float manhattan_distance(const Point& other) const {
        return std::abs(x - other.x) + std::abs(y - other.y);
    }

    float euclidean_distance(const Point& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// Bounding box para uma net
struct BoundingBox {
    float min_x, min_y;
    float max_x, max_y;

    BoundingBox() : min_x(0), min_y(0), max_x(0), max_y(0) {}

    BoundingBox(const std::vector<Point>& points) {
        if (points.empty()) return;

        min_x = max_x = points[0].x;
        min_y = max_y = points[0].y;

        for (const auto& p : points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }
    }

    float width() const { return max_x - min_x; }
    float height() const { return max_y - min_y; }
};

// Calcula centroide de um conjunto de pontos
Point compute_centroid(const std::vector<Point>& points);

// Encontra ponto de Steiner usando heurística de grid
// Retorna o ponto que minimiza a distância total para source + todos os sinks
Point find_steiner_point(
    const Point& source,
    const std::vector<Point>& sinks,
    float grid_resolution = 1.0f);

} // namespace routing
