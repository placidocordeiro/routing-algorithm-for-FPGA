#include "routing/geometry_utils.h"
#include <algorithm>

namespace routing {

Point compute_centroid(const std::vector<Point>& points) {
    if (points.empty()) return Point();

    float sum_x = 0, sum_y = 0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }

    return Point(sum_x / points.size(), sum_y / points.size());
}

Point find_steiner_point(
    const Point& source,
    const std::vector<Point>& sinks,
    float grid_resolution) {

    if (sinks.empty()) return source;

    // Criar lista de todos os pontos (source + sinks)
    std::vector<Point> all_points = sinks;
    all_points.push_back(source);

    // Bounding box
    BoundingBox bbox(all_points);

    // Usar centroide como ponto inicial
    Point best_steiner = compute_centroid(all_points);

    // Calcular custo inicial
    float best_cost = 0;
    for (const auto& sink : sinks) {
        best_cost += source.manhattan_distance(sink);
    }

    // Busca local em grid refinado
    // Ampliar bounding box um pouco
    float margin = std::max(bbox.width(), bbox.height()) * 0.2f;
    float search_x_min = bbox.min_x - margin;
    float search_x_max = bbox.max_x + margin;
    float search_y_min = bbox.min_y - margin;
    float search_y_max = bbox.max_y + margin;

    // Fazer busca em grid
    for (float x = search_x_min; x <= search_x_max; x += grid_resolution) {
        for (float y = search_y_min; y <= search_y_max; y += grid_resolution) {
            Point candidate(x, y);

            // Custo de conectar source -> steiner -> sinks
            float cost = source.manhattan_distance(candidate);
            for (const auto& sink : sinks) {
                cost += candidate.manhattan_distance(sink);
            }

            // Atualizar melhor ponto
            if (cost < best_cost) {
                best_cost = cost;
                best_steiner = candidate;
            }
        }
    }

    return best_steiner;
}

} // namespace routing
