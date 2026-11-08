#include <cmath>
#include <cstddef>
#include <cstdint>

// Internal C ABI; no device, Python object, allocator or control ownership.
// Preserve first-maximum ties and the original distance arithmetic. Space is
// O(N) for the returned mask instead of O(N*H) distances and O(N*H*2) deltas.
extern "C" int lidar_best_hypothesis(const double *points, std::size_t n,
    const std::int64_t *pairs, std::size_t count, double threshold,
    double min_length, std::uint8_t *mask)
{
    if (!points || !pairs || !mask || n == 0 || count == 0) return -1;
    std::size_t best = 0, best_count = 0;
    double best_x = 0, best_y = 0, best_nx = 0, best_ny = 0;
    bool best_valid = false;
    for (std::size_t j = 0; j < count; ++j) {
        const auto a = pairs[2*j], b = pairs[2*j+1];
        if (a < 0 || b < 0 || std::size_t(a) >= n || std::size_t(b) >= n) return -1;
        const double x = points[2*a], y = points[2*a+1];
        const double dx = points[2*b]-x, dy = points[2*b+1]-y;
        const double length = std::sqrt(dx*dx+dy*dy);
        const bool valid = length >= min_length;
        const double denominator = length > 1e-12 ? length : 1e-12;
        const double nx = -dy/denominator, ny = dx/denominator;
        std::size_t hits = 0;
        if (valid) {
            for (std::size_t i = 0; i < n; ++i)
                hits += std::abs((points[2*i]-x)*nx+(points[2*i+1]-y)*ny) < threshold;
        }
        if (j == 0 || hits > best_count) {
            best = j; best_count = hits; best_valid = valid;
            best_x = x; best_y = y; best_nx = nx; best_ny = ny;
        }
    }
    for (std::size_t i = 0; i < n; ++i)
        mask[i] = best_valid && std::abs((points[2*i]-best_x)*best_nx+(points[2*i+1]-best_y)*best_ny) < threshold;
    return int(best);
}
