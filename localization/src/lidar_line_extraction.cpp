#include "lidar_line_extraction.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <vector>

extern "C" int lidar_best_hypothesis(const double *, std::size_t,
                                    const std::int64_t *, std::size_t,
                                    double, double, std::uint8_t *);

namespace {
struct Point { double x, y; };
static_assert(sizeof(Point) == 2 * sizeof(double), "interleaved point ABI");

bool tls(const std::vector<Point> &points, Point &center, double *vt,
         lidar_svd_v1 svd, std::vector<double> &a, std::vector<double> &u)
{
    center = {0, 0};
    for (const auto &p : points) {
        center.x += p.x;
        center.y += p.y;
    }
    center.x /= points.size();
    center.y /= points.size();
    if (!std::isfinite(center.x) || !std::isfinite(center.y)) return false;
    const auto n = points.size();
    a.resize(2 * n);
    u.resize(2 * n);
    for (std::size_t i = 0; i < n; ++i) {
        a[i] = points[i].x - center.x;
        a[n + i] = points[i].y - center.y;
        if (!std::isfinite(a[i]) || !std::isfinite(a[n + i])) return false;
    }
    double singular[2];
    std::fill(vt, vt + 4, std::numeric_limits<double>::quiet_NaN());
    return svd(102, 'S', n, 2, a.data(), n, singular, u.data(), n, vt, 2) == 0;
}

bool finite_tls(const Point &center, const double *vt)
{
    return std::isfinite(center.x) && std::isfinite(center.y) &&
           std::isfinite(vt[0]) && std::isfinite(vt[1]) &&
           std::isfinite(vt[2]) && std::isfinite(vt[3]);
}

double quantile(const std::vector<double> &projection,
                const std::vector<std::int64_t> &order,
                std::size_t begin, std::size_t end, double q)
{
    const double rank = (end - begin - 1) * q;
    const auto lower = static_cast<std::size_t>(rank);
    const double fraction = rank - lower;
    const double a = projection[order[begin + lower]];
    const double b = projection[order[std::min(begin + lower + 1, end - 1)]];
    const double difference = b - a;
    return fraction < .5 ? a + difference * fraction
                         : b - difference * (1 - fraction);
}

int extract(const double *xy, std::size_t n, double threshold,
            std::size_t min_points, double min_span, lidar_pairs_v1 pairs,
            lidar_order_v1 sorted_order, lidar_svd_v1 svd,
            lidar_line_v1 *out, std::size_t capacity, std::size_t *count)
{
    std::vector<Point> remaining;
    remaining.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
        if (std::isfinite(xy[2 * i]) && std::isfinite(xy[2 * i + 1]))
            remaining.push_back({xy[2 * i], xy[2 * i + 1]});
    if (remaining.size() < min_points) return LIDAR_EXTRACT_OK;
    if (!pairs || !sorted_order || !svd)
        return LIDAR_EXTRACT_INVALID_ARGUMENT;

    std::vector<Point> fit, next;
    fit.reserve(n);
    next.reserve(n);
    std::vector<double> a, u, projection;
    std::vector<std::int64_t> order;
    std::vector<std::uint8_t> mask(n), visited;
    std::vector<lidar_line_v1> lines;
    // Every output support group consumes >= min_points distinct fit points.
    lines.reserve(std::min(capacity, n / min_points));
    for (int round = 0; round < 8; ++round) {
        n = remaining.size();
        if (n < min_points) break;
        // A failing or incomplete foreign callback must not leave uninitialised
        // indices that could be dereferenced inside this numerical kernel.
        std::int64_t hypotheses[320];
        std::fill(hypotheses, hypotheses + 320, -1);
        if (pairs(n, hypotheses) != 1) return LIDAR_EXTRACT_PAIRS_FAILED;
        for (const auto index : hypotheses)
            if (index < 0 || std::size_t(index) >= n)
                return LIDAR_EXTRACT_INVALID_CALLBACK_RESULT;
        if (lidar_best_hypothesis(
                reinterpret_cast<const double *>(remaining.data()), n,
                hypotheses, 160, threshold, .15, mask.data()) < 0)
            return LIDAR_EXTRACT_SCORE_FAILED;

        fit.clear();
        for (std::size_t i = 0; i < n; ++i)
            if (mask[i]) fit.push_back(remaining[i]);
        if (fit.size() < min_points) break;
        Point center;
        double vt[4];
        if (!tls(fit, center, vt, svd, a, u)) return LIDAR_EXTRACT_SVD_FAILED;
        if (!finite_tls(center, vt)) return LIDAR_EXTRACT_INVALID_CALLBACK_RESULT;
        const double nx = vt[1], ny = vt[3];
        fit.clear();
        next.clear();
        for (const auto &p : remaining) {
            if (std::abs((p.x - center.x) * nx + (p.y - center.y) * ny) < threshold)
                fit.push_back(p);
            else
                next.push_back(p);
        }
        remaining.swap(next);
        if (fit.size() < min_points) continue;
        if (!tls(fit, center, vt, svd, a, u)) return LIDAR_EXTRACT_SVD_FAILED;
        if (!finite_tls(center, vt)) return LIDAR_EXTRACT_INVALID_CALLBACK_RESULT;
        const double tx = vt[0], ty = vt[2];
        projection.resize(fit.size());
        order.assign(fit.size(), -1);
        for (std::size_t i = 0; i < fit.size(); ++i) {
            projection[i] = (fit[i].x - center.x) * tx + (fit[i].y - center.y) * ty;
            if (!std::isfinite(projection[i]))
                return LIDAR_EXTRACT_INVALID_CALLBACK_RESULT;
        }
        // Retain NumPy's exact default argsort, including equal-projection
        // ordering: std::sort selected different support points at boundaries.
        if (sorted_order(projection.data(), projection.size(), order.data()) != 1)
            return LIDAR_EXTRACT_ORDER_FAILED;
        visited.assign(order.size(), 0);
        for (std::size_t i = 0; i < order.size(); ++i) {
            const auto index = order[i];
            if (index < 0 || std::size_t(index) >= order.size() || visited[index])
                return LIDAR_EXTRACT_INVALID_CALLBACK_RESULT;
            visited[index] = 1;
            if (i && projection[index] < projection[order[i - 1]])
                return LIDAR_EXTRACT_INVALID_CALLBACK_RESULT;
        }

        for (std::size_t begin = 0, end = 1; end <= order.size(); ++end) {
            if (end < order.size() &&
                projection[order[end]] - projection[order[end - 1]] <= .15)
                continue;
            const auto size = end - begin;
            if (size >= min_points) {
                const double lo = quantile(projection, order, begin, end, .05);
                const double hi = quantile(projection, order, begin, end, .95);
                if (hi - lo >= min_span) {
                    if (lines.size() >= capacity) return LIDAR_EXTRACT_CAPACITY;
                    lidar_line_v1 line{};
                    line.count = std::min(std::size_t(32), size);
                    line.normal[0] = vt[1];
                    line.normal[1] = vt[3];
                    line.span = hi - lo;
                    const double step = double(size - 1) / double(line.count - 1);
                    for (std::size_t j = 0; j < std::size_t(line.count); ++j) {
                        const auto index = begin +
                            (j + 1 == std::size_t(line.count) ? size - 1
                             : static_cast<std::size_t>(j * step));
                        const auto p = fit[order[index]];
                        line.points[2 * j] = p.x;
                        line.points[2 * j + 1] = p.y;
                    }
                    lines.push_back(line);
                }
            }
            begin = end;
        }
    }
    if (!lines.empty())
        std::memcpy(out, lines.data(), lines.size() * sizeof(lidar_line_v1));
    *count = lines.size();
    return LIDAR_EXTRACT_OK;
}
} // namespace

extern "C" std::uint32_t lidar_extract_lines_abi_version(void) { return 1; }

extern "C" int lidar_extract_lines_v1(
    const double *xy, std::size_t n, double threshold, std::size_t min_points,
    double min_span, lidar_pairs_v1 pairs, lidar_order_v1 sorted_order,
    lidar_svd_v1 svd, lidar_line_v1 *out, std::size_t capacity, std::size_t *count)
{
    if (!count) return LIDAR_EXTRACT_INVALID_ARGUMENT;
    *count = 0;
    if (min_points < 2 || !std::isfinite(threshold) || threshold <= 0 ||
        !std::isfinite(min_span) || min_span < 0 || (!out && capacity) ||
        n > std::size_t(std::numeric_limits<std::int64_t>::max()) ||
        n > std::numeric_limits<std::size_t>::max() / (2 * sizeof(double)) ||
        capacity > std::numeric_limits<std::size_t>::max() / sizeof(lidar_line_v1) ||
        (n && !xy))
        return LIDAR_EXTRACT_INVALID_ARGUMENT;
    if (!n) return LIDAR_EXTRACT_OK;
    try {
        return extract(xy, n, threshold, min_points, min_span, pairs, sorted_order,
                       svd, out, capacity, count);
    } catch (const std::bad_alloc &) {
        return LIDAR_EXTRACT_ALLOCATION_FAILED;
    } catch (...) {
        return LIDAR_EXTRACT_EXCEPTION;
    }
}
