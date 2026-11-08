#ifndef LIDAR_LINE_EXTRACTION_H
#define LIDAR_LINE_EXTRACTION_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Internal numerical ABI. Callbacks run synchronously on the calling thread.
 * Pair callback writes 160 ordered index pairs; order callback writes a full
 * permutation. Their success value is 1, failure is 0. The SVD function must
 * be the caller's ILP64 LAPACKE_dgesdd implementation (column-major layout).
 * No device, clock, localisation policy or control ownership is held here.
 */
typedef int (*lidar_pairs_v1)(size_t point_count, int64_t *pairs);
typedef int (*lidar_order_v1)(const double *projection, size_t point_count,
                            int64_t *order);
typedef int64_t (*lidar_svd_v1)(int layout, char jobz, int64_t m, int64_t n,
                              double *a, int64_t lda, double *s, double *u,
                              int64_t ldu, double *vt, int64_t ldvt);

typedef struct lidar_line_v1 {
    int64_t count;
    double normal[2];
    double span;
    double points[64];
} lidar_line_v1;

enum lidar_extract_result_v1 {
    LIDAR_EXTRACT_OK = 0,
    LIDAR_EXTRACT_INVALID_ARGUMENT = -1,
    LIDAR_EXTRACT_PAIRS_FAILED = -2,
    LIDAR_EXTRACT_SCORE_FAILED = -3,
    LIDAR_EXTRACT_SVD_FAILED = -4,
    LIDAR_EXTRACT_CAPACITY = -5,
    LIDAR_EXTRACT_ORDER_FAILED = -6,
    LIDAR_EXTRACT_ALLOCATION_FAILED = -7,
    LIDAR_EXTRACT_INVALID_CALLBACK_RESULT = -8,
    LIDAR_EXTRACT_EXCEPTION = -9
};

uint32_t lidar_extract_lines_abi_version(void);

/* On success, count lines are copied to out. On every returned error, *count
 * is zero and out is unchanged; partial lines are never published. count is
 * mandatory. Empty input succeeds without xy, callbacks or an output buffer.
 * A zero-capacity output may be NULL and succeeds if no lines are found.
 */
int lidar_extract_lines_v1(const double *xy, size_t n, double threshold,
                          size_t min_points, double min_span,
                          lidar_pairs_v1 pairs, lidar_order_v1 sorted_order,
                          lidar_svd_v1 svd, lidar_line_v1 *out,
                          size_t capacity, size_t *count);

#ifdef __cplusplus
}
#endif
#endif
