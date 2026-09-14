/* bench_law.c — đo thời gian MỘT lần gọi luật điều khiển, dùng CHÍNH mã nguồn
 * đang chạy trong node (src/hac/hac.c, src/fuzzy/fuzzy_type1.c), cộng hai biến
 * thể Fuzzy tương đương trong fuzzy_fast.c để trả lời "bản gốc có bị thiệt không".
 *
 * Mỗi luật nằm ở translation unit riêng và không bật LTO — giống hệt cách
 * hac_node / fuzzy_node được link — nên lời gọi không bị inline mất.
 *
 * Các luật được đo XEN KẼ từng vòng để trôi tần số CPU (governor powersave,
 * nhiệt) rơi đều lên tất cả. 'call' là hàm rỗng cùng chữ ký, đo chi phí của
 * chính khung đo (vòng lặp + lời gọi + ghi kết quả).
 *
 * Dùng qua bench_law.py. Ra stdout:
 *   name,median_ns,p10_ns,p90_ns,min_ns,mean_ns
 *   check,name,max_abs_diff_vs_original
 */
#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "hac.h"
#include "fuzzy_type1.h"

/* fuzzy_fast.c — cùng FIS, bỏ tính lặp, kết quả phải y hệt bản gốc */
float fuzzy_fast_pre(float e, float ed);
float fuzzy_fast_merge(float e, float ed);

#define NIN 4096

typedef float (*law2_t)(float, float);

static float in_e[NIN], in_ed[NIN];
static volatile float sink;

__attribute__((noinline)) static float empty_law(float e, float ed) {
  (void)e;
  (void)ed;
  return 0.0f;
}

static double now_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double)t.tv_sec * 1e9 + (double)t.tv_nsec;
}

static int cmp(const void *a, const void *b) {
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

/* Tham số giống rx150_hac_gains.yaml; đầu vào HAC đã bão hoà như trong node
 * (|e| <= error_limit shoulder = 0.333, |ed| <= 3.14). */
static double round_hac(int batch, unsigned *idx) {
  double t0 = now_ns();
  for (int k = 0; k < batch; ++k, ++*idx)
    sink = hac_eval(0.333f * in_e[*idx & (NIN - 1)], 3.14f * in_ed[*idx & (NIN - 1)], 0.3f, 12.0f, 1200.0f);
  return (now_ns() - t0) / batch;
}

/* Fuzzy nhận e, ed đã chuẩn hoá về [-1, 1] như trong node. */
static double round_law2(law2_t f, int batch, unsigned *idx) {
  double t0 = now_ns();
  for (int k = 0; k < batch; ++k, ++*idx) sink = f(in_e[*idx & (NIN - 1)], in_ed[*idx & (NIN - 1)]);
  return (now_ns() - t0) / batch;
}

static void report(const char *name, double *v, int n) {
  double mean = 0.0;
  for (int i = 0; i < n; ++i) mean += v[i];
  qsort(v, n, sizeof(double), cmp);
  printf("%s,%.4f,%.4f,%.4f,%.4f,%.4f\n", name, v[n / 2], v[n / 10], v[(9 * n) / 10], v[0], mean / n);
}

static void check(const char *name, law2_t f) {
  double worst = 0.0;
  for (int i = 0; i <= 400; ++i)
    for (int j = 0; j <= 400; ++j) {
      float e = -1.2f + 2.4f * (float)i / 400.0f, ed = -1.2f + 2.4f * (float)j / 400.0f;
      double d = fabs((double)f(e, ed) - (double)fuzzy_type1_eval(e, ed));
      if (d > worst) worst = d;
    }
  printf("check,%s,%.3e\n", name, worst);
}

int main(int argc, char **argv) {
  int rounds = argc > 1 ? atoi(argv[1]) : 300;
  int batch_fast = argc > 2 ? atoi(argv[2]) : 100000; /* hac, empty */
  int batch_slow = argc > 3 ? atoi(argv[3]) : 200;    /* fuzzy */

  unsigned s = 12345u;
  for (int i = 0; i < NIN; ++i) {
    s = s * 1103515245u + 12345u;
    in_e[i] = ((float)((s >> 8) & 0xFFFF) / 32767.5f) - 1.0f;
    s = s * 1103515245u + 12345u;
    in_ed[i] = ((float)((s >> 8) & 0xFFFF) / 32767.5f) - 1.0f;
  }

  const char *names[] = {"hac", "fuzzy", "fuzzy_pre", "fuzzy_merge", "call"};
  const law2_t fz[] = {NULL, fuzzy_type1_eval, fuzzy_fast_pre, fuzzy_fast_merge, empty_law};
  const int nlaw = 5;
  double *t[5];
  for (int l = 0; l < nlaw; ++l) t[l] = malloc(sizeof(double) * rounds);
  unsigned idx = 0;

  for (int r = -20; r < rounds; ++r) { /* 20 vòng đầu: hâm nóng, bỏ */
    for (int l = 0; l < nlaw; ++l) {
      double v = (l == 0)   ? round_hac(batch_fast, &idx)
                 : (l == 4) ? round_law2(fz[l], batch_fast, &idx)
                            : round_law2(fz[l], batch_slow, &idx);
      if (r >= 0) t[l][r] = v;
    }
  }
  for (int l = 0; l < nlaw; ++l) {
    report(names[l], t[l], rounds);
    free(t[l]);
  }
  check("fuzzy_pre", fuzzy_fast_pre);
  check("fuzzy_merge", fuzzy_fast_merge);
  return 0;
}
