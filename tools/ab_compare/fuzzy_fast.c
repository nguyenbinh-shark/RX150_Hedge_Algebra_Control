/* fuzzy_fast.c — CHỈ DÙNG CHO bench_law: hai cách tính CÙNG một FIS với bản
 * fuzzy_type1.c do fis2c.py sinh, bỏ phần tính lặp, KẾT QUẢ PHẢI Y HỆT.
 * bench_law.c kiểm sai khác trên lưới dày và in ra; khác 0 là lỗi ở file này.
 *
 * Không chép bảng luật / hàm thuộc: #include thẳng file đã sinh để dùng đúng
 * in_mf, out_mf, rules, mf_eval của nó (sinh lại .fis thì file này tự theo).
 *
 *   fuzzy_fast_pre    bảng MF đầu ra (7 × 201) và toạ độ y tính MỘT lần — bản gốc
 *                     dựng lại chúng ở mỗi lần gọi dù chúng là hằng số.
 *   fuzzy_fast_merge  thêm: gộp các luật có cùng hệ quả trước khi quét 201 điểm.
 *                     max_r min(f_r, mu_m(y)) = min(max_r f_r, mu_m(y)) với mọi
 *                     luật r cùng hệ quả m — đúng tuyệt đối với min/max, nên vòng
 *                     trong còn 7 thay vì 25, bỏ qua hệ quả có độ kích hoạt 0.
 */
#define fuzzy_type1_eval_core fz_orig_core_unused_
#define fuzzy_type1_eval fz_orig_eval_unused_
#include "fuzzy_type1.c"
#undef fuzzy_type1_eval_core
#undef fuzzy_type1_eval

float fuzzy_fast_pre(float e, float ed);
float fuzzy_fast_merge(float e, float ed);

static float Y[FUZZY_N];
static float MU_OUT[FUZZY_MAXMO][FUZZY_N];
static int tables_ready = 0;

static void init_tables(void) {
  for (int k = 0; k < FUZZY_N; k++) {
    /* đúng biểu thức của bản gốc để giá trị trùng từng bit */
    Y[k] = out_lo[0] + (out_hi[0] - out_lo[0]) * (FUZZY_N == 1 ? 0.0f : (float) k / (float) (FUZZY_N - 1));
    for (int m = 0; m < out_nmf[0]; m++) MU_OUT[m][k] = mf_eval(out_mf[0][m].type, out_mf[0][m].p, Y[k]);
  }
  tables_ready = 1;
}

static void firing_of(float e, float ed, float firing[FUZZY_NR]) {
  const float in[FUZZY_NI] = {e, ed};
  float mu_in[FUZZY_NI][FUZZY_MAXMI];
  for (int i = 0; i < FUZZY_NI; i++)
    for (int m = 0; m < in_nmf[i]; m++) mu_in[i][m] = mf_eval(in_mf[i][m].type, in_mf[i][m].p, in[i]);
  for (int r = 0; r < FUZZY_NR; r++) {
    float f = 1.0f;
    int seen = 0;
    for (int ii = 0; ii < FUZZY_NI; ii++) {
      int idx = rules[r].in[ii];
      if (idx != 0) {
        float v = mu_in[ii][idx - 1];
        f = seen ? ((rules[r].conn == 2) ? s_norm(f, v) : t_norm(f, v)) : v;
        seen = 1;
      }
    }
    firing[r] = (seen ? f : 1.0f) * rules[r].weight;
  }
}

static float centroid(const float agg[FUZZY_N]) {
  float num = 0.0f, den = 0.0f;
  for (int k = 0; k < FUZZY_N; k++) {
    num += Y[k] * agg[k];
    den += agg[k];
  }
  return (den > 1e-9f) ? num / den : 0.0f;
}

float fuzzy_fast_pre(float e, float ed) {
  if (!tables_ready) init_tables();
  float firing[FUZZY_NR], agg[FUZZY_N];
  firing_of(e, ed, firing);
  for (int k = 0; k < FUZZY_N; k++) {
    float a = 0.0f;
    for (int r = 0; r < FUZZY_NR; r++) {
      int oi = rules[r].out[0];
      if (oi == 0) continue;
      a = s_norm(a, imp(firing[r], MU_OUT[oi - 1][k]));
    }
    agg[k] = a;
  }
  return centroid(agg);
}

float fuzzy_fast_merge(float e, float ed) {
  if (!tables_ready) init_tables();
  float firing[FUZZY_NR], strength[FUZZY_MAXMO] = {0}, agg[FUZZY_N];
  firing_of(e, ed, firing);
  for (int r = 0; r < FUZZY_NR; r++) {
    int oi = rules[r].out[0];
    if (oi != 0) strength[oi - 1] = s_norm(strength[oi - 1], firing[r]);
  }
  int active[FUZZY_MAXMO], na = 0;
  for (int m = 0; m < out_nmf[0]; m++)
    if (strength[m] > 0.0f) active[na++] = m;
  for (int k = 0; k < FUZZY_N; k++) {
    float a = 0.0f;
    for (int j = 0; j < na; j++) a = s_norm(a, imp(strength[active[j]], MU_OUT[active[j]][k]));
    agg[k] = a;
  }
  return centroid(agg);
}
