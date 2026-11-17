/*
 * 자리콕 웹(Web) - 핵심 판정 로직 구현
 * 구조: Sensing -> Processing -> Actuating -> (Feedback) -> Sensing/Processing  (Closed-Loop)
 */
#include "jarikok.h"

/* ============================================================
 * 0. freestanding 수학 함수 (libc 없이 동작)
 * ============================================================ */
static float jk_absf(float x) { return x < 0 ? -x : x; }
static float jk_clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static float jk_expf(float x)
{
    /* e^x = 2^k * e^r,  |r| <= ln2/2 ; e^r 는 테일러 급수 */
    const float LN2 = 0.69314718f;
    int k;
    float r, t, s;
    int i;
    if (x > 80.0f) x = 80.0f;
    if (x < -80.0f) x = -80.0f;
    k = (int)(x / LN2 + (x >= 0 ? 0.5f : -0.5f));
    r = x - (float)k * LN2;
    s = 1.0f; t = 1.0f;
    for (i = 1; i < 10; i++) { t *= r / (float)i; s += t; }
    while (k > 0) { s *= 2.0f; k--; }
    while (k < 0) { s *= 0.5f; k++; }
    return s;
}

static float jk_lnf(float x)
{
    /* ln x = k ln2 + 2 atanh((m-1)/(m+1)),  m in [0.75, 1.5) */
    const float LN2 = 0.69314718f;
    int k = 0, i;
    float y, y2, s, t;
    if (x <= 0.0f) return -80.0f;
    while (x >= 1.5f) { x *= 0.5f; k++; }
    while (x < 0.75f) { x *= 2.0f; k--; }
    y = (x - 1.0f) / (x + 1.0f);
    y2 = y * y; s = 0.0f; t = y;
    for (i = 1; i < 20; i += 2) { s += t / (float)i; t *= y2; }
    return (float)k * LN2 + 2.0f * s;
}

float jk_sigmoid(float x) { return 1.0f / (1.0f + jk_expf(-x)); }

float jk_logit(float p)
{
    p = jk_clampf(p, 0.001f, 0.999f);
    return jk_lnf(p / (1.0f - p));
}

#ifdef __wasm__
/* clang 이 구조체 초기화에 memset 을 쓸 수 있으므로 직접 제공 */
void *memset(void *d, int c, unsigned long n)
{
    unsigned char *p = (unsigned char *)d;
    while (n--) *p++ = (unsigned char)c;
    return d;
}
#endif

/* ============================================================
 * 1. 데이터
 * ============================================================ */
/* 1량 벤치 구성: 한쪽 7+7+6, 반대쪽 7+7+5 = 39석 */
static const int BENCH_LEN[JK_BENCHES_PER_CAR] = { 7, 7, 6, 7, 7, 5 };

/* 대경선 역별 초기 하차율(사전확률). 0:구미 1:사곡 2:북삼 3:왜관 4:서대구 5:대구 6:동대구 7:경산(종착)
 * 운행 데이터가 쌓이면 폐루프 학습으로 갱신된다. */
static const float RATE_INIT[JK_STATIONS] = {
    0.00f, 0.08f, 0.08f, 0.15f, 0.20f, 0.30f, 0.45f, 1.00f
};

typedef struct {
    /* Sensing 원시값 */
    float fsr[4];        /* 앞왼, 앞오, 뒤왼, 뒤오 (kg) */
    float cap;           /* 정전용량 측정값 (pF)         */
    float cap_base;      /* 빈 좌석 기준선 (pF) - 폐루프 보정 */
    int   heads;         /* ROI 내 머리 수 (YOLO)       */
    float head_rise;     /* 머리 높이 상승량             */
    int   yolo_req;      /* 이번 주기 YOLO 실행 요청 여부 */

    /* Sensing 가공값 */
    float total;
    float cop_y;         /* 앞(+1) ~ 뒤(-1) 무게중심 */
    int   p, c;          /* 이진화 결과 (Active High) */

    /* Processing */
    int   raw;           /* 진리표 판정 결과(디바운스 전) */
    int   rule;
    int   cand, cand_n;  /* 디바운스 후보 상태와 연속 횟수 */
    int   state;         /* 확정 상태 */
    float cop_base;      /* 개인별 기준 자세 */
    int   cop_base_n;
    float cop_shift;
    float prob;          /* 이번 역 하차 확률 */
    int   leave_latch;

    /* Actuating / Feedback */
    int   blocked_cycles;
    int   hri;
    int   was_person_at_arrival;
    int   was_leaving_at_arrival;
    int   emptied_during_stop;
} seat_t;

static seat_t S[JK_SEATS];
static int   g_next_station, g_eta, g_at_station, g_prev_at_station;
static int   g_arrival_station;
static float g_cop_th;
static float g_rate[JK_STATIONS];
static int   g_hits, g_fa, g_miss;
static int   g_yolo_runs;

/* ============================================================
 * 2. 좌석 배치 (같은 벤치 안에서만 '옆자리' 가 성립)
 * ============================================================ */
int jk_bench_len(int b) { return (b >= 0 && b < JK_BENCHES_PER_CAR) ? BENCH_LEN[b] : 0; }

static void seat_pos(int seat, int *bench, int *idx)
{
    int local = seat % JK_SEATS_PER_CAR, b = 0;
    while (b < JK_BENCHES_PER_CAR && local >= BENCH_LEN[b]) { local -= BENCH_LEN[b]; b++; }
    *bench = b; *idx = local;
}

int jk_left_neighbor(int seat)
{
    int b, i;
    if (seat < 0 || seat >= JK_SEATS) return -1;
    seat_pos(seat, &b, &i);
    return i > 0 ? seat - 1 : -1;
}

int jk_right_neighbor(int seat)
{
    int b, i;
    if (seat < 0 || seat >= JK_SEATS) return -1;
    seat_pos(seat, &b, &i);
    return i < BENCH_LEN[b] - 1 ? seat + 1 : -1;
}

static int valid(int s) { return s >= 0 && s < JK_SEATS; }
static int is_person(int st) { return st == JK_OCCUPIED || st == JK_LEAVING; }

/* ============================================================
 * 3. 초기화
 * ============================================================ */
void jk_init(void)
{
    int i;
    for (i = 0; i < JK_SEATS; i++) {
        seat_t *s = &S[i];
        s->fsr[0] = s->fsr[1] = s->fsr[2] = s->fsr[3] = 0;
        s->cap = 0; s->cap_base = 0;
        s->heads = 0; s->head_rise = 0; s->yolo_req = 0;
        s->total = 0; s->cop_y = 0; s->p = 0; s->c = 0;
        s->raw = JK_EMPTY; s->rule = JK_R_P0;
        s->cand = JK_EMPTY; s->cand_n = 0; s->state = JK_EMPTY;
        s->cop_base = 0; s->cop_base_n = 0; s->cop_shift = 0;
        s->prob = 0; s->leave_latch = 0;
        s->blocked_cycles = 0; s->hri = 0;
        s->was_person_at_arrival = 0; s->was_leaving_at_arrival = 0; s->emptied_during_stop = 0;
    }
    for (i = 0; i < JK_STATIONS; i++) g_rate[i] = RATE_INIT[i];
    g_next_station = 1; g_eta = 600; g_at_station = 0; g_prev_at_station = 0;
    g_arrival_station = -1;
    g_cop_th = JK_COP_TH_INIT;
    g_hits = g_fa = g_miss = 0;
    g_yolo_runs = 0;
}

/* ============================================================
 * 4. SENSING
 * ============================================================ */
void jk_set_sensor(int seat, float fl, float fr, float rl, float rr, float cap_pf)
{
    if (!valid(seat)) return;
    S[seat].fsr[0] = fl; S[seat].fsr[1] = fr; S[seat].fsr[2] = rl; S[seat].fsr[3] = rr;
    S[seat].cap = cap_pf;
}

void jk_set_train(int next_station, int eta_s, int at_station)
{
    g_next_station = next_station < 0 ? 0 : (next_station >= JK_STATIONS ? JK_STATIONS - 1 : next_station);
    g_eta = eta_s < 0 ? 0 : eta_s;
    g_at_station = at_station ? 1 : 0;
}

/* 압력/정전용량 이진화 + CoP 계산 + YOLO 게이팅.
 * 압력 센서는 정논리(Active High): 하중이 있을 때만 True.
 * True 인 좌석만 CCTV 좌석 박스(ROI)에 YOLO 머리 추적을 돌린다 -> 연산량 절감. */
int jk_stage_sensing(void)
{
    int i, n = 0;
    for (i = 0; i < JK_SEATS; i++) {
        seat_t *s = &S[i];
        float front = s->fsr[0] + s->fsr[1];
        float rear  = s->fsr[2] + s->fsr[3];
        s->total = front + rear;
        s->p = (s->total >= JK_P_TH_KG) ? 1 : 0;
        s->c = ((s->cap - s->cap_base) >= JK_C_TH_PF) ? 1 : 0;
        s->cop_y = (s->total > 0.5f) ? (front - rear) / s->total : 0.0f;
        s->yolo_req = s->p;
        s->heads = 0; s->head_rise = 0;    /* 요청되지 않은 좌석은 머리 정보 없음 */
        n += s->yolo_req;
    }
    g_yolo_runs = n;
    return n;
}

int jk_yolo_requested(int seat) { return valid(seat) ? S[seat].yolo_req : 0; }

void jk_set_head(int seat, int heads_in_roi, float head_rise)
{
    if (!valid(seat) || !S[seat].yolo_req) return;   /* 게이팅: 요청한 좌석만 받는다 */
    S[seat].heads = heads_in_roi < 0 ? 0 : heads_in_roi;
    S[seat].head_rise = head_rise;
}

/* ============================================================
 * 5. PROCESSING
 * ============================================================ */

/* 진리표 (압력 P, 정전용량 C, ROI 머리 수, 이웃 좌석 머리 여부)
 *  P C | 머리 | 이웃머리 | 판정
 *  0 * |  -   |    -     | 빈자리      (압력 없으면 착석 아님, 정전용량만 True = 손·옷 스침 노이즈)
 *  1 * |  >=1 |    -     | 착석        (P1C0 이라도 머리가 있으면 두꺼운 외투 등으로 C 약화된 사람)
 *  1 1 |  0   |   있음   | 2칸 차지    (몸은 이 좌석에, 머리는 옆 좌석 -> 한 사람이 두 칸)
 *  1 1 |  0   |   없음   | 착석        (머리 가림 - 두 센서가 모두 사람이라고 하면 사람)
 *  1 0 |  0   |    -     | 가방        (무게는 있으나 인체 유전율 없음 -> 짐)
 */
int jk_truth_table(int p, int c, int heads, int neighbor_head, int *rule_out)
{
    int st, rule;
    if (!p)              { st = JK_EMPTY;    rule = JK_R_P0; }
    else if (heads > 0)  { st = JK_OCCUPIED; rule = JK_R_P1_HEAD; }
    else if (c) {
        if (neighbor_head) { st = JK_SPREAD;   rule = JK_R_P1C1_SPREAD; }
        else               { st = JK_OCCUPIED; rule = JK_R_P1C1_NOHEAD; }
    } else               { st = JK_BAG;      rule = JK_R_P1C0_BAG; }
    if (rule_out) *rule_out = rule;
    return st;
}

/* 역 간 하차 확률: 로그 오즈 = logit(역별 하차율) + 증거 가중치
 *   역 근접(도착 90초 전) 구간에서만 CoP 전방 이동 / 머리 상승을 하차 증거로 해석한다. */
float jk_alight_probability(float prior, int near, int cop_forward, int head_rise, int terminus)
{
    float lo;
    if (terminus) return 1.0f;
    if (!near) return jk_clampf(prior, 0.0f, 1.0f);
    lo = jk_logit(prior);
    if (cop_forward) lo += JK_W_COP;
    if (head_rise)   lo += JK_W_HEAD;
    if (!cop_forward && !head_rise) lo += JK_W_STILL;
    return jk_sigmoid(lo);
}

static int neighbor_has_head(int i)
{
    int l = jk_left_neighbor(i), r = jk_right_neighbor(i);
    return (valid(l) && S[l].heads > 0) || (valid(r) && S[r].heads > 0);
}

void jk_stage_processing(void)
{
    int i;
    int near = (g_at_station || g_eta <= JK_NEAR_ETA_S);
    int terminus = (g_next_station == JK_STATIONS - 1);

    for (i = 0; i < JK_SEATS; i++) {
        seat_t *s = &S[i];
        int prev = s->state;

        /* (1) 진리표 */
        s->raw = jk_truth_table(s->p, s->c, s->heads, neighbor_has_head(i), &s->rule);

        /* (2) 디바운스: 같은 판정이 JK_DEBOUNCE 주기 연속일 때만 확정 (흔들림·순간 노이즈 제거) */
        {
            int raw_cls = is_person(s->raw) ? JK_OCCUPIED : s->raw;
            int cur_cls = is_person(s->state) ? JK_OCCUPIED : s->state;
            if (raw_cls == cur_cls) {
                s->cand = cur_cls; s->cand_n = 0;
            } else {
                if (s->cand == raw_cls) s->cand_n++;
                else { s->cand = raw_cls; s->cand_n = 1; }
                if (s->cand_n >= JK_DEBOUNCE) { s->state = raw_cls; s->cand_n = 0; }
            }
        }

        /* (3) 새로 착석 -> 개인별 CoP 기준자세 학습 시작 */
        if (is_person(s->state) && !is_person(prev)) {
            s->cop_base = 0; s->cop_base_n = 0; s->leave_latch = 0;
        }
        if (!is_person(s->state)) { s->leave_latch = 0; s->prob = 0; s->cop_shift = 0; continue; }

        /* (4) CoP 기준자세: 역 근접 구간이 아닐 때만 학습(하차 준비 자세가 기준에 섞이지 않게) */
        if (!near && s->p) {
            if (s->cop_base_n < JK_COP_BASE_CYCLES) {
                s->cop_base = (s->cop_base * s->cop_base_n + s->cop_y) / (float)(s->cop_base_n + 1);
                s->cop_base_n++;
            } else {
                s->cop_base += 0.02f * (s->cop_y - s->cop_base);
            }
        }
        s->cop_shift = (s->cop_base_n > 0 && s->p) ? (s->cop_y - s->cop_base) : 0.0f;

        /* (5) 역 간 하차 확률 */
        {
            int cop_fwd = s->cop_shift >= g_cop_th;
            int rise = s->head_rise >= JK_HEAD_RISE_TH;
            s->prob = jk_alight_probability(g_rate[g_next_station], near, cop_fwd, rise, terminus && near);
        }

        /* (6) 곧 내릴 자리: 역 근접 + 확률 >= 70%. 한 번 판정되면 정차 종료(출발)까지 유지 */
        if (near && s->prob >= JK_LEAVE_P_TH) s->leave_latch = 1;
        s->state = s->leave_latch ? JK_LEAVING : JK_OCCUPIED;
    }
}

/* ============================================================
 * 6. ACTUATING + FEEDBACK (Closed-Loop)
 * ============================================================ */
static int display_of(int st)
{
    switch (st) {
    case JK_EMPTY:   return JK_DISP_EMPTY;
    case JK_LEAVING: return JK_DISP_LEAVING;
    case JK_BAG:
    case JK_SPREAD:  return JK_DISP_BLOCKED;
    default:         return JK_DISP_OCCUPIED;
    }
}

int jk_count_display_car(int car, int disp)
{
    int i, n = 0;
    for (i = car * JK_SEATS_PER_CAR; i < (car + 1) * JK_SEATS_PER_CAR; i++)
        if (display_of(S[i].state) == disp) n++;
    return n;
}

int jk_count_display(int disp)
{
    int car, n = 0;
    for (car = 0; car < JK_CARS; car++) n += jk_count_display_car(car, disp);
    return n;
}

void jk_stage_actuating(void)
{
    int i, car;
    int arrived  = (g_at_station && !g_prev_at_station);
    int departed = (!g_at_station && g_prev_at_station);

    /* (F1) 정차 시작: 실제 하차 여부를 확인하기 위해 현재 예측을 기록 */
    if (arrived) {
        g_arrival_station = g_next_station;
        for (i = 0; i < JK_SEATS; i++) {
            S[i].was_person_at_arrival  = is_person(S[i].state);
            S[i].was_leaving_at_arrival = (S[i].state == JK_LEAVING);
            S[i].emptied_during_stop = 0;
        }
    }
    if (g_at_station) {
        for (i = 0; i < JK_SEATS; i++)
            if (S[i].was_person_at_arrival && !S[i].p) S[i].emptied_during_stop = 1;
    }

    /* (F2) 출발: 예측 vs 실제 비교 -> CoP 임계값 적응 + 역별 하차율 학습 */
    /* 종착역은 전원 하차(p=1)이므로 CoP 판단 검증에서 제외 */
    if (departed && g_arrival_station >= 0 && g_arrival_station != JK_STATIONS - 1) {
        int persons = 0, left = 0;
        for (i = 0; i < JK_SEATS; i++) {
            seat_t *s = &S[i];
            if (!s->was_person_at_arrival) continue;
            persons++;
            if (s->emptied_during_stop) left++;
            if (s->was_leaving_at_arrival && s->emptied_during_stop)       { g_hits++; g_cop_th -= 0.002f; }
            else if (s->was_leaving_at_arrival && !s->emptied_during_stop) { g_fa++;   g_cop_th += 0.02f;  }
            else if (!s->was_leaving_at_arrival && s->emptied_during_stop) { g_miss++; g_cop_th -= 0.004f; }
            s->was_person_at_arrival = s->was_leaving_at_arrival = s->emptied_during_stop = 0;
        }
        g_cop_th = jk_clampf(g_cop_th, JK_COP_TH_MIN, JK_COP_TH_MAX);
        if (persons > 0) {
            float obs = (float)left / (float)persons;
            g_rate[g_arrival_station] = jk_clampf(0.9f * g_rate[g_arrival_station] + 0.1f * obs, 0.01f, 0.99f);
        }
    }
    if (departed) {
        /* 출발했으니 남아 있는 사람의 '곧 내릴 자리' 표시는 해제 */
        for (i = 0; i < JK_SEATS; i++) {
            if (S[i].state == JK_LEAVING) { S[i].state = JK_OCCUPIED; }
            S[i].leave_latch = 0;
            S[i].was_person_at_arrival = S[i].was_leaving_at_arrival = S[i].emptied_during_stop = 0;
        }
        g_arrival_station = -1;
    }

    /* (F3) 빈 좌석 정전용량 기준선 보정 (온도·습도 드리프트 제거) */
    for (i = 0; i < JK_SEATS; i++) {
        seat_t *s = &S[i];
        if (s->state == JK_EMPTY && !s->p && s->raw == JK_EMPTY)
            s->cap_base += 0.05f * (s->cap - s->cap_base);
    }

    /* (A) HRI: 가방/2칸 차지가 오래 지속 + 해당 칸이 붐비면(빈자리 2석 이하) 경고 */
    for (car = 0; car < JK_CARS; car++) {
        int crowded = jk_count_display_car(car, JK_DISP_EMPTY) <= 2;
        for (i = car * JK_SEATS_PER_CAR; i < (car + 1) * JK_SEATS_PER_CAR; i++) {
            seat_t *s = &S[i];
            if (s->state == JK_BAG || s->state == JK_SPREAD) s->blocked_cycles++;
            else s->blocked_cycles = 0;
            s->hri = (s->blocked_cycles >= JK_HRI_CYCLES && crowded) ? 1 : 0;
        }
    }

    g_prev_at_station = g_at_station;
}

/* ============================================================
 * 7. 조회
 * ============================================================ */
int   jk_state(int i)      { return valid(i) ? S[i].state : -1; }
int   jk_display(int i)    { return valid(i) ? display_of(S[i].state) : -1; }
int   jk_rule(int i)       { return valid(i) ? S[i].rule : -1; }
int   jk_hri_alert(int i)  { return valid(i) ? S[i].hri : 0; }
float jk_leave_prob(int i) { return valid(i) ? S[i].prob : 0; }
float jk_cop_y(int i)      { return valid(i) ? S[i].cop_y : 0; }
float jk_cop_shift(int i)  { return valid(i) ? S[i].cop_shift : 0; }
float jk_total_kg(int i)   { return valid(i) ? S[i].total : 0; }
float jk_cap_delta(int i)  { return valid(i) ? S[i].cap - S[i].cap_base : 0; }
int   jk_bit_p(int i)      { return valid(i) ? S[i].p : 0; }
int   jk_bit_c(int i)      { return valid(i) ? S[i].c : 0; }
int   jk_heads(int i)      { return valid(i) ? S[i].heads : 0; }
int   jk_yolo_runs(void)   { return g_yolo_runs; }
float jk_cop_threshold(void) { return g_cop_th; }
float jk_station_rate(int st) { return (st >= 0 && st < JK_STATIONS) ? g_rate[st] : 0; }
int   jk_stat_hits(void)   { return g_hits; }
int   jk_stat_false_alarms(void) { return g_fa; }
int   jk_stat_misses(void) { return g_miss; }

int jk_led_rgb(int i)
{
    switch (jk_display(i)) {
    case JK_DISP_EMPTY:   return 0x2563EB; /* 파랑 */
    case JK_DISP_LEAVING: return 0xF59E0B; /* 주황 */
    case JK_DISP_BLOCKED: return 0xEF4444; /* 빨강 (HRI 시 점멸) */
    default:              return 0x000000; /* 착석: 꺼짐 */
    }
}

/* 다음 역에서 생길 빈자리 기대값 = 착석 좌석 하차확률의 합 */
float jk_expected_free_next(void)
{
    int i; float e = 0;
    int terminus = (g_next_station == JK_STATIONS - 1);
    for (i = 0; i < JK_SEATS; i++) {
        if (!is_person(S[i].state)) continue;
        e += (S[i].state == JK_LEAVING) ? 1.0f
             : (terminus ? 1.0f : (S[i].prob > 0 ? S[i].prob : g_rate[g_next_station]));
    }
    return e;
}

/* 추천 칸 = 빈자리 + 다음 역 하차 기대값이 큰 칸 */
int jk_recommend_car(void)
{
    int car, i, best = 0; float bestv = -1;
    for (car = 0; car < JK_CARS; car++) {
        float v = 0;
        for (i = car * JK_SEATS_PER_CAR; i < (car + 1) * JK_SEATS_PER_CAR; i++) {
            if (S[i].state == JK_EMPTY) v += 1.0f;
            else if (S[i].state == JK_LEAVING) v += 1.0f;
            else if (S[i].state == JK_OCCUPIED) v += S[i].prob;
        }
        if (v > bestv + 1e-4f) { bestv = v; best = car; }
    }
    (void)jk_absf;
    return best;
}
