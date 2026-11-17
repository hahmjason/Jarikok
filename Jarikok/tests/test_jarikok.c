/* 자리콕 판정 로직 단위 테스트 :  make test */
#include <stdio.h>
#include "../c/jarikok.h"

static int fails = 0, passes = 0;
#define CHECK(cond, msg) do { if (cond) passes++; else { fails++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } } while (0)

/* 사람(60kg) 착석 센서값: cop 는 앞(+)/뒤(-) 무게중심 */
static void person(int seat, float cop) {
    float w = 60.0f, front = w * (1 + cop) / 2, rear = w - front;
    jk_set_sensor(seat, front / 2, front / 2, rear / 2, rear / 2, 10.0f);
}
static void bag(int seat)   { jk_set_sensor(seat, 2.0f, 2.0f, 2.0f, 2.0f, 0.4f); }
static void empty(int seat) { jk_set_sensor(seat, 0, 0, 0, 0, 0.2f); }

/* 한 주기 실행: heads[] 는 YOLO 가 '요청받은 좌석' 에 대해서만 돌려줄 결과 */
static int cycle(const int *heads, const float *rise, int next, int eta, int at) {
    int i, n;
    jk_set_train(next, eta, at);
    n = jk_stage_sensing();
    for (i = 0; i < JK_SEATS; i++)
        if (jk_yolo_requested(i)) jk_set_head(i, heads ? heads[i] : 0, rise ? rise[i] : 0);
    jk_stage_processing();
    jk_stage_actuating();
    return n;
}

static void all_empty(void) { int i; for (i = 0; i < JK_SEATS; i++) empty(i); }

int main(void) {
    int heads[JK_SEATS] = {0};
    float rise[JK_SEATS] = {0};
    int i, rule;

    printf("자리콕 로직 테스트\n");

    /* ---------- 1. 구성 ---------- */
    {
        int b, sum = 0;
        for (b = 0; b < JK_BENCHES_PER_CAR; b++) sum += jk_bench_len(b);
        CHECK(sum == JK_SEATS_PER_CAR, "1량 39석");
        CHECK(JK_SEATS == 78 && JK_SEATS + JK_STANDING == JK_CAPACITY, "78석 + 입석 218 = 296");
        CHECK(jk_left_neighbor(0) == -1 && jk_right_neighbor(0) == 1, "벤치 시작 좌석 이웃");
        CHECK(jk_right_neighbor(6) == -1 && jk_left_neighbor(7) == -1, "벤치 경계에서는 옆자리 아님");
        CHECK(jk_right_neighbor(38) == -1 && jk_left_neighbor(39) == -1, "1호차/2호차 경계");
    }

    /* ---------- 2. 진리표 ---------- */
    CHECK(jk_truth_table(0, 0, 0, 0, &rule) == JK_EMPTY && rule == JK_R_P0, "P0C0 빈자리");
    CHECK(jk_truth_table(0, 1, 0, 0, &rule) == JK_EMPTY, "P0C1 (손 스침) 빈자리");
    CHECK(jk_truth_table(1, 1, 1, 0, &rule) == JK_OCCUPIED && rule == JK_R_P1_HEAD, "P1C1+머리 착석");
    CHECK(jk_truth_table(1, 0, 0, 0, &rule) == JK_BAG && rule == JK_R_P1C0_BAG, "P1C0 가방");
    CHECK(jk_truth_table(1, 0, 1, 0, &rule) == JK_OCCUPIED, "P1C0 이지만 머리 있음(두꺼운 외투) 착석");
    CHECK(jk_truth_table(1, 1, 0, 1, &rule) == JK_SPREAD && rule == JK_R_P1C1_SPREAD, "P1C1 머리 옆자리 -> 2칸 차지");
    CHECK(jk_truth_table(1, 1, 0, 0, &rule) == JK_OCCUPIED && rule == JK_R_P1C1_NOHEAD, "P1C1 머리 가림 -> 착석");

    /* ---------- 3. YOLO 게이팅 (압력 Active High 일 때만) ---------- */
    jk_init(); all_empty();
    person(0, 0); bag(1); jk_set_sensor(2, 0, 0, 0, 0, 12.0f); /* 2번: 정전용량만 */
    jk_set_train(1, 600, 0);
    CHECK(jk_stage_sensing() == 2, "YOLO 는 압력 True 인 2석만 실행");
    CHECK(jk_yolo_requested(0) && jk_yolo_requested(1) && !jk_yolo_requested(2), "게이팅 대상");
    jk_set_head(2, 1, 0);
    CHECK(jk_heads(2) == 0, "요청 안 한 좌석의 YOLO 결과는 무시");

    /* ---------- 4. 디바운스 ---------- */
    jk_init(); all_empty(); person(5, 0); heads[5] = 1;
    cycle(heads, rise, 1, 600, 0);
    CHECK(jk_state(5) == JK_EMPTY, "1주기만으로는 확정 안 됨");
    cycle(heads, rise, 1, 600, 0);
    CHECK(jk_state(5) == JK_EMPTY, "2주기도 확정 안 됨");
    cycle(heads, rise, 1, 600, 0);
    CHECK(jk_state(5) == JK_OCCUPIED, "3주기 연속 -> 착석 확정");
    empty(5); heads[5] = 0;
    cycle(heads, rise, 1, 600, 0);
    CHECK(jk_state(5) == JK_OCCUPIED, "순간 들썩임 1주기 -> 유지");
    person(5, 0); heads[5] = 1;
    cycle(heads, rise, 1, 600, 0);
    CHECK(jk_state(5) == JK_OCCUPIED, "노이즈 후 복귀");

    /* ---------- 5. 가방 / 2칸 차지 표시 ---------- */
    jk_init(); all_empty();
    for (i = 0; i < JK_SEATS; i++) heads[i] = 0;
    person(10, 0); heads[10] = 1;
    bag(11);
    person(12, 0);                       /* 12: 몸만 걸침 (머리는 13) */
    person(13, 0); heads[13] = 1;
    for (i = 0; i < 3; i++) cycle(heads, rise, 1, 600, 0);
    CHECK(jk_state(11) == JK_BAG && jk_display(11) == JK_DISP_BLOCKED, "옆자리 가방 -> 빨강");
    CHECK(jk_state(12) == JK_SPREAD && jk_display(12) == JK_DISP_BLOCKED, "2칸 차지 -> 빨강");
    CHECK(jk_display(0) == JK_DISP_EMPTY && jk_led_rgb(0) == 0x2563EB, "빈자리 파랑 LED");

    /* ---------- 6. 하차 확률 ---------- */
    {
        float p_dongdaegu = jk_alight_probability(0.45f, 1, 1, 0, 0);
        float p_small_cop = jk_alight_probability(0.08f, 1, 1, 0, 0);
        float p_small_both = jk_alight_probability(0.08f, 1, 1, 1, 0);
        float p_far = jk_alight_probability(0.45f, 0, 1, 1, 0);
        float p_still = jk_alight_probability(0.45f, 1, 0, 0, 0);
        CHECK(p_dongdaegu >= JK_LEAVE_P_TH, "동대구(45%) + CoP 전방 -> 곧 내림");
        CHECK(p_small_cop < JK_LEAVE_P_TH, "작은 역(8%) + CoP 만 -> 보류");
        CHECK(p_small_both >= JK_LEAVE_P_TH, "작은 역 + CoP + 머리상승 -> 곧 내림");
        CHECK(p_far > 0.44f && p_far < 0.46f, "역에서 멀면 증거 무시(사전확률)");
        CHECK(p_still < 0.45f, "역 근접 + 징후 없음 -> 확률 감소");
        CHECK(jk_alight_probability(0.1f, 1, 0, 0, 1) == 1.0f, "종착역 = 100%");
    }

    /* ---------- 7. CoP 하차 감지 + 폐루프 피드백 ---------- */
    jk_init(); all_empty();
    for (i = 0; i < JK_SEATS; i++) { heads[i] = 0; rise[i] = 0; }
    person(20, -0.1f); heads[20] = 1;   /* 내릴 사람 */
    person(21, -0.1f); heads[21] = 1;   /* 안 내릴 사람 */
    for (i = 0; i < 10; i++) cycle(heads, rise, 6, 300, 0);    /* 동대구 300초 전: 기준자세 학습 */
    CHECK(jk_state(20) == JK_OCCUPIED, "평상시 착석");
    person(20, 0.35f);                                           /* 엉덩이를 앞으로 */
    cycle(heads, rise, 6, 60, 0);
    CHECK(jk_state(20) == JK_LEAVING && jk_display(20) == JK_DISP_LEAVING, "CoP 전방 이동 -> 곧 내릴 자리");
    CHECK(jk_state(21) == JK_OCCUPIED, "자세 그대로인 사람은 착석 유지");
    CHECK(jk_expected_free_next() > 1.0f, "다음 역 빈자리 기대값 > 1");
    {
        float th0 = jk_cop_threshold();
        float r0 = jk_station_rate(6);
        cycle(heads, rise, 6, 0, 1);                             /* 동대구 도착 */
        empty(20); heads[20] = 0;                                /* 20번 하차 */
        cycle(heads, rise, 6, 0, 1);
        cycle(heads, rise, 7, 400, 0);                           /* 출발 */
        CHECK(jk_stat_hits() == 1 && jk_stat_false_alarms() == 0, "예측 적중 집계");
        CHECK(jk_cop_threshold() < th0, "적중 -> CoP 임계 소폭 완화");
        CHECK(jk_station_rate(6) != r0, "동대구 하차율 학습 갱신");
        CHECK(jk_state(21) == JK_OCCUPIED, "출발 후 남은 승객 착석");
    }
    /* 오경보: 앞으로 앉았지만 안 내림 */
    {
        float th1;
        for (i = 0; i < 6; i++) cycle(heads, rise, 7, 400, 0);
        person(21, 0.4f);
        th1 = jk_cop_threshold();
        jk_init();   /* 단순화를 위해 새 시나리오 */
        all_empty(); person(30, -0.1f); heads[30] = 1;
        for (i = 0; i < 10; i++) cycle(heads, rise, 5, 300, 0);
        person(30, 0.4f);
        cycle(heads, rise, 5, 30, 0);
        CHECK(jk_state(30) == JK_LEAVING, "대구 접근 + CoP -> 곧 내릴 자리");
        cycle(heads, rise, 5, 0, 1);
        cycle(heads, rise, 5, 0, 1);
        cycle(heads, rise, 6, 400, 0);    /* 안 내리고 출발 */
        CHECK(jk_stat_false_alarms() == 1, "오경보 집계");
        CHECK(jk_cop_threshold() > JK_COP_TH_INIT, "오경보 -> CoP 임계 강화");
        CHECK(jk_state(30) == JK_OCCUPIED, "출발 후 곧 내릴 자리 해제");
        (void)th1;
    }

    /* 종착역(경산)은 전원 하차라 CoP 임계 학습에서 제외 */
    {
        float th; int h0;
        jk_init(); all_empty();
        for (i = 0; i < JK_SEATS; i++) heads[i] = 0;
        person(50, 0); heads[50] = 1;
        for (i = 0; i < 10; i++) cycle(heads, rise, 7, 300, 0);
        cycle(heads, rise, 7, 30, 0);
        CHECK(jk_state(50) == JK_LEAVING, "종착역 접근 -> 전원 곧 내릴 자리");
        th = jk_cop_threshold(); h0 = jk_stat_hits();
        cycle(heads, rise, 7, 0, 1); empty(50); heads[50] = 0;
        cycle(heads, rise, 7, 0, 1);
        cycle(heads, rise, 7, 0, 0);
        CHECK(jk_cop_threshold() == th && jk_stat_hits() == h0, "종착역 결과는 θ 학습 제외");
    }

    /* ---------- 8. 정전용량 기준선 보정 ---------- */
    jk_init(); all_empty();
    for (i = 0; i < JK_SEATS; i++) heads[i] = 0;
    for (i = 0; i < 200; i++) { jk_set_sensor(40, 0, 0, 0, 0, 2.5f); cycle(heads, rise, 1, 600, 0); }
    CHECK(jk_cap_delta(40) < 0.5f, "빈 좌석 드리프트(2.5pF) 기준선 흡수");

    /* ---------- 9. HRI 경고 ---------- */
    jk_init();
    for (i = 0; i < JK_SEATS_PER_CAR; i++) { person(i, 0); heads[i] = 1; }
    for (i = JK_SEATS_PER_CAR; i < JK_SEATS; i++) { empty(i); heads[i] = 0; }
    bag(3); heads[3] = 0;
    for (i = 0; i < JK_HRI_CYCLES + 5; i++) cycle(heads, rise, 1, 600, 0);
    CHECK(jk_hri_alert(3) == 1, "만석 칸에서 가방 60초 이상 -> HRI 경고");
    CHECK(jk_recommend_car() == 1, "빈자리 많은 2호차 추천");

    printf("결과: %d 통과, %d 실패\n", passes, fails);
    return fails ? 1 : 0;
}
