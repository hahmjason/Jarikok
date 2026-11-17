/*
 * 자리콕 웹(Web) - 대경선 실시간 스마트 좌석 알리미
 * 핵심 판정 로직 (Sensing -> Processing -> Actuating, Closed-Loop)
 *
 * 이 C 코드 하나가
 *   1) 네이티브(gcc)로 컴파일되어 단위 테스트에서 검증되고,
 *   2) WebAssembly(clang --target=wasm32)로 컴파일되어 웹 화면에서 그대로 실행된다.
 * 즉, 웹 화면에 찍히는 모든 좌석 상태는 이 파일의 로직으로만 결정된다.
 *
 * 표준 라이브러리에 의존하지 않는다(freestanding) - 열차 내 MCU/게이트웨이 이식 고려.
 */
#ifndef JARIKOK_H
#define JARIKOK_H

#ifdef __wasm__
#define JK_API __attribute__((visibility("default")))
#else
#define JK_API
#endif

/* ---------------- 열차 구성 (11주차 과제 기준) ----------------
 * 대경선 2량 1편성: 총 296명 = 좌석 78석 + 입석 218명
 * 좌석 78석 = 1량당 39석. 롱시트(벤치) 6개로 모식화:
 *   한쪽 벽: 7 + 7 + 6 = 20석 / 반대쪽 벽: 7 + 7 + 5 = 19석
 */
#define JK_CARS            2
#define JK_SEATS_PER_CAR   39
#define JK_SEATS           (JK_CARS * JK_SEATS_PER_CAR)   /* 78 */
#define JK_STANDING        218
#define JK_CAPACITY        296
#define JK_BENCHES_PER_CAR 6

/* 대경선 정차역 (구미 -> 경산 방향) */
#define JK_STATIONS        8

/* ---------------- 좌석 상태 ---------------- */
enum jk_state {
    JK_EMPTY    = 0,  /* 빈자리                                   */
    JK_OCCUPIED = 1,  /* 사람 착석                                 */
    JK_LEAVING  = 2,  /* 곧 내릴 자리 (이번 역 하차 예정)          */
    JK_BAG      = 3,  /* 가방(짐)만 놓인 자리   : 압력 T + 정전용량 F */
    JK_SPREAD   = 4   /* 한 사람이 2칸 차지한 자리                 */
};

/* 웹 화면 표시 분류 (과제의 3가지 포인팅 + 일반 착석) */
enum jk_display {
    JK_DISP_OCCUPIED = 0, /* 회색   */
    JK_DISP_EMPTY    = 1, /* 파랑   : 현재 빈 좌석                 */
    JK_DISP_LEAVING  = 2, /* 주황   : 곧 내릴 하차 예정 좌석        */
    JK_DISP_BLOCKED  = 3  /* 빨강   : 가방 적재 및 2칸 차지 좌석    */
};

/* 판정 근거 (어떤 규칙이 적용됐는지 웹에서 보여주기 위함) */
enum jk_rule {
    JK_R_P0          = 0, /* 압력 F               -> 빈자리 (YOLO 미실행)          */
    JK_R_P1_HEAD     = 1, /* 압력 T + 머리 검출    -> 착석                          */
    JK_R_P1C1_NOHEAD = 2, /* 압력 T + 정전용량 T + 머리 없음 + 이웃 머리 없음 -> 착석(가림) */
    JK_R_P1C1_SPREAD = 3, /* 압력 T + 정전용량 T + 머리 없음 + 이웃 좌석에 머리 -> 2칸 차지 */
    JK_R_P1C0_BAG    = 4  /* 압력 T + 정전용량 F + 머리 없음 -> 가방                 */
};

/* ---------------- 임계값 (Processing 파라미터) ---------------- */
#define JK_P_TH_KG          5.0f   /* 압력 Active High 임계: 합산 하중 >= 5kg -> True */
#define JK_C_TH_PF          3.0f   /* 정전용량 임계: 기준선 대비 증가량 >= 3pF -> True  */
#define JK_DEBOUNCE         3      /* 같은 판정이 3주기(3초) 연속이어야 상태 확정     */
#define JK_COP_TH_INIT      0.30f  /* CoP 전방 이동 임계 (적응형, 폐루프로 조정)      */
#define JK_COP_TH_MIN       0.20f
#define JK_COP_TH_MAX       0.50f
#define JK_HEAD_RISE_TH     0.10f  /* YOLO 머리 높이 상승 임계 (정규화 좌표)          */
#define JK_NEAR_ETA_S       90     /* 다음 역 도착 90초 전부터 하차 징후 해석         */
#define JK_LEAVE_P_TH       0.70f  /* 하차 확률 >= 70% -> 곧 내릴 자리                */
#define JK_COP_BASE_CYCLES  5      /* 착석 직후 5주기 동안 개인별 CoP 기준자세 학습   */
#define JK_HRI_CYCLES       60     /* 가방/2칸 차지가 60초 이상 지속 + 입석 승객 존재 -> HRI 경고 */

/* 로그 오즈 가중치 (역 간 하차 확률 계산) */
#define JK_W_COP            3.0f
#define JK_W_HEAD           1.5f
#define JK_W_STILL         -0.5f   /* 역 근접인데 아무 징후 없음 -> 하차 확률 감소 */

/* ---------------- API ---------------- */
/* 초기화 */
JK_API void  jk_init(void);

/* [Sensing] 좌석 센서 입력: 넓은 압력센서 4분할(앞왼/앞오/뒤왼/뒤오, kg), 정전용량(pF) */
JK_API void  jk_set_sensor(int seat, float fl, float fr, float rl, float rr, float cap_pf);
/* [Sensing] 압력/정전용량을 이진화하고 YOLO 실행 대상(압력 True 좌석)을 선별. 반환: YOLO 대상 좌석 수 */
JK_API int   jk_stage_sensing(void);
JK_API int   jk_yolo_requested(int seat);
/* [Sensing] Top-View CCTV 좌석 박스(ROI) 안 YOLO 머리 추적 결과 */
JK_API void  jk_set_head(int seat, int heads_in_roi, float head_rise);
/* [Sensing] 비콘 기반 열차 위치: 다음 역 번호, 도착까지 남은 초, 정차 중 여부 */
JK_API void  jk_set_train(int next_station, int eta_s, int at_station);

/* [Processing] 진리표 판정 + 디바운스 + 하차 확률 계산 */
JK_API void  jk_stage_processing(void);
/* [Actuating] 웹/LED 출력 생성 + 폐루프 피드백(임계 보정, 역별 하차율 학습, 기준선 보정) */
JK_API void  jk_stage_actuating(void);

/* 출력 조회 */
JK_API int   jk_state(int seat);
JK_API int   jk_display(int seat);
JK_API int   jk_rule(int seat);
JK_API int   jk_led_rgb(int seat);        /* 2단계 LED 스트립 색 (0xRRGGBB, 0 = 꺼짐) */
JK_API int   jk_hri_alert(int seat);      /* 가방/2칸 차지 장기 점유 경고 */
JK_API float jk_leave_prob(int seat);
JK_API float jk_cop_y(int seat);
JK_API float jk_cop_shift(int seat);
JK_API float jk_total_kg(int seat);
JK_API float jk_cap_delta(int seat);
JK_API int   jk_bit_p(int seat);
JK_API int   jk_bit_c(int seat);
JK_API int   jk_heads(int seat);
JK_API int   jk_count_display(int disp);           /* 표시 분류별 좌석 수 (전체) */
JK_API int   jk_count_display_car(int car, int disp);
JK_API float jk_expected_free_next(void);           /* 다음 역에서 생길 것으로 예상되는 빈자리 수 */
JK_API int   jk_recommend_car(void);                /* 추천 칸 (0/1) */
JK_API int   jk_yolo_runs(void);                    /* 이번 주기 YOLO 실행 좌석 수 */
JK_API float jk_cop_threshold(void);
JK_API float jk_station_rate(int station);
JK_API int   jk_stat_hits(void);
JK_API int   jk_stat_false_alarms(void);
JK_API int   jk_stat_misses(void);

/* 좌석 배치 */
JK_API int   jk_bench_len(int bench);               /* 0..5 */
JK_API int   jk_left_neighbor(int seat);            /* 같은 벤치의 왼쪽 좌석, 없으면 -1 */
JK_API int   jk_right_neighbor(int seat);

/* 순수 함수 (테스트용) */
JK_API int   jk_truth_table(int p, int c, int heads, int neighbor_head, int *rule_out);
JK_API float jk_alight_probability(float prior, int near, int cop_forward, int head_rise, int terminus);
JK_API float jk_sigmoid(float x);
JK_API float jk_logit(float p);

#endif
