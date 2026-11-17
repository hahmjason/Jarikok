# 자리콕 웹(Web) — 대경선 실시간 스마트 좌석 알리미

> **칸마다 헤맬 필요 없이 빈자리와 곧 내릴 자리를 웹으로 한 번에 찾자!**
> 좌석 상태 감지 센서(압력·정전용량·CCTV) + 기차 Wi-Fi 웹 시트 모식도 = **자리콕 웹** (1 + 1 = NEW 1)

글쓰기와 발표 3조 (담당교수 이은영) · 함준석(조장) · 진백성 · 김시원 · 서영환 · 최별

## 무엇이 들어 있나

| 경로 | 내용 |
|---|---|
| `c/jarikok.h`, `c/jarikok.c` | **핵심 판정 로직 (C)** — Sensing → Processing → Actuating → Feedback 폐루프 |
| `tests/test_jarikok.c` | 로직 단위 테스트 46개 (진리표, 게이팅, 디바운스, 하차 확률, 폐루프 피드백, HRI) |
| `index.html`, `assets/` | 시연 웹사이트 — 같은 C 코드를 **WebAssembly** 로 컴파일해 브라우저에서 실행 |
| `assets/sim.js` | 가상 열차·승객·센서 시뮬레이터 (판정은 하지 않고 센서 값만 생성) |
| `report/03_공정방법.md` | 제안서 Ⅲ-3 공정 방법 원고 |
| `report/07_결론.md` | 제안서 Ⅲ-7 결론 원고 |

## 과제(11주차)에서 정한 로직 ↔ 코드 대응

| 과제의 설계 | 코드 |
|---|---|
| 2량 1편성, 좌석 78석 + 입석 218 = 296 | `JK_SEATS 78`, `JK_STANDING 218`, `JK_CAPACITY 296` |
| 압력 True + 정전용량 True = 실제 사람 | `jk_truth_table()` → `JK_OCCUPIED` |
| 압력 True + 정전용량 False = 가방 놓인 비매너 자리 | `jk_truth_table()` → `JK_BAG` |
| 혼자 2칸 차지 | 몸은 이 좌석(P1·C1), 머리는 옆 좌석 → `JK_SPREAD` |
| 압력 정논리(Active High), 착석 신호 True 일 때만 YOLO | `jk_stage_sensing()` 의 `yolo_req = p`, `jk_set_head()` 는 요청 좌석만 수용 |
| Top-View CCTV 좌석 박스 + YOLO 머리 추적 | 좌석 ROI 머리 수 `heads`, 머리 상승 `head_rise` |
| 무게중심 이동(CoP Shift Detection) | 4분할 압력 → `cop_y`, 개인 기준자세 대비 `cop_shift ≥ θ` |
| 역 간 하차 확률 계산 | `jk_alight_probability()` : logit(역별 하차율) + 3.0·CoP + 1.5·머리상승 − 0.5·무징후 |
| 비콘·Wi-Fi 로 열차 자동 인식 → C 로직·HTML 웹 | `jk_set_train()` + WebAssembly 웹 화면 |
| 빈자리 / 곧 내릴 자리 / 가방·2칸 차지 포인팅 | `jk_display()` : 파랑 / 주황 / 빨강 (+ 착석 회색) |
| Closed-Loop | 출발 시 예측 검증 → θ 적응, 역별 하차율 학습, 정전용량 기준선 보정 |
| 확장: LED 스트립 (빈자리 파랑), 비매너 좌석 HRI | `jk_led_rgb()`, `jk_hri_alert()` |
| 확장: 도서관 사석화 방지 | 같은 진리표(P1·C0 = 가방만 있는 자리) 재사용 가능 |

## 실행

```bash
make test   # C 로직 단위 테스트
make all    # 테스트 + WebAssembly 빌드(assets/jarikok_wasm.js) + 보고서 HTML(assets/report.js)
```

웹사이트는 서버 없이 `index.html` 을 브라우저로 열어도 동작합니다. (wasm 을 base64 로 내장)

## 판정 규칙 요약

```
P = (4분할 하중 합계 ≥ 5kg)          C = (정전용량 - 기준선 ≥ 3pF)
P=0            → 빈자리 (YOLO 미실행)
P=1, 머리 있음  → 착석
P=1, C=1, 머리 없음, 옆자리 머리 있음 → 2칸 차지
P=1, C=1, 머리 없음, 옆자리 머리 없음 → 착석(가림)
P=1, C=0, 머리 없음 → 가방
3주기 연속 같은 판정일 때만 확정
다음 역 90초 전부터: 하차 확률 ≥ 70% → 곧 내릴 자리 (출발 시 해제)
```
