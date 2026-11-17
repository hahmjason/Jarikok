// =====================================================================
// 자리콕 웹 - 가상 열차/센서 시뮬레이터 (물리 세계 역할)
// ---------------------------------------------------------------------
// 이 파일은 "판정"을 하지 않는다. 승객의 행동을 흉내 내어
//   - 좌석 압력센서 4분할 값, 정전용량 값 (Sensing 원시값)
//   - C 로직이 '요청한' 좌석에 대해서만 YOLO 머리 추적 결과
//   - 비콘이 알려주는 다음 역 / ETA
// 를 만들어 낼 뿐이다. 좌석 상태 판정은 전부 c/jarikok.c (WebAssembly) 가 한다.
// =====================================================================
(function (root) {
  'use strict';

  const STATIONS = ['구미', '사곡', '북삼', '왜관', '서대구', '대구', '동대구', '경산'];
  // 역 간 주행 시간(초) - 모식화 값
  const RUN_S = [0, 240, 300, 300, 600, 240, 180, 480];
  const DWELL_S = 30;
  // '실제' 승객이 내리는 역 가중치 (C 로직의 사전확률과 일부러 조금 다르게 -> 학습 확인용)
  const ALIGHT_W = [0, 1, 1, 2.5, 3, 4.5, 6, 8];

  function rnd(a, b) { return a + Math.random() * (b - a); }
  function gauss(sd) { return (Math.random() + Math.random() + Math.random() - 1.5) * sd * 1.15; }
  function pickAlight(fromIdx) {
    let tot = 0;
    for (let s = fromIdx + 1; s < STATIONS.length; s++) tot += ALIGHT_W[s];
    let r = Math.random() * tot;
    for (let s = fromIdx + 1; s < STATIONS.length; s++) { r -= ALIGHT_W[s]; if (r <= 0) return s; }
    return STATIONS.length - 1;
  }

  class TrainSim {
    constructor(nSeats, neighbors) {
      this.n = nSeats;
      this.neighbors = neighbors;          // seat -> [left, right] (C 로직이 알려준 배치)
      this.reset();
    }

    reset() {
      this.seats = new Array(this.n).fill(null);
      this.next = 1;               // 다음 역
      this.eta = RUN_S[1];
      this.atStation = false;
      this.dwell = 0;
      this.time = 0;
      this.drift = 0;
      this.uid = 0;
      this.log = [];
      // 구미 출발 시점: 약 85% 착석 + 가방 + 2칸 차지 몇 건
      for (let i = 0; i < this.n; i++) if (Math.random() < 0.85) this.seatPerson(i, 0);
      for (let k = 0; k < 4; k++) this.placeBagNearRandom(0);
      for (let k = 0; k < 2; k++) this.spreadRandom(0);
    }

    newPerson(from) {
      return {
        kind: 'person', id: ++this.uid,
        alight: pickAlight(from),
        w: rnd(45, 85),
        copBase: rnd(-0.25, 0.05),
        coat: Math.random() < 0.12,          // 두꺼운 외투: 정전용량 약화
        prepLead: rnd(20, 75),               // 도착 몇 초 전에 엉덩이를 앞으로 옮기는지
        willPrep: Math.random() < 0.88,      // 하차 전 자세를 바꾸는 사람 비율
        fidget: Math.random() < 0.06,        // 안 내리는데 앞으로 당겨 앉는 사람(오경보 원인)
        lean: 0, rise: 0,
      };
    }

    seatPerson(i, from) { this.seats[i] = this.newPerson(from); }

    emptyNeighbor(i) {
      const nb = this.neighbors[i].filter(j => j >= 0 && this.seats[j] === null);
      return nb.length ? nb[Math.floor(Math.random() * nb.length)] : -1;
    }

    placeBag(owner) {
      const j = this.emptyNeighbor(owner);
      if (j < 0) return false;
      this.seats[j] = { kind: 'bag', owner, w: rnd(5.5, 11) };
      this.seats[owner].bagSeat = j;
      return true;
    }

    spread(owner) {
      const j = this.emptyNeighbor(owner);
      if (j < 0) return false;
      this.seats[j] = { kind: 'spread', owner, w: rnd(14, 26) };
      this.seats[owner].spreadSeat = j;
      return true;
    }

    persons() {
      const out = [];
      this.seats.forEach((o, i) => { if (o && o.kind === 'person') out.push(i); });
      return out;
    }

    placeBagNearRandom() {
      const p = this.persons(); if (!p.length) return;
      for (let t = 0; t < 10; t++) { const i = p[Math.floor(Math.random() * p.length)]; if (this.placeBag(i)) return; }
    }
    spreadRandom() {
      const p = this.persons(); if (!p.length) return;
      for (let t = 0; t < 10; t++) { const i = p[Math.floor(Math.random() * p.length)]; if (this.spread(i)) return; }
    }

    leave(i) {
      const o = this.seats[i];
      if (!o || o.kind !== 'person') return;
      if (o.bagSeat != null && this.seats[o.bagSeat] && this.seats[o.bagSeat].owner === i) this.seats[o.bagSeat] = null;
      if (o.spreadSeat != null && this.seats[o.spreadSeat] && this.seats[o.spreadSeat].owner === i) this.seats[o.spreadSeat] = null;
      this.seats[i] = null;
    }

    // HRI 경고(LED 점멸)를 본 승객이 가방을 치우거나 바로 앉는 반응 -> 폐루프의 사람 쪽 고리
    reactToHri(hri) {
      for (let i = 0; i < this.n; i++) {
        if (!hri[i]) continue;
        const o = this.seats[i];
        if (!o || o.kind === 'person') continue;
        if (Math.random() < 0.08) {
          const owner = this.seats[o.owner];
          if (owner) { owner.bagSeat = null; owner.spreadSeat = null; }
          this.seats[i] = null;
          this.log.push(`HRI 경고 후 ${i + 1}번 좌석 정리됨`);
        }
      }
    }

    // ---- 1초 진행 ----
    step() {
      this.time++;
      this.drift = 1.2 + 1.0 * Math.sin(this.time / 400);     // 온도·습도에 따른 정전용량 드리프트

      if (this.atStation) {
        this.dwell++;
        if (this.dwell <= 6) {
          // 하차: 이번 역이 목적지인 사람들이 처음 몇 초 동안 내림
          for (let i = 0; i < this.n; i++) {
            const o = this.seats[i];
            if (o && o.kind === 'person' && o.alight === this.next && Math.random() < 0.45) this.leave(i);
          }
        } else if (this.next < STATIONS.length - 1) {
          // 승차: 입석 승객이 빈자리에 앉음 (일부는 가방을 옆자리에 두거나 2칸 차지)
          for (let i = 0; i < this.n; i++) {
            if (this.seats[i] === null && Math.random() < 0.10) {
              this.seatPerson(i, this.next);
              const r = Math.random();
              if (r < 0.04) this.placeBag(i); else if (r < 0.06) this.spread(i);
            }
          }
        }
        if (this.dwell >= DWELL_S) {
          if (this.next === STATIONS.length - 1) { this.log.push('경산 종착 - 새 운행 시작'); this.reset(); return; }
          this.atStation = false;
          this.next++;
          this.eta = RUN_S[this.next];
        }
      } else {
        this.eta--;
        if (this.eta <= 0) { this.eta = 0; this.atStation = true; this.dwell = 0; }
      }

      // 자세 변화
      for (let i = 0; i < this.n; i++) {
        const o = this.seats[i];
        if (!o || o.kind !== 'person') continue;
        const target = o.alight === this.next;
        if (target && o.willPrep && !this.atStation && this.eta <= o.prepLead) o.lean = Math.min(o.lean + 0.12, rnd(0.38, 0.55));
        else if (o.fidget && !this.atStation && this.eta < 70 && this.eta > 20) o.lean = Math.min(o.lean + 0.1, 0.33);
        else o.lean = Math.max(o.lean - 0.1, 0);
        o.rise = (target && o.willPrep && (this.atStation || this.eta <= 12)) ? 0.16 : 0;
      }
    }

    // ---- Sensing 원시값 ----
    sensor(i) {
      const o = this.seats[i];
      const n = () => Math.abs(gauss(0.15));
      let w = 0, cop = 0, cap = this.drift + gauss(0.25);
      if (!o) {
        if (Math.random() < 0.01) cap += 6;      // 손·옷이 스침 (정전용량만 True)
        return [n(), n(), n(), n(), cap];
      }
      if (o.kind === 'person') {
        w = o.w * 0.75;                           // 좌석에 실리는 하중
        cop = Math.max(-0.95, Math.min(0.95, o.copBase + o.lean + gauss(0.03)));
        cap += o.coat ? rnd(1.0, 2.6) : rnd(8, 13);
      } else if (o.kind === 'bag') {
        w = o.w; cop = gauss(0.2); cap += rnd(0, 0.6);
      } else {                                    // 2칸 차지: 몸 일부(엉덩이·다리)가 걸쳐 있음
        w = o.w; cop = gauss(0.2); cap += rnd(6, 9);
      }
      const front = w * (1 + cop) / 2, rear = w - front;
      const lr = rnd(0.45, 0.55);
      return [front * lr + n(), front * (1 - lr) + n(), rear * lr + n(), rear * (1 - lr) + n(), cap];
    }

    // ---- 가상 YOLO: C 로직이 요청한 좌석 ROI 에 대해서만 호출됨 ----
    yolo(i) {
      const o = this.seats[i];
      if (!o || o.kind !== 'person') return [0, 0];
      if (Math.random() < 0.03) return [0, 0];     // 3% 미검출
      return [1, o.rise + gauss(0.01)];
    }

    // 화면 설명용 "실제 정답" (판정에는 쓰지 않음)
    truth(i) { const o = this.seats[i]; return o ? o.kind : 'empty'; }
    truthAlight(i) { const o = this.seats[i]; return o && o.kind === 'person' ? o.alight : -1; }
  }

  root.JarikokSim = { TrainSim, STATIONS, RUN_S, DWELL_S };
})(typeof window !== 'undefined' ? window : globalThis);
