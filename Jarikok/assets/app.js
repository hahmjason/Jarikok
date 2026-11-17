// =====================================================================
// 자리콕 웹 - 화면(Actuating 출력부)
// 좌석 판정은 하지 않는다. C 로직(WebAssembly)의 출력만 읽어 그린다.
// =====================================================================
(async function () {
  'use strict';
  const { TrainSim, STATIONS } = window.JarikokSim;

  // ---------- 1. C 로직(WebAssembly) 로드 ----------
  const bin = Uint8Array.from(atob(window.JARIKOK_WASM_B64), c => c.charCodeAt(0));
  const { instance } = await WebAssembly.instantiate(bin, {});
  const C = instance.exports;
  C.jk_init();

  const SEATS = 78, PER_CAR = 39;
  const BENCH = [0, 1, 2, 3, 4, 5].map(b => C.jk_bench_len(b));     // [7,7,6,7,7,5]
  const DISP = { OCC: 0, EMPTY: 1, LEAVE: 2, BLOCK: 3 };
  const DISP_CLASS = ['s-occ', 's-empty', 's-leave', 's-block'];
  const DISP_NAME = ['착석', '빈자리', '곧 내릴 자리', '가방·2칸 차지'];
  const STATE_NAME = ['빈자리', '착석', '곧 내릴 자리', '가방', '2칸 차지'];
  const RULE_TEXT = [
    '압력 F → 빈자리 (YOLO 미실행)',
    '압력 T + ROI 머리 검출 → 착석',
    '압력 T + 정전용량 T + 머리 없음 + 옆자리 머리 없음 → 착석(머리 가림)',
    '압력 T + 정전용량 T + 머리 없음 + 옆자리 머리 있음 → 2칸 차지',
    '압력 T + 정전용량 F + 머리 없음 → 가방',
  ];
  const TRUTH_KIND = { empty: '비어 있음', person: '사람', bag: '가방', spread: '2칸 차지(몸 일부)' };

  const neighbors = [];
  for (let i = 0; i < SEATS; i++) neighbors.push([C.jk_left_neighbor(i), C.jk_right_neighbor(i)]);

  let sim = new TrainSim(SEATS, neighbors);
  let speed = 5, playing = true, timer = null, selected = -1;
  const lastSensor = [];
  let prevAt = false, prevStats = { h: 0, f: 0, m: 0 };
  const trainNo = String(1 + Math.floor(Math.random() * 15)).padStart(2, '0');
  document.getElementById('train-no').textContent = `제 ${trainNo}편성`;
  document.getElementById('beacon').textContent = `DG${trainNo}-1`;

  // ---------- 2. 한 주기 = Sensing → Processing → Actuating ----------
  function cycle() {
    sim.step();
    for (let i = 0; i < SEATS; i++) { const s = sim.sensor(i); lastSensor[i] = s; C.jk_set_sensor(i, s[0], s[1], s[2], s[3], s[4]); }
    C.jk_set_train(sim.next, sim.eta, sim.atStation ? 1 : 0);
    C.jk_stage_sensing();
    for (let i = 0; i < SEATS; i++) {
      if (C.jk_yolo_requested(i)) { const y = sim.yolo(i); C.jk_set_head(i, y[0], y[1]); }   // 게이팅된 좌석만
    }
    C.jk_stage_processing();
    C.jk_stage_actuating();
    const hri = []; for (let i = 0; i < SEATS; i++) hri.push(C.jk_hri_alert(i));
    sim.reactToHri(hri);
    events();
    render();
  }

  // ---------- 3. 이벤트 로그 ----------
  const logEl = document.getElementById('log');
  function log(msg, cls) {
    const li = document.createElement('li');
    li.innerHTML = `<span class="t">${clock()}</span> ${msg}`;
    if (cls) li.className = cls;
    logEl.prepend(li);
    while (logEl.children.length > 40) logEl.lastChild.remove();
  }
  function events() {
    while (sim.log.length) log(sim.log.shift(), 'sys');
    if (sim.atStation && !prevAt) {
      const n = C.jk_count_display(DISP.LEAVE);
      log(`<b>${STATIONS[sim.next]}</b> 도착 · 곧 내릴 자리로 예측한 좌석 ${n}석`);
    }
    if (!sim.atStation && prevAt) {
      const h = C.jk_stat_hits(), f = C.jk_stat_false_alarms(), m = C.jk_stat_misses();
      const st = STATIONS[sim.next - 1];
      log(`<b>${st}</b> 출발 · 피드백: 적중 +${h - prevStats.h}, 오경보 +${f - prevStats.f}, 놓침 +${m - prevStats.m} → θ=${C.jk_cop_threshold().toFixed(3)}, r(${st})=${C.jk_station_rate(sim.next - 1).toFixed(2)}`, 'fb');
      prevStats = { h, f, m };
    }
    if (sim.time === 1) prevStats = { h: C.jk_stat_hits(), f: C.jk_stat_false_alarms(), m: C.jk_stat_misses() };
    prevAt = sim.atStation;
  }

  // ---------- 4. 좌석 모식도 ----------
  const carsEl = document.getElementById('cars');
  const seatEls = [];
  function buildCars() {
    carsEl.innerHTML = '';
    for (let car = 0; car < 2; car++) {
      const wrap = document.createElement('div');
      wrap.className = 'car';
      wrap.innerHTML = `<div class="car-h"><b>${car + 1}호차</b><span class="car-sum" id="car-sum-${car}"></span></div>`;
      const body = document.createElement('div'); body.className = 'car-body';
      let base = car * PER_CAR;
      for (let wall = 0; wall < 2; wall++) {
        const w = document.createElement('div'); w.className = 'wall';
        for (let b = wall * 3; b < wall * 3 + 3; b++) {
          if (b % 3) { const d = document.createElement('div'); d.className = 'door'; w.appendChild(d); }
          const bench = document.createElement('div'); bench.className = 'bench';
          for (let k = 0; k < BENCH[b]; k++) {
            const idx = base++;
            const s = document.createElement('button');
            s.className = 'seat'; s.dataset.i = idx;
            s.setAttribute('aria-label', `${idx + 1}번 좌석`);
            s.onclick = () => { selected = idx; renderDetail(); renderSel(); renderTruthTable(); };
            bench.appendChild(s); seatEls[idx] = s;
          }
          w.appendChild(bench);
        }
        body.appendChild(w);
        if (wall === 0) { const a = document.createElement('div'); a.className = 'aisle'; a.textContent = '통로 · 입석'; body.appendChild(a); }
      }
      wrap.appendChild(body);
      carsEl.appendChild(wrap);
    }
  }

  // ---------- 5. 렌더링 ----------
  function clock() {
    const t = 8 * 3600 + 12 * 60 + sim.time;
    return `${String(Math.floor(t / 3600) % 24).padStart(2, '0')}:${String(Math.floor(t / 60) % 60).padStart(2, '0')}:${String(t % 60).padStart(2, '0')}`;
  }
  const $ = id => document.getElementById(id);

  function render() {
    $('clock').textContent = clock().slice(0, 5);
    // 노선
    $('route').innerHTML = STATIONS.map((s, i) => {
      let cls = i < sim.next ? 'passed' : '';
      if (i === sim.next) cls = sim.atStation ? 'here' : 'next';
      if (i === sim.next - 1 && !sim.atStation) cls = 'passed last';
      return `<div class="st ${cls}"><i></i><span>${s}</span></div>`;
    }).join('');
    $('next-label').textContent = sim.atStation ? '정차 중' : '다음 역';
    $('next-st').textContent = STATIONS[sim.next] + (sim.next === STATIONS.length - 1 ? ' (종착)' : '');
    $('eta').textContent = sim.atStation ? '문 열림' : `${Math.floor(sim.eta / 60)}분 ${sim.eta % 60}초 후`;

    $('cnt-empty').textContent = C.jk_count_display(DISP.EMPTY);
    $('cnt-leave').textContent = C.jk_count_display(DISP.LEAVE);
    $('cnt-block').textContent = C.jk_count_display(DISP.BLOCK);
    const rec = C.jk_recommend_car();
    const exp = C.jk_expected_free_next();
    $('forecast').innerHTML = sim.next === STATIONS.length - 1
      ? `<b>${STATIONS[sim.next]}</b>(종착)에서 모든 좌석이 비워집니다`
      : `<b>${STATIONS[sim.next]}</b>에서 약 <b>${exp.toFixed(1)}석</b> 빈자리 예상 · 추천 <b>${rec + 1}호차</b>${rec !== 0 ? ' (지금 1호차 → 이동 추천)' : ' (지금 칸)'}`;

    for (let car = 0; car < 2; car++) {
      $(`car-sum-${car}`).innerHTML =
        `<span class="t-empty">빈 ${C.jk_count_display_car(car, DISP.EMPTY)}</span> · <span class="t-leave">곧 ${C.jk_count_display_car(car, DISP.LEAVE)}</span> · <span class="t-block">가방 ${C.jk_count_display_car(car, DISP.BLOCK)}</span>${car === rec ? ' <span class="rec">추천</span>' : ''}`;
    }

    let hriN = 0;
    for (let i = 0; i < SEATS; i++) {
      const d = C.jk_display(i), st = C.jk_state(i), hri = C.jk_hri_alert(i);
      hriN += hri;
      const el = seatEls[i];
      el.className = `seat ${DISP_CLASS[d]}${hri ? ' hri' : ''}${st === 4 ? ' spread' : ''}${st === 3 ? ' bagi' : ''}${i === selected ? ' sel' : ''}`;
      el.title = `${i + 1}번 · ${STATE_NAME[st]}` + (d === DISP.LEAVE ? ` (하차확률 ${(C.jk_leave_prob(i) * 100).toFixed(0)}%)` : '');
    }

    // 파이프라인
    let pT = 0; for (let i = 0; i < SEATS; i++) pT += C.jk_bit_p(i);
    const yr = C.jk_yolo_runs();
    $('p-true').textContent = pT;
    $('yolo-runs').textContent = yr;
    $('yolo-save').textContent = `${Math.round((1 - yr / SEATS) * 100)}%`;
    $('cop-th').textContent = C.jk_cop_threshold().toFixed(3);
    $('exp-free').textContent = exp.toFixed(1);
    $('rate-next').textContent = `${STATIONS[sim.next]} ${C.jk_station_rate(sim.next).toFixed(2)}`;
    $('hri-n').textContent = hriN;
    $('rec-car').textContent = `${rec + 1}호차`;
    const h = C.jk_stat_hits(), f = C.jk_stat_false_alarms(), m = C.jk_stat_misses();
    $('fb-hit').textContent = h; $('fb-fa').textContent = f; $('fb-miss').textContent = m;
    $('fb-prec').textContent = h + f ? `${Math.round(h / (h + f) * 100)}%` : '-';
    $('rates').innerHTML = STATIONS.slice(1, -1).map((s, k) => {
      const r = C.jk_station_rate(k + 1);
      return `<div class="rate"><span>${s}</span><div class="bar"><i style="width:${Math.min(100, r * 100 / 0.6)}%"></i></div><b>${r.toFixed(2)}</b></div>`;
    }).join('');
    // LED 스트립 (1호차)
    const led = $('led-strip');
    if (!led.children.length) for (let i = 0; i < PER_CAR; i++) led.appendChild(document.createElement('i'));
    for (let i = 0; i < PER_CAR; i++) {
      const rgb = C.jk_led_rgb(i);
      const e = led.children[i];
      e.style.background = rgb ? '#' + rgb.toString(16).padStart(6, '0') : '#1f2937';
      e.className = C.jk_hri_alert(i) ? 'blink' : '';
    }

    renderDetail();
    if (document.getElementById('tab-logic').classList.contains('active')) renderTruthTable();
  }

  function renderSel() { seatEls.forEach((e, i) => e.classList.toggle('sel', i === selected)); }

  function bit(v) { return `<span class="bit b${v}">${v ? 'True' : 'False'}</span>`; }

  function renderDetail() {
    if (selected < 0) return;
    const i = selected;
    const car = Math.floor(i / PER_CAR) + 1;
    const st = C.jk_state(i), d = C.jk_display(i), rule = C.jk_rule(i);
    const p = C.jk_bit_p(i), c = C.jk_bit_c(i), yreq = C.jk_yolo_requested(i);
    const fsr = lastSensor[i] || [0, 0, 0, 0, 0];   // 이번 주기에 C 로직에 넣은 값
    const prob = C.jk_leave_prob(i);
    const truth = sim.truth(i), ta = sim.truthAlight(i);
    $('d-seat').textContent = `${car}호차 · ${i + 1}번 좌석`;
    $('d-body').className = '';
    $('d-body').innerHTML = `
      <div class="d-state ${DISP_CLASS[d]}">${STATE_NAME[st]} <span>→ 웹 표시: ${DISP_NAME[d]}</span></div>
      <table class="kv">
        <tr><th colspan="2" class="sec">① Sensing</th></tr>
        <tr><td>압력 합계</td><td><b>${C.jk_total_kg(i).toFixed(1)} kg</b> (임계 5kg) → P ${bit(p)}</td></tr>
        <tr><td>4분할 하중(kg)</td><td class="small">앞왼 ${fsr[0].toFixed(1)} · 앞오 ${fsr[1].toFixed(1)} · 뒤왼 ${fsr[2].toFixed(1)} · 뒤오 ${fsr[3].toFixed(1)}</td></tr>
        <tr><td>정전용량 ΔC</td><td><b>${C.jk_cap_delta(i).toFixed(2)} pF</b> (임계 3pF) → C ${bit(c)}</td></tr>
        <tr><td>YOLO 머리 추적</td><td>${yreq ? `실행됨 · ROI 머리 <b>${C.jk_heads(i)}</b>개` : '<span class="muted">미실행 (압력 False → 게이팅)</span>'}</td></tr>
        <tr><th colspan="2" class="sec">② Processing</th></tr>
        <tr><td>진리표 규칙</td><td>${RULE_TEXT[rule]}</td></tr>
        <tr><td>무게중심 CoP</td><td>${C.jk_cop_y(i).toFixed(2)} · 기준 대비 이동 <b>${C.jk_cop_shift(i) >= 0 ? '+' : ''}${C.jk_cop_shift(i).toFixed(2)}</b> (θ ${C.jk_cop_threshold().toFixed(2)})</td></tr>
        <tr><td>이번 역 하차 확률</td><td>${(st === 1 || st === 2) ? `<b>${(prob * 100).toFixed(0)}%</b> ${prob >= 0.7 ? '≥ 70% → 곧 내릴 자리' : '(70% 미만)'}` : '<span class="muted">착석 아님</span>'}</td></tr>
        <tr><th colspan="2" class="sec">③ Actuating</th></tr>
        <tr><td>LED (2단계)</td><td><i class="sw" style="background:${C.jk_led_rgb(i) ? '#' + C.jk_led_rgb(i).toString(16).padStart(6, '0') : '#1f2937'}"></i> ${C.jk_hri_alert(i) ? '<b class="t-block">HRI 점멸 경고</b>' : ''}</td></tr>
        <tr><th colspan="2" class="sec">정답(시뮬레이터 내부 · 판정에는 미사용)</th></tr>
        <tr><td>실제 상태</td><td>${TRUTH_KIND[truth]}${ta >= 0 ? ` · 실제 하차역 ${STATIONS[ta]}` : ''}</td></tr>
      </table>`;
  }

  // ---------- 6. 진리표 탭 ----------
  const TT_ROWS = [
    { p: 0, c: 0, h: 0, nh: 0, hs: '(미실행)', ns: '-' },
    { p: 0, c: 1, h: 0, nh: 0, hs: '(미실행)', ns: '-' },
    { p: 1, c: 1, h: 1, nh: 0, hs: '있음', ns: '-' },
    { p: 1, c: 0, h: 1, nh: 0, hs: '있음', ns: '-' },
    { p: 1, c: 1, h: 0, nh: 1, hs: '없음', ns: '있음' },
    { p: 1, c: 1, h: 0, nh: 0, hs: '없음', ns: '없음' },
    { p: 1, c: 0, h: 0, nh: 0, hs: '없음', ns: '-' },
  ];
  // 판정 열은 C 의 jk_truth_table 을 실제로 호출해서 채운다 (rule 출력 포인터는 사용하지 않음: 0)
  const DISP_OF_STATE = [DISP.EMPTY, DISP.OCC, DISP.LEAVE, DISP.BLOCK, DISP.BLOCK];
  TT_ROWS.forEach(r => { r.state = C.jk_truth_table(r.p, r.c, r.h, r.nh, 0); });
  function rowRule(r) {
    if (!r.p) return 0; if (r.h) return 1; if (r.c) return r.nh ? 3 : 2; return 4;
  }
  function renderTruthTable() {
    const tb = document.querySelector('#tt tbody');
    const counts = TT_ROWS.map(() => 0);
    let selRow = -1;
    for (let i = 0; i < SEATS; i++) {
      const p = C.jk_bit_p(i), c = C.jk_bit_c(i), rule = C.jk_rule(i);
      TT_ROWS.forEach((r, k) => { if (r.p === p && r.c === c && rowRule(r) === rule) { counts[k]++; if (i === selected) selRow = k; } });
    }
    tb.innerHTML = TT_ROWS.map((r, k) => `<tr class="${k === selRow ? 'hl' : ''}">
      <td>${r.p}</td><td>${r.c}</td><td>${r.hs}</td><td>${r.ns}</td>
      <td><b>${STATE_NAME[r.state]}</b></td>
      <td><i class="sw ${DISP_CLASS[DISP_OF_STATE[r.state]]}"></i>${DISP_NAME[DISP_OF_STATE[r.state]]}</td>
      <td>${counts[k]}석</td></tr>`).join('');
  }

  // ---------- 7. 문서 ----------
  $('md-process').innerHTML = window.JARIKOK_REPORT.process;
  $('md-conclusion').innerHTML = window.JARIKOK_REPORT.conclusion;

  // ---------- 8. 제어 ----------
  function schedule() { clearInterval(timer); if (playing) timer = setInterval(cycle, 1000 / speed); }
  $('btn-play').onclick = () => { playing = !playing; $('btn-play').textContent = playing ? '일시정지' : '재생'; schedule(); };
  document.querySelectorAll('.speed').forEach(b => b.onclick = () => {
    speed = +b.dataset.speed; document.querySelectorAll('.speed').forEach(x => x.classList.toggle('active', x === b)); schedule();
  });
  $('btn-skip').onclick = () => { if (!sim.atStation && sim.eta > 95) { sim.eta = 95; log(`${STATIONS[sim.next]} 도착 95초 전으로 이동`, 'sys'); } };
  $('btn-reset').onclick = () => {
    C.jk_init(); sim = new TrainSim(SEATS, neighbors); prevAt = false; prevStats = { h: 0, f: 0, m: 0 };
    logEl.innerHTML = ''; log('구미 출발 · 새 운행 시작', 'sys'); render();
  };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.toggle('active', x === t));
    document.querySelectorAll('.panel').forEach(p => p.classList.toggle('active', p.id === 'tab-' + t.dataset.tab));
    if (t.dataset.tab === 'logic') renderTruthTable();
  });

  buildCars();
  log('구미 출발 · 기차 Wi-Fi 접속 → 자리콕 웹 자동 실행', 'sys');
  // 첫 화면이 비어 보이지 않도록 디바운스(3주기)만큼 미리 돌린다
  for (let k = 0; k < 8; k++) cycle();
  selected = 8; renderSel(); renderDetail();
  schedule();
  window.__jarikok = { C, get sim() { return sim; }, cycle };
})();
