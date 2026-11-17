"""report/*.md (공정 방법, 결론) -> assets/report.js : 웹사이트의 '공정 방법' / '결론' 탭에 그대로 표시
GitHub 에서는 ```mermaid 블록이 그림으로 보이고, 웹사이트에서는 같은 구조의 HTML 블록도로 바꿔 넣는다."""
import json
import pathlib
import re
import sys

import markdown

DIAGRAM = """
<div class="loop">
  <div class="lp lp-s"><div class="lp-h">① SENSING</div>
    <div>넓은 압력센서 4분할 → <b>P, CoP</b></div>
    <div>정전용량 센서 → <b>C</b></div>
    <div>Top-View CCTV + YOLO → <b>좌석 ROI 머리</b></div>
    <div>비콘 · 기차 Wi-Fi → <b>편성 · 다음 역 · ETA</b></div></div>
  <div class="lp-arrow">→<small>P=True 좌석만<br>YOLO 요청 ⇄</small></div>
  <div class="lp lp-p"><div class="lp-h">② PROCESSING</div>
    <div>압력 Active High 게이팅</div>
    <div>진리표 판정 + 3초 디바운스</div>
    <div>CoP Shift 탐지</div>
    <div>역 간 하차 확률 계산</div></div>
  <div class="lp-arrow">→</div>
  <div class="lp lp-a"><div class="lp-h">③ ACTUATING</div>
    <div>기차 Wi-Fi 웹 시트 모식도</div>
    <div><i class="sw s-empty"></i>빈자리 <i class="sw s-leave"></i>곧 내릴 <i class="sw s-block"></i>가방·2칸</div>
    <div>(2단계) 좌석 하단 LED HRI</div></div>
  <div class="lp-fb">↺ FEEDBACK &nbsp; F1 하차 예측 검증 → CoP 임계 적응 · F2 역별 하차율 학습 · F3 정전용량 기준선 보정 · F4 HRI 후 승객 반응 재감지</div>
</div>
"""

root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out = {}
for key, name in [("process", "03_공정방법.md"), ("conclusion", "07_결론.md")]:
    text = (root / "report" / name).read_text(encoding="utf-8")
    text = re.sub(r"```mermaid\n.*?```", "<!--DIAGRAM-->", text, flags=re.S)
    html = markdown.markdown(text, extensions=["tables", "fenced_code"])
    out[key] = html.replace("<!--DIAGRAM-->", DIAGRAM)
js = "// 자동 생성 파일 - report/*.md 를 수정한 뒤 make report 로 다시 만드세요.\nwindow.JARIKOK_REPORT = " + json.dumps(out, ensure_ascii=False) + ";\n"
(root / "assets" / "report.js").write_text(js, encoding="utf-8")
print("assets/report.js written")
