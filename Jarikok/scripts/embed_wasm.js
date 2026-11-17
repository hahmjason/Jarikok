// build/jarikok.wasm -> assets/jarikok_wasm.js (base64 내장)
const fs = require('fs');
const [, , src, dst] = process.argv;
const b64 = fs.readFileSync(src).toString('base64');
fs.writeFileSync(dst,
  '// 자동 생성 파일 - 직접 수정하지 마세요. (make wasm 으로 c/jarikok.c 에서 생성)\n' +
  'window.JARIKOK_WASM_B64 = "' + b64 + '";\n');
console.log('embedded', src, '->', dst, b64.length, 'chars');
