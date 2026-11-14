// Writes the relay box flat patterns as DXF (R12, mm) for laser/waterjet cutting:
// layer CUT = closed outer contour, layer BEND = bend lines (not to be cut).
//   node tools/relaisbox_dxf.mjs
import { mkdirSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as g from '../viewer/relaisbox-blech.js';

const out = fileURLToPath(new URL('../viewer/downloads/relaisbox/', import.meta.url));
mkdirSync(out, { recursive: true });

const f = (v) => v.toFixed(3);
function dxf(contour, bends, shift = [0, 0]) {
  const s = ['0', 'SECTION', '2', 'HEADER', '9', '$ACADVER', '1', 'AC1009', '9', '$INSUNITS', '70', '4', '0', 'ENDSEC',
    '0', 'SECTION', '2', 'TABLES', '0', 'TABLE', '2', 'LAYER', '70', '2',
    '0', 'LAYER', '2', 'CUT', '70', '0', '62', '7', '6', 'CONTINUOUS',
    '0', 'LAYER', '2', 'BEND', '70', '0', '62', '1', '6', 'CONTINUOUS', '0', 'ENDTAB', '0', 'ENDSEC',
    '0', 'SECTION', '2', 'ENTITIES', '0', 'POLYLINE', '8', 'CUT', '66', '1', '70', '1', '10', '0', '20', '0', '30', '0'];
  for (const [x, y] of contour) s.push('0', 'VERTEX', '8', 'CUT', '10', f(x + shift[0]), '20', f(y + shift[1]));
  s.push('0', 'SEQEND', '8', 'CUT');
  for (const [[x1, y1], [x2, y2]] of bends) s.push('0', 'LINE', '8', 'BEND', '10', f(x1 + shift[0]), '20', f(y1 + shift[1]), '11', f(x2 + shift[0]), '21', f(y2 + shift[1]));
  s.push('0', 'ENDSEC', '0', 'EOF');
  return s.join('\n') + '\n';
}

const tubShift = [g.TUB.flatL / 2, g.TUB.flatW / 2];
writeFileSync(out + 'pos1_wanne_abwicklung.dxf', dxf(g.tubOutline(32), g.tubBendLines().map((b) => b.p), tubShift));
writeFileSync(out + 'pos2_winkelsteg_abwicklung.dxf', dxf(g.stegOutline(), g.stegBendLines(), [g.STEG.L / 2, 0]));
console.log('written', out);
