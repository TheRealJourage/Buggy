// Exhaust sizing formulas (sources in Vault Buggy/Abgasanlage.md, .research-exhaust-claude-design.md)
export const V_CYL_CC = 2792 / 6;
const IN = 25.4;

// Bell: primary length [in] = 850 * ED / rpm - 3, ED = 180 + exhaust opening BBDC
export const bellLength = (rpm, ed) => (850 * ed / rpm - 3) * IN;
export const bellRpm = (lengthMm, ed) => 850 * ed / (lengthMm / IN + 3);

// stainlessheaders/Vizard: area [in^2] = rpm / 88200 * cylinder volume [in^3]
const cid = (cc) => cc / 16.387;
export function primaryIdForRpm(rpm, cc = V_CYL_CC) {
  const a = rpm / 88200 * cid(cc);
  return 2 * Math.sqrt(a / Math.PI) * IN;
}
export function rpmForPrimaryId(idMm, cc = V_CYL_CC) {
  const a = Math.PI * (idMm / IN / 2) ** 2;
  return a * 88200 / cid(cc);
}

export const area = (idMm) => Math.PI * idMm * idMm / 4;
// diameter with the same area as n pipes of idMm
export const equalAreaId = (idMm, n) => idMm * Math.sqrt(n);
// ID of each of n pipes that together have the area of one pipe idMm
export const splitId = (idMm, n) => idMm / Math.sqrt(n);

// firing order 1-5-3-6-2-4, 120 deg apart: crank angle of each cylinder's firing
export const FIRING_ORDER = [1, 5, 3, 6, 2, 4];
export function firingGaps(group) {
  const ang = group.map((c) => FIRING_ORDER.indexOf(c) * 120).sort((a, b) => a - b);
  return ang.map((a, i) => ((i + 1 < ang.length ? ang[i + 1] : ang[0] + 720) - a));
}
