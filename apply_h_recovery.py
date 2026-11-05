#!/usr/bin/env python3
"""Apply only Hall-FOC H recovery changes to an existing STM32 project.
Run from the CubeIDE project root, or supply its path as the first argument.
Preserves the user's existing T-mode (including manually edited currents/limits).
"""
from pathlib import Path
import sys, re, difflib
root = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path.cwd().resolve()
files = [root/'src'/'hw'/'driver'/'motor.c', root/'src'/'common'/'def.h']
for p in files:
    if not p.is_file():
        raise SystemExit(f'ERROR: No file: {p}. Run from your STM32CubeIDE project root.')
motor = files[0].read_text(encoding='utf-8')
defh = files[1].read_text(encoding='utf-8')
if 'HFOC_IQ_TARGET_A' not in motor or 'hall_foc.forced_selected' not in motor or 'HALL_ELEC_OFFSET' not in defh:
    raise SystemExit('ERROR: H mode or hall offset not found. Files NOT changed.')
# Validate original markers up front to prevent partial changes
hdefs = re.search(r'(?m)^#define HFOC_IQ_TARGET_A\s+[^\r\n]+\n#define HFOC_OC_LIMIT_A\s+[^\r\n]+\n#define HFOC_RAMP_SAMPLES\s+[^\r\n]+\n#define HFOC_TOTAL_SAMPLES\s+[^\r\n]+\n#define HFOC_TARGET_EDGES\s+[^\r\n]+',motor)
if not hdefs:
    raise SystemExit('ERROR: H constants not in expected layout; files unchanged.')
if '/* H: proven 2->6 baseline with staged current boost */' in motor:
    raise SystemExit('Already applied. No changes made.')
# Replace constants - JOC remains at original 1.9A
newdefs = '''/* H: proven 2->6 baseline with staged current boost */
#define HFOC_IQ_TARGET_A         1.70f
#define HFOC_BOOST_IQ_A          2.00f
#define HFOC_BOOST_START_SAMPLES 5000U  /* 250 ms */
#define HFOC_BOOST_RAMP_SAMPLES  1000U  /* 50 ms: 1.7 -> 2.0 A */
#define HFOC_OC_LIMIT_A          2.30f  /* H ONLY; software phase-current limit */
#define JFOC_OC_LIMIT_A          1.90f  /* J stays unchanged */
#define HFOC_RAMP_SAMPLES        3000U  /* 150 ms at 20 kHz */
#define HFOC_TOTAL_SAMPLES       13000U /* 650 ms */
#define HFOC_TARGET_EDGES        3U'''
motor=motor[:hdefs.start()]+newdefs+motor[hdefs.end():]
# Change H angle handling, leaving J unchanged
pattern=r'''(?ms)^    /\* H only: use the fixed Hall sector center instead of extrapolating\n.*?^        0\.0f, 2\.0f \* PI\);\n'''
newangle='''    /* Restore the Hall angle interpolation from the H run which
     * actually recorded 010 -> 110. Do not alter J startup. */
    theta_e = hallGetElectricalAngle();
'''
motor,n=re.subn(pattern,newangle,motor,count=1)
if n!=1:raise SystemExit('ERROR: expected H sector-center code not found; files unchanged.')
# Gradual boost after original H ramp to keep change controlled.
old='''      iq_ref = HFOC_IQ_TARGET_A * fraction;
'''
new='''      iq_ref = HFOC_IQ_TARGET_A * fraction;
      if (n >= HFOC_BOOST_START_SAMPLES)
      {
        float boost_frac = (float)(n - HFOC_BOOST_START_SAMPLES) /
                           (float)HFOC_BOOST_RAMP_SAMPLES;
        if (boost_frac > 1.0f) boost_frac = 1.0f;
        iq_ref += (HFOC_BOOST_IQ_A - HFOC_IQ_TARGET_A) * boost_frac;
      }
'''
if motor.count(old)!=1:raise SystemExit('ERROR: H Iq command not unique; files unchanged.')
motor=motor.replace(old,new,1)
# Hall H/J separate guard both in outer H update and motorCurrentLoop, preserve J threshold
old='''  if (a >= HFOC_OC_LIMIT_A || b >= HFOC_OC_LIMIT_A ||
      c >= HFOC_OC_LIMIT_A)
'''
new='''  const float hall_oc_limit = hall_foc.forced_selected ?
      JFOC_OC_LIMIT_A : HFOC_OC_LIMIT_A;
  if (a >= hall_oc_limit || b >= hall_oc_limit ||
      c >= hall_oc_limit)
'''
if motor.count(old)!=1:raise SystemExit('ERROR: Hall OC guard not as expected; files unchanged.')
motor=motor.replace(old,new,1)
old='''      hall_foc.selected ? HFOC_OC_LIMIT_A :
'''
new='''      hall_foc.selected ?
        (hall_foc.forced_selected ? JFOC_OC_LIMIT_A : HFOC_OC_LIMIT_A) :
'''
if motor.count(old)!=1:raise SystemExit('ERROR: motorCurrentLoop OC guard not as expected; files unchanged.')
motor=motor.replace(old,new,1)
# Edge count in two phases; do not change actual confirmed Hall counts/J logic
old='''  volatile uint32_t forward_edges, handoff_sample;
'''
new='''  volatile uint32_t forward_edges, handoff_sample;
  volatile uint32_t h_edges_before_boost, h_edges_after_boost;
'''
if motor.count(old)!=1:raise SystemExit('ERROR: H struct marker not found; files unchanged.')
motor=motor.replace(old,new,1)
old='''      hall_foc.edges++;
      hall_foc.last_hall = raw;
'''
new='''      hall_foc.edges++;
      if (!hall_foc.forced_selected)
      {
        if (hall_foc.samples < HFOC_BOOST_START_SAMPLES)
          hall_foc.h_edges_before_boost++;
        else
          hall_foc.h_edges_after_boost++;
      }
      hall_foc.last_hall = raw;
'''
if motor.count(old)!=1:raise SystemExit('ERROR: confirmed Hall edge marker not found; files unchanged.')
motor=motor.replace(old,new,1)
old='''    uartPrintf(_DEF_UART1, "Angle: Hall sector CENTER (no speed interpolation)\\r\\n");
'''
new='''    uartPrintf(_DEF_UART1, "Angle: Hall interpolated (original 2->6 H baseline)\\r\\n");
'''
if motor.count(old)!=1:raise SystemExit('ERROR: H UART angle label marker not found; files unchanged.')
motor=motor.replace(old,new,1)
old='''  if (hall_foc.forced_selected)
    uartPrintf(_DEF_UART1,
        "J stage %u (0 align, 1 forced, 2 blend, 3 Hall);'''
new='''  if (!hall_foc.forced_selected)
  {
    uartPrintf(_DEF_UART1,
        "H: Iq 1.7A -> 2.0A after 250ms (50ms ramp), SW phase OC 2.30A, max 650ms\\r\\n");
    uartPrintf(_DEF_UART1,
        "H edges before/after boost: %lu / %lu\\r\\n",
        (unsigned long)hall_foc.h_edges_before_boost,
        (unsigned long)hall_foc.h_edges_after_boost);
  }
  if (hall_foc.forced_selected)
    uartPrintf(_DEF_UART1,
        "J stage %u (0 align, 1 forced, 2 blend, 3 Hall);'''
if motor.count(old)!=1:raise SystemExit('ERROR: UART H print position not found; files unchanged.')
motor=motor.replace(old,new,1)
old='''      (long)(HFOC_OC_LIMIT_A*1000.0f),
      (unsigned long)(hall_foc.forced_selected ? 700U : 450U));
'''
new='''      (long)((hall_foc.forced_selected ? JFOC_OC_LIMIT_A : HFOC_OC_LIMIT_A)*1000.0f),
      (unsigned long)(hall_foc.forced_selected ? 700U : 650U));
'''
if motor.count(old)!=1:raise SystemExit('ERROR: H duration label not found; files unchanged.')
motor=motor.replace(old,new,1)
motor = motor.replace('Drive 450 ms max; stop after two stable Hall transitions.',
                      'Drive 650 ms max; stop after three stable Hall transitions.')
# Replace offset regardless of currently +60 or already 5.295
pattern=r'(?m)^#define HALL_ELEC_OFFSET\s+[^\r\n]+'
defh,n=re.subn(pattern,'#define HALL_ELEC_OFFSET                  5.295f /* baseline H: Hall 010 -> 110 once */',defh,count=1)
if n!=1:raise SystemExit('ERROR: Hall offset macro not found; files unchanged.')
# Write after all transformations succeeded
for p,updated in zip(files,[motor,defh]):
    backup=p.with_name(p.name+'.before_h_2to6.bak')
    if not backup.exists():backup.write_bytes(p.read_bytes())
    raw = p.read_bytes()
    line_end = b'\r\n' if b'\r\n' in raw else b'\n'
    p.write_bytes(updated.replace('\n', '\r\n' if line_end == b'\r\n' else '\n').encode('utf-8'))
print('OK: Updated only motor.c H-mode and def.h Hall offset. Other T changes preserved.')
print('H=1.7A  -> 2.0A after 250ms; H phase-OC=2.30A; H auto-stop=650ms; J OC stays 1.90A')
