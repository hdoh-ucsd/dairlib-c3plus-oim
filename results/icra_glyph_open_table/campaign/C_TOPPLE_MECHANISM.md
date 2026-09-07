# C glyph "topple" mechanism — smoke-trial forensics (2026-09-07)

Verdict: **all 3 C smoke failures are TABLE-EDGE FALLS, not intrinsic tip-overs.**

Scene fact: open_table ground = `ground.urdf` box 5 x 0.91 x 0.1 → usable surface ends at
y = ±0.455 m. The protocol goal (0.50, −0.40) sits **5.5 cm from the edge**. In every trial the
letter's z sagged over the lip 1–2 s BEFORE roll/pitch crossed 0.3 rad — the attitude-based
SAMPLING_C3_TOPPLE_GUARD fires inherently post-lip.

Proximate driver (trials 2–3, HIGH confidence): **EE hook-drag**. Arm posture freezes
(q1,q2,q4 const) and only the base joint sweeps; object rides at constant radius ~0.68 m with
angular rate −0.045 rad/s == base-joint rate −0.047 rad/s — the 0.03 m EE peg is captured
in/against the C's ~60° open mouth (empty sector 1.22–2.27 rad body frame, outer r 0.079 m)
and carts the letter along an arc (trial 3: ~200° sweep, 1.3 m, over the OPPOSITE edge y=+0.455).

Per-trial: t1 topple @88.2 s at (0.131,−0.449) after edge-sliding, fell back on table, planner
died 1290 s later on the EE workspace assert; t2 @47.7 s at (0.502,−0.460) — blew through the
goal (pos_err bottomed 0.035 m) without decelerating, over the lip, guard trip, freefall
(pos_err 164 m); t3 @115.4 s at (−0.512,+0.456) after the long hook-drag, freefall (190 m).

Why C alone: extents/support widths comparable to I/R/A (NOT statically tippier — matches the
non-toppling microtests). Unique C factors: (1) mouth capture — only glyph where the pusher can
be geometrically hooked; (2) fully convex curved outer rim → tangential slide, push → spin+coast;
(3) 0.05 kg at μ=0.3 ≈ 0.15 N ground friction → coasts through the goal.

Implication (NO changes made — frozen baseline): failure class = scene-margin edge fall +
unmodeled hook capture. Fix surface, if authorized later: object-position workspace bounds
(only the EE is checked today) and/or goal-to-edge margin. Full-campaign C trials quantify rate.

Evidence: smoke/c/trial{1,2,3}/state_trace.jsonl (z-dip precedes attitude in all 3; topple
coordinates match the 0.455 m half-width exactly).
