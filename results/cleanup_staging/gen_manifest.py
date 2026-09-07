#!/usr/bin/env python3
"""READ-ONLY classification pass for results cleanup. Generates manifest, dup list,
referenced-artifacts list, size report. Deletes nothing."""
import os, re, csv, hashlib, subprocess, sys

R = '/root/push_anything_ADMM/results'
WT = '/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant'
WTR = os.path.join(WT, 'results')
OUT = os.path.join(WTR, 'cleanup_staging')

def dsize(p):
    if os.path.isfile(p):
        return os.path.getsize(p)
    tot = 0
    for root, dirs, files in os.walk(p):
        for f in files:
            fp = os.path.join(root, f)
            try: tot += os.path.getsize(fp)
            except OSError: pass
    return tot

# ---- tracked files in worktree
tracked = set(subprocess.run(['git','-C',WT,'ls-files','results/'],capture_output=True,text=True).stdout.split())

IN_SCOPE_TOP = set()  # filled below

rows = []  # (path, decision, category, reason)
def add(rel, decision, category, reason, base=R):
    rows.append((os.path.join(base, rel), decision, category, reason))

top = sorted(os.listdir(R))

# explicit in-scope top-level names (dirs + loose oim receipt files)
inscope_pred = lambda n: (n.startswith('c3plus_') or n.startswith('oim_') or n.startswith('xarm6_')
    or n.startswith('panda_to_xarm6') or n.startswith('final_oim_c3plus')
    or n == 'full_sampling_geometric_release_progress_MOqEAE'
    or n.startswith('c3ab_') or n.startswith('icra_sign') or n.startswith('obstacle_pushing_comparison')
    or n.startswith('franka_oim') or n == 'parallel_c3plus_audit')

# ---- per-dir custom rules -------------------------------------------------
def expand(name):
    p = os.path.join(R, name)
    def sub(x): return os.path.join(name, x)
    if name in ('final_oim_c3plus_comparison','final_oim_c3plus_comparison_runs'):
        add(name,'KEEP','final_comparison','PROTECTED: canonical final OIM-vs-C3+ comparison (runs compaction is another agent\'s job)' if name.endswith('runs') else 'PROTECTED: canonical final comparison figures/metrics/provenance'); return
    if name in ('xarm6_environment_validation','xarm6_native_controller_validation'):
        add(name,'KEEP','validation','PROTECTED validation receipts (reports + CSVs)'); return
    if name == 'xarm6_oim_arc':
        add(name,'DELETE','gates_arc_runs','DELETE candidate: gates-arc runs; user already ordered D-mirror erasure as wrong outer loop; keep zero raw, write one invalidation note'); return
    if name == 'panda_to_xarm6_port_final_r6':
        add(name,'KEEP','acceptance_gate','PROTECTED: the 3/5 acceptance gate round'); return
    if re.fullmatch(r'panda_to_xarm6_final_probe[4-7]', name):
        why = {'4':'pre-fix chatter waveform (full-rate window)','5':'post-armature check','6':'v_des cancellation capture','7':'terminal-hold verify'}[name[-1]]
        add(name,'KEEP','rootcause_probe',f'PROTECTED probe: {why}'); return
    if name in ('panda_to_xarm6_final_probe','panda_to_xarm6_final_probe2','panda_to_xarm6_final_probe3'):
        add(name,'DELETE','superseded_probe','Superseded by probe4-7 (chatter/v_des root-cause probes)'); return
    if re.fullmatch(r'panda_to_xarm6_port_final_r[1-5]', name) or re.fullmatch(r'panda_to_xarm6_port_runs(_r[2-7])?', name):
        add(name,'KEEP_SUMMARY_ONLY','port_iteration_runs','Raw trials; ledgers + per-round summaries live in worktree results/panda_to_xarm6_*; one representative trace per bug extracted by another agent'); return
    if name == 'obstacle_pushing_comparison_2026-09-06':
        for f in sorted(os.listdir(p)):
            if f.endswith(('.md','.yaml')):
                add(sub(f),'KEEP','report','Report/manifest of the post-outer-loop-fix comparison')
            else:
                add(sub(f),'DELETE','raw_draws','Pre-fidelity raw draw dirs (frames); report+manifest retained')
        return
    if name == 'c3plus_lcs_contact_v2':
        for f in sorted(os.listdir(p)):
            if f.endswith(('.md','.yaml')):
                add(sub(f),'KEEP','report','Validation/stall reports + manifest')
            else:
                add(sub(f),'DELETE','raw_draws','Raw draws; reports retained')
        return
    if name == 'c3plus_variant_cost_probe':
        keep = {'C3PLUS_VARIANT_INVENTORY_AND_COST_PROBE_REPORT.md':'report','probe_manifest.yaml':'manifest','figures':'figures','variant_registry':'summary','repository_inventory':'summary'}
        for f in sorted(os.listdir(p)):
            if f in keep:
                add(sub(f),'KEEP',keep[f],'Report/summary artifact of variant cost probe')
            else:
                add(sub(f),'DELETE','probe_banks','Candidate banks / probe state dumps; report + summaries retained')
        return
    if name == 'c3plus_p5_fix_validation':
        add(sub('README.md'),'KEEP','report','P5 fix validation README')
        add(sub('shelf_gap_p5fix_tight_success.mp4'),'KEEP','video_receipt','Receipt video of first-ever tight shelf traversal')
        base = 'shelf_gap_fix/plain'
        for d in sorted(os.listdir(os.path.join(p,base))):
            if d == 'draw0':
                add(sub(f'{base}/draw0'),'KEEP','canonical_run','PROTECTED: canonical successful shelf traversal (tight 0.018m)')
            elif d == 'draw1':
                add(sub(f'{base}/draw1'),'KEEP','plateau_run','PROTECTED: one representative plateau draw')
            else:
                add(sub(f'{base}/{d}'),'DELETE','raw_draws','Redundant draw; draw0 (canonical) + draw1 (plateau) retained')
        add(sub('single_obstacle_fix'),'DELETE','raw_draws','Raw single_obstacle draws; behavior covered by README + obstacle comparison report')
        return
    if name.startswith('c3ab_'):
        if os.path.isfile(p):
            if name.endswith('.jsonl'):
                add(name,'KEEP','ledger','c3ab ledger jsonl (result records kept per brief)')
            else:
                add(name,'DELETE','raw_trials','c3ab loose raw artifact')
            return
        if name == 'c3ab_ablation_study':
            for f in sorted(os.listdir(p)):
                if f.endswith(('.md','.jsonl','.json','.yaml','.csv')):
                    add(sub(f),'KEEP','ledger','Ablation report/ledger')
                else:
                    add(sub(f),'DELETE','raw_trials','Raw trial dir; jsonl ledgers + report retained')
            return
        add(name,'DELETE','raw_trials','c3ab raw trial dir; the paired .jsonl ledger is kept')
        return
    if name in ('c3plus_cost_decomposition_t010','c3plus_collision_awareness_t010'):
        for f in sorted(os.listdir(p)):
            if f.endswith(('.md','.yaml','.json')):
                add(sub(f),'KEEP','report','Report/manifest/summary json')
            else:
                add(sub(f),'DELETE','raw_draws','Raw scene run dirs; summary retained')
        return
    if name == 'c3plus_baseline_t010':
        for f in sorted(os.listdir(p)):
            if f.endswith(('.md','.yaml')):
                add(sub(f),'KEEP','report','Baseline report/manifest')
            elif f == 'single_obstacle':
                add(sub('single_obstacle/seed_0/config_snapshot'),'KEEP','move_to_source_candidate','FLAG MOVE-to-source: SDF here is referenced by micro_push_test.py — relocate into the source tree, do not delete')
                for s in sorted(os.listdir(os.path.join(p,f))):
                    if s != 'seed_0':
                        add(sub(f'single_obstacle/{s}'),'DELETE','raw_draws','Raw baseline seed run')
                for s in sorted(os.listdir(os.path.join(p,'single_obstacle','seed_0'))):
                    if s != 'config_snapshot':
                        add(sub(f'single_obstacle/seed_0/{s}'),'DELETE','raw_draws','Raw baseline seed_0 artifacts (config_snapshot kept as MOVE candidate)')
            else:
                add(sub(f),'DELETE','raw_draws','Raw baseline scene runs; report retained')
        return
    if name == 'icra_sign_faithful_port':
        for f in sorted(os.listdir(p)):
            if f == 'runs':
                for rf in sorted(os.listdir(os.path.join(p,'runs'))):
                    if rf.endswith('.csv'):
                        add(sub(f'runs/{rf}'),'KEEP','summary_csv','ICRA-sign run summary CSV (active Agent D work)')
                    elif rf == 'v5_zfix':
                        add(sub(f'runs/{rf}'),'KEEP','representative_draw','Latest run version kept as representative (active work)')
                    else:
                        add(sub(f'runs/{rf}'),'KEEP_SUMMARY_ONLY','repeated_draws','Bulk repeated run versions (v1..v5); CSVs + latest kept; ACTIVE work — do not delete without Agent D sign-off')
            else:
                add(sub(f),'KEEP','icra_port_asset','PROTECTED: Agent D active ICRA work (reports/handoff/obstacles/scene/tests/validation/figures/manifest)')
        return
    if name == 'parallel_c3plus_audit':
        add(name,'KEEP','audit','Agent A/D audit artifacts; worktree copy of reports is protected — be generous, active arc'); return
    if name == 'franka_oim_arc':
        add(sub('franka_oim_trials_ledger.json'),'KEEP','ledger','Trials ledger json (ledgers protected)')
        for f in sorted(os.listdir(p)):
            if f != 'franka_oim_trials_ledger.json':
                add(sub(f),'DELETE','abandoned_arc','franka_oim_arc raw runs (DELETE candidate per cleanup brief)')
        return
    if name in ('c3plus_inner_qp_obstacle_t010',):
        for f in sorted(os.listdir(p)):
            if f.endswith(('.md','.json','.yaml','.png')):
                add(sub(f),'KEEP','report','Report/summary/calibration artifacts')
            else:
                add(sub(f),'DELETE','raw_draws','DELETE candidate per brief (raw run dirs)')
        return
    if name in ('full_sampling_geometric_release_progress_MOqEAE','oim_sampling_c3plus','c3plus_obstacle_lcs_fix'):
        add(name,'DELETE','superseded_arc','DELETE candidate per cleanup brief'); return
    if name == 'oim_xarm_full_20260829_seed0':
        add(name,'KEEP_SUMMARY_ONLY','early_cpp_receipt','2026-08-29 C++ full run receipt; console logs are the summary — raw step dumps compactable'); return
    if name.startswith('c3plus_route_') or name == 'c3plus_channel_route_v1' or name == 'c3plus_single_obstacle_failure_investigation':
        for f in sorted(os.listdir(p)):
            if f.endswith(('.md','.yaml','.json')):
                add(sub(f),'KEEP','report','Route/investigation report')
            else:
                add(sub(f),'KEEP_SUMMARY_ONLY','analysis_data','Analysis data behind a kept report; compact after report is verified self-contained')
        return
    # leftovers not named in the brief: small reports/receipts -> KEEP; else judgement
    if name in ('c3plus_port_vs_markdown_design','c3plus_nonpen_vs_qpcost_single'):
        add(name,'KEEP','report','Report/summary-only dir (small)'); return
    if name in ('c3plus_baseline_t01',):
        add(name,'DELETE','superseded_baseline','Superseded by c3plus_baseline_t010 (t010 is the referenced baseline tier)'); return
    if name in ('oim_c3plus','oim_c3plus_full_lcs','oim_c3plus_smoke'):
        add(name,'DELETE','early_smoke','Early OIM C3+ smoke/until-goal runs, superseded by final_oim_c3plus_comparison'); return
    if name in ('oim_cpp_contact_capsule_gate_20260831','oim_renderer_xarm_integrated_20260829','oim_renderer_xarm_smoke_20260829'):
        add(name,'KEEP_SUMMARY_ONLY','gate_receipt','Gate receipt (summary.json + mp4); step jsonl compactable'); return
    if os.path.isfile(p) and (name.startswith('oim_')):
        add(name,'KEEP','loose_receipt','Loose 2026-08-29 C++ port receipt (txt/mp4) cited in memory; keep'); return
    add(name,'KEEP_SUMMARY_ONLY','unclassified_in_scope','In-scope leftover not named in brief; conservative default')

# ---- build shared-results rows
for name in top:
    if inscope_pred(name):
        IN_SCOPE_TOP.add(name)
        expand(name)
    else:
        add(name,'DO_NOT_TOUCH','out_of_scope','Out of scope (push_t/jack/fig8/reference/other lineage)')

# ---- worktree results rows
for name in sorted(os.listdir(WTR)):
    if name == 'cleanup_staging':
        continue
    add(name,'KEEP','tracked_worktree','Tracked worktree summaries/reports (git-tracked)',base=WTR)

# ---- task 3: referenced artifacts ------------------------------------------
delete_paths = [p for p,d,_,_ in rows if d=='DELETE']
delete_tops = set()
for p,d,_,_ in rows:
    if d=='DELETE':
        delete_tops.add(p)

report_files = []
for base in [os.path.join(WTR,'final_oim_c3plus_comparison'), os.path.join(WTR,'panda_to_xarm6_final'),
             os.path.join(WTR,'parallel_c3plus_audit'), os.path.join(R,'final_oim_c3plus_comparison'),
             os.path.join(R,'final_oim_c3plus_comparison_runs')]:
    for root,dirs,files in os.walk(base):
        for f in files:
            if f.endswith(('.md','.yaml','.json')):
                report_files.append(os.path.join(root,f))
for f in os.listdir(WT):
    if f.endswith('.md'):
        report_files.append(os.path.join(WT,f))
# also repo-root reports of the main repo? restricted: worktree root only per brief.

path_re = re.compile(r'(?:/root/push_anything_ADMM/)?results/[A-Za-z0-9_./\-]+')
refrows = []
referenced_delete_tops = set()
referenced_any = set()
for rf in report_files:
    try:
        txt = open(rf, errors='replace').read()
    except Exception:
        continue
    for m in set(path_re.findall(txt)):
        rel = m.split('results/',1)[1].rstrip('.')
        ap = os.path.join(R, rel)
        exists = os.path.exists(ap)
        conflict = 'no'
        for dp in delete_tops:
            if ap == dp or ap.startswith(dp + '/'):
                conflict = 'yes'
                referenced_delete_tops.add(dp)
                break
        # track top-level referencing for manifest column
        referenced_any.add(os.path.join(R, rel.split('/')[0]))
        if conflict=='yes' or any(seg in rel for seg in IN_SCOPE_TOP if rel.startswith(seg)):
            refrows.append((rf.replace(WT,'<worktree>').replace(R,'<results>'), m, 'yes' if exists else 'no', conflict))

with open(os.path.join(OUT,'referenced_artifacts.csv'),'w',newline='') as f:
    w=csv.writer(f); w.writerow(['report','referenced_path','exists','decision_conflict'])
    for r in sorted(set(refrows)): w.writerow(r)

# ---- task 1 manifest -------------------------------------------------------
def reproducible(decision, cat):
    if cat in ('report','ledger','manifest','summary','summary_csv','figures','video_receipt'): return 'no'
    if decision in ('DELETE','KEEP_SUMMARY_ONLY'): return 'yes (re-run from pinned configs; sim runs are re-runnable)'
    return 'partial'

with open(os.path.join(OUT,'RESULTS_CLEANUP_MANIFEST.csv'),'w',newline='') as f:
    w=csv.writer(f)
    w.writerow(['path','size_bytes','tracked','category','referenced_by_final_report','reproducible','decision','reason'])
    totals={}
    for p,dec,cat,reason in rows:
        sz=dsize(p)
        totals[dec]=totals.get(dec,0)+sz
        tr='yes' if (p.startswith(WTR) and any(t.startswith('results/'+os.path.relpath(p,WTR)) for t in tracked)) else 'no'
        ref='yes' if (p in referenced_any or any(p==dp or p.startswith(dp+'/') for dp in referenced_delete_tops) or any(a==p or a.startswith(p+'/') for a in referenced_any)) else 'no'
        w.writerow([p,sz,tr,cat,ref,reproducible(dec,cat),dec,reason])
    print('TOTALS_BY_DECISION')
    for k,v in sorted(totals.items()):
        print(f'  {k}: {v/1e9:.2f} GB  ({v} bytes)')

# ---- task 4 size report ----------------------------------------------------
inscope_rows=[(p,dsize(p)) for p,dec,_,_ in rows if dec!='DO_NOT_TOUCH']
tot=sum(s for _,s in inscope_rows)
with open(os.path.join(OUT,'size_report_before.md'),'w') as f:
    f.write('# In-scope size report (before cleanup)\n\n')
    f.write(f'Total in-scope size: **{tot/1e9:.2f} GB** ({tot} bytes) across {len(inscope_rows)} manifest paths.\n\n')
    f.write('## Top 50 largest in-scope paths\n\n| size | path |\n|---|---|\n')
    for p,s in sorted(inscope_rows,key=lambda x:-x[1])[:50]:
        f.write(f'| {s/1e6:,.1f} MB | {p} |\n')
print('IN_SCOPE_TOTAL_GB', tot/1e9)
print('CONFLICT_DELETE_TOPS', sorted(referenced_delete_tops))
