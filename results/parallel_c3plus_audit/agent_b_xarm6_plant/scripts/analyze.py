#!/usr/bin/env python3
"""Agent B plant/execution-fidelity audit for shelf_gap draws.

Data notes / schema honesty:
- No direct pusher-object gap or contact boolean exists in any log. gap_state_trace.csv
  and obstacle_lcs_contacts.csv are OBJECT-vs-OBSTACLE signals only.
- Actual EE position is recovered from candidate_ranking_costs.csv candidate_id==0
  rows (the "current location" candidate, verified smooth ~30-70 Hz trajectory);
  controller_cycle sel_ee_* is the SELECTED SAMPLE (teleports), not the plant EE.
- Contact is therefore a PROXY: EE-to-object-center planar distance d(t) with a
  per-population calibrated contact distance d_c = 5th pct of d during object
  motion (speed > 1 cm/s) + 2 mm. Reported alongside raw numbers.
"""
import json, csv, math, os, sys
from bisect import bisect_left, bisect_right

OUT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant/results/parallel_c3plus_audit/agent_b_xarm6_plant"

POPS = {
    "cmp_full": "/root/push_anything_ADMM/results/obstacle_pushing_comparison_2026-09-06/shelf_gap/4_route_guidance_full_stack",
    "cmp_base": "/root/push_anything_ADMM/results/obstacle_pushing_comparison_2026-09-06/shelf_gap/1_baseline_no_route_guidance",
    "fixval": "/root/push_anything_ADMM/results/c3plus_p5_fix_validation/shelf_gap_fix/plain",
}

def yaw_of(q):
    qw,qx,qy,qz = q
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def load_state(d):
    T=[]; X=[]; Y=[]; YAW=[]; PE=[]
    with open(os.path.join(d,"state_trace.jsonl")) as f:
        for line in f:
            o=json.loads(line)
            T.append(o["t"]); q=o["obj"]
            X.append(q[4]); Y.append(q[5]); YAW.append(yaw_of(q[0:4])); PE.append(o["pos_err"])
    return T,X,Y,YAW,PE

def load_ee(d):
    T=[]; EX=[]; EY=[]
    sel = {}  # event_id -> (time, cand_id)
    with open(os.path.join(d,"candidate_ranking_costs.csv")) as f:
        for row in csv.DictReader(f):
            if row["candidate_id"]=="0":
                T.append(float(row["time"])); EX.append(float(row["ee_x"])); EY.append(float(row["ee_y"]))
            if row.get("selected")=="1":
                sel[int(row["event_id"])] = (float(row["time"]), int(row["candidate_id"]))
    return T,EX,EY,sel

def interp(ts, vs, t):
    i = bisect_left(ts, t)
    if i<=0: return vs[0]
    if i>=len(ts): return vs[-1]
    t0,t1=ts[i-1],ts[i]
    if t1==t0: return vs[i]
    a=(t-t0)/(t1-t0)
    return vs[i-1]*(1-a)+vs[i]*a

def episodes(times, flags, gap_merge=0.15):
    eps=[]; start=None; last=None
    for t,fl in zip(times,flags):
        if fl:
            if start is None: start=t
            elif t-last>gap_merge:
                eps.append((start,last)); start=t
            last=t
        # keep episode open across brief False (handled by gap_merge on next True)
    if start is not None: eps.append((start,last))
    return [(a,b) for a,b in eps if b>a]

def median(v):
    v=sorted(v); n=len(v)
    if n==0: return float("nan")
    return v[n//2] if n%2 else 0.5*(v[n//2-1]+v[n//2])

def analyze_draw(pop, drawdir, name):
    T,X,Y,YAW,PE = load_state(drawdir)
    ET,EX,EY,sel = load_ee(drawdir)
    tend = T[-1]
    # EE-object distance at EE sample times
    D=[math.hypot(ex-interp(T,X,t), ey-interp(T,Y,t)) for t,ex,ey in zip(ET,EX,EY)]
    # object speed at EE times (central diff on state trace)
    def obj_speed(t):
        dt=0.25
        return math.hypot(interp(T,X,t+dt)-interp(T,X,t-dt), interp(T,Y,t+dt)-interp(T,Y,t-dt))/(2*dt)
    SPD=[obj_speed(t) for t in ET]
    moving=[d for d,s in zip(D,SPD) if s>0.01]
    if moving:
        m=sorted(moving); dc=m[max(0,int(0.05*len(m))-1)]+0.002
    else:
        dc=float("nan")
    contact=[d<=dc for d in D]
    # time-weighted contact fraction
    cf_num=0.0; tot=0.0
    for i in range(1,len(ET)):
        dt=ET[i]-ET[i-1]
        if dt<=0 or dt>1: continue
        tot+=dt
        if contact[i]: cf_num+=dt
    contact_fraction = cf_num/tot if tot else float("nan")
    eps = episodes(ET, contact)
    ep_dur=[b-a for a,b in eps]
    # productive windows: 0.5s, disp>5mm
    nwin=0; nprod=0; w=0.5; t=T[0]
    while t+w<=tend:
        dx=interp(T,X,t+w)-interp(T,X,t); dy=interp(T,Y,t+w)-interp(T,Y,t)
        nwin+=1
        if math.hypot(dx,dy)>0.005: nprod+=1
        t+=w
    prod_frac = nprod/nwin if nwin else float("nan")

    # plateau onset from running-min of pos_err? spec: southward progress. Use running min of y.
    runmin=[]; m=Y[0]
    for y in Y:
        m=min(m,y); runmin.append(m)
    final=runmin[-1]
    onset=tend
    for t,rm in zip(T,runmin):
        if rm<=final+0.01:
            onset=t; break
    # per-episode object displacement
    def ep_disp(a,b):
        return math.hypot(interp(T,X,b)-interp(T,X,a), interp(T,Y,b)-interp(T,Y,a))
    ep_d=[(a,b,ep_disp(a,b)) for a,b in eps]

    def phase_stats(t0,t1):
        idx=[i for i,t in enumerate(ET) if t0<=t<t1]
        if not idx: return dict(cf=float("nan"),mean_d=float("nan"))
        cf=sum(1 for i in idx if contact[i])/len(idx)
        return dict(cf=cf, mean_d=sum(D[i] for i in idx)/len(idx))
    pre=phase_stats(0,onset); post=phase_stats(onset,tend)
    pre_ep=[d for a,b,d in ep_d if b<=onset]; post_ep=[d for a,b,d in ep_d if a>=onset]

    # yaw before/after
    yaw_pre=[yv for t,yv in zip(T,YAW) if t<onset]; yaw_post=[yv for t,yv in zip(T,YAW) if t>=onset]
    # southward rate pre plateau
    south = (Y[0]-final)
    # obstacle lambda before/after
    lam_pre=[]; lam_post=[]
    with open(os.path.join(drawdir,"obstacle_lcs_contacts.csv")) as f:
        for row in csv.DictReader(f):
            if row["active"]=="1":
                lv=float(row["lambda_obs"]); tt=float(row["time"])
                (lam_pre if tt<onset else lam_post).append(lv)
    # mode fraction after plateau + reposition counts (controller_cycle)
    c3_pre=c3_post=n_pre=n_post=0
    with open(os.path.join(drawdir,"controller_cycle_costs.csv")) as f:
        for row in csv.DictReader(f):
            tt=float(row["time"]); c3=row["is_c3_mode"]=="1"
            if tt<onset: n_pre+=1; c3_pre+=c3
            else: n_post+=1; c3_post+=c3
    # EE path length + mean EE-obj distance after plateau
    path_post=sum(math.hypot(EX[i]-EX[i-1],EY[i]-EY[i-1]) for i in range(1,len(ET)) if ET[i]>=onset)
    dur_post=max(1e-9,tend-onset)

    # prediction vs execution (route progress of the SELECTED candidate)
    rows=[]
    with open(os.path.join(drawdir,"candidate_route_progress.csv")) as f:
        for row in csv.DictReader(f):
            e=int(row["event_id"]); c=int(row["candidate_id"])
            if e in sel and sel[e][1]==c:
                t=sel[e][0]
                ox,oy=interp(T,X,t),interp(T,Y,t)
                px,py=float(row["pred_x"])-ox, float(row["pred_y"])-oy
                pn=math.hypot(px,py)
                rx,ry=interp(T,X,t+0.5)-ox, interp(T,Y,t+0.5)-oy
                if pn>1e-4:
                    realized=(rx*px+ry*py)/pn
                    d_at=interp(ET,D,t)
                    rows.append(dict(t=t,pred=pn,real=realized,rho=realized/pn,in_contact=d_at<=dc))
    rho_all=median([r["rho"] for r in rows])
    rho_c=median([r["rho"] for r in rows if r["in_contact"]])
    rho_nc=median([r["rho"] for r in rows if not r["in_contact"]])

    # osc/sim/planner anomaly greps
    import re
    anomalies={}
    for lg in ("osc.log","sim.log","planner.log"):
        p=os.path.join(drawdir,lg); c=0
        if os.path.exists(p):
            with open(p,errors="replace") as f:
                for line in f:
                    if re.search(r"warn|limit|clamp|saturat|abort|error|fail",line,re.I):
                        c+=1
        anomalies[lg]=c
    repos_count=0
    with open(os.path.join(drawdir,"planner.log"),errors="replace") as f:
        for line in f:
            if line.startswith("Repositioning"): repos_count+=1

    return dict(
        name=name, tend=tend, dc=dc, contact_fraction=contact_fraction,
        n_episodes=len(eps), median_episode_s=median(ep_dur),
        productive_window_fraction=prod_frac, plateau_onset=onset,
        south_total=south, y0=Y[0], y_final_min=final, final_pos_err=PE[-1],
        pre_cf=pre["cf"], post_cf=post["cf"], pre_mean_d=pre["mean_d"], post_mean_d=post["mean_d"],
        pre_ep_disp=median(pre_ep), post_ep_disp=median(post_ep),
        lam_pre=(sum(lam_pre)/len(lam_pre) if lam_pre else 0.0),
        lam_post=(sum(lam_post)/len(lam_post) if lam_post else 0.0),
        lam_max_post=(max(lam_post) if lam_post else 0.0),
        yaw_pre_med=median(yaw_pre), yaw_post_med=median(yaw_post),
        c3_frac_pre=(c3_pre/n_pre if n_pre else float("nan")),
        c3_frac_post=(c3_post/n_post if n_post else float("nan")),
        ee_path_rate_post=path_post/dur_post, repos_count=repos_count,
        n_cycles=len(rows), median_pred=median([r["pred"] for r in rows]),
        median_real=median([r["real"] for r in rows]),
        rho_median=rho_all, rho_in_contact=rho_c, rho_no_contact=rho_nc,
        n_rho_contact=sum(1 for r in rows if r["in_contact"]),
        anomalies=anomalies,
        # timelines for divergence plots
        _T=T,_Y=Y,_YAW=YAW,_ET=ET,_D=D,_contact=contact,
    )

results=[]
for pop,root in POPS.items():
    for dr in ("draw0","draw1","draw2"):
        d=os.path.join(root,dr)
        if not os.path.isdir(d): continue
        print("analyzing",pop,dr,file=sys.stderr)
        results.append((pop,dr,analyze_draw(pop,d,f"{pop}/{dr}")))

os.makedirs(OUT,exist_ok=True)
with open(os.path.join(OUT,"contact_acquisition.csv"),"w") as f:
    w=csv.writer(f)
    w.writerow(["population","draw","sim_t_end","d_contact_proxy_m","contact_fraction","n_episodes","median_episode_s","productive_window_fraction","final_pos_err","south_progress_m"])
    for pop,dr,r in results:
        w.writerow([pop,dr,f"{r['tend']:.1f}",f"{r['dc']:.4f}",f"{r['contact_fraction']:.3f}",r['n_episodes'],f"{r['median_episode_s']:.2f}",f"{r['productive_window_fraction']:.3f}",f"{r['final_pos_err']:.4f}",f"{r['south_total']:.4f}"])

with open(os.path.join(OUT,"prediction_vs_execution.csv"),"w") as f:
    w=csv.writer(f)
    w.writerow(["population","draw","n_cycles","median_predicted_m","median_realized_m","rho_median","rho_in_contact","n_in_contact","rho_no_contact"])
    for pop,dr,r in results:
        w.writerow([pop,dr,r['n_cycles'],f"{r['median_pred']:.4f}",f"{r['median_real']:.4f}",f"{r['rho_median']:.3f}",f"{r['rho_in_contact']:.3f}",r['n_rho_contact'],f"{r['rho_no_contact']:.3f}"])

# per-draw phase table + divergence timeline (30s bins of contact frac + south rate + yaw)
with open(os.path.join(OUT,"phase_table.csv"),"w") as f:
    w=csv.writer(f)
    w.writerow(["population","draw","plateau_onset_s","pre_contact_frac","post_contact_frac","pre_mean_eeobj_d","post_mean_eeobj_d","pre_ep_disp_med_m","post_ep_disp_med_m","lam_obs_pre_mean","lam_obs_post_mean","lam_obs_post_max","yaw_pre_med","yaw_post_med","c3_frac_pre","c3_frac_post","ee_path_rate_post_mps","repos_events","anom_osc","anom_sim","anom_planner"])
    for pop,dr,r in results:
        w.writerow([pop,dr,f"{r['plateau_onset']:.1f}",f"{r['pre_cf']:.3f}",f"{r['post_cf']:.3f}",f"{r['pre_mean_d']:.3f}",f"{r['post_mean_d']:.3f}",f"{r['pre_ep_disp']:.4f}",f"{r['post_ep_disp']:.4f}",f"{r['lam_pre']:.4f}",f"{r['lam_post']:.4f}",f"{r['lam_max_post']:.4f}",f"{r['yaw_pre_med']:.3f}",f"{r['yaw_post_med']:.3f}",f"{r['c3_frac_pre']:.3f}",f"{r['c3_frac_post']:.3f}",f"{r['ee_path_rate_post']:.4f}",r['repos_count'],r['anomalies']['osc.log'],r['anomalies']['sim.log'],r['anomalies']['planner.log']])

# 10s-bin timeline for fixval success-vs-plateau divergence
with open(os.path.join(OUT,"timeline_10s_bins.csv"),"w") as f:
    w=csv.writer(f)
    w.writerow(["population","draw","t_bin_start","obj_y","obj_yaw","contact_frac_bin","south_rate_mm_per_s"])
    for pop,dr,r in results:
        T,Y,YAW,ET,contact = r["_T"],r["_Y"],r["_YAW"],r["_ET"],r["_contact"]
        t=0.0
        while t< r["tend"]:
            idx=[i for i,tt in enumerate(ET) if t<=tt<t+10]
            cfb=sum(1 for i in idx if contact[i])/len(idx) if idx else float("nan")
            y0=interp(T,Y,t); y1=interp(T,Y,min(t+10,r["tend"]))
            yawb=interp(T,YAW,t+5)
            w.writerow([pop,dr,f"{t:.0f}",f"{y0:.4f}",f"{yawb:.3f}",f"{cfb:.3f}" if idx else "",f"{(y0-y1)*100:.2f}"])
            t+=10
print("done")
