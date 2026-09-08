import pandas as pd, numpy as np, glob, itertools, os
runs=['icra_sign/pair01','open_task/pair03','shelf_gap/pair02','single_obstacle/pair01','slalom/pair03','ycb_clutter/pair04']
logc3={'icra_sign/pair01':44,'open_task/pair03':22,'shelf_gap/pair02':97,'single_obstacle/pair01':142,'slalom/pair03':115,'ycb_clutter/pair04':80}
base=os.path.join(os.path.dirname(os.path.abspath(__file__)),'../xarm6_c3plus_scene_smoke/runs/')
dfs={r:pd.read_csv(glob.glob(base+r+'/xarm6*_metrics.csv')[0]) for r in runs}
def seg(df,gapc3,sc3,srep,minep,merge):
    gap=df['pusher_object_gap'].to_numpy()
    spd=(np.hypot(df['tip_x'].diff(),df['tip_y'].diff())/df['sim_time'].diff()).rolling(11,center=True,min_periods=1).median().to_numpy()
    lab=np.zeros(len(gap),bool); inc=False
    for i in range(len(gap)):
        g,v=gap[i],spd[i]; near=np.isfinite(g) and g<gapc3
        inc = (near or not(np.isfinite(v) and v>srep)) if inc else (near or (np.isfinite(v) and v<sc3))
        lab[i]=inc
    eps=[];i=0;n=len(lab)
    while i<n:
        if lab[i]:
            j=i
            while j+1<n and lab[j+1]: j+=1
            eps.append([i,j]); i=j+1
        else: i+=1
    m=[]
    for e in eps:
        if m and e[0]-m[-1][1]<=merge: m[-1][1]=e[1]
        else: m.append(e)
    return [e for e in m if e[1]-e[0]+1>=minep]
for sc3,srep,minep,merge in itertools.product([0.015,0.02],[0.03,0.04],[30,60],[15,40]):
    out=[]
    for r in runs:
        n=len(seg(dfs[r],0.010,sc3,srep,minep,merge))
        out.append(f"{r.split('/')[0][:6]}:{n}/{logc3[r]}")
    print(sc3,srep,minep,merge,' '.join(out))
