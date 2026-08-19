"""Grid-search the robust solver on ONE trace, then report the holdout."""
import sys, csv, math, collections, itertools, numpy as np
C=299792458.0; WE=7.2921151467e-5

def lla2ecef(lat,lon,h):
    a=6378137.0; f=1/298.257223563; e2=f*(2-f)
    la,lo=math.radians(lat),math.radians(lon); N=a/math.sqrt(1-e2*math.sin(la)**2)
    return np.array([(N+h)*math.cos(la)*math.cos(lo),(N+h)*math.cos(la)*math.sin(lo),(N*(1-e2)+h)*math.sin(la)])

def load(d):
    T={}
    for r in csv.DictReader(open(f'datasets/{d}/ground_truth.csv')):
        T[int(r['UnixTimeMillis'])]=lla2ecef(float(r['LatitudeDegrees']),float(r['LongitudeDegrees']),float(r['AltitudeMeters']))
    tk=sorted(T)
    ep=collections.defaultdict(list); wls={}
    for r in csv.DictReader(open(f'datasets/{d}/device_gnss.csv')):
        try:
            st=int(r['State'])
            if not (st&0x1) or not (st&0x8): continue
            ms=int(r['utcTimeMillis'])
            pr=float(r['RawPseudorangeMeters'])+float(r['SvClockBiasMeters'])\
               -float(r['IonosphericDelayMeters'])-float(r['TroposphericDelayMeters'])\
               -float(r.get('FullInterSignalBiasNanos') or 0)*C*1e-9
            sv=np.array([float(r['SvPositionXEcefMeters']),float(r['SvPositionYEcefMeters']),float(r['SvPositionZEcefMeters'])])
            mp=int(r.get('MultipathIndicator') or 0)
            ep[ms].append((int(r['ConstellationType']),pr,sv,float(r['Cn0DbHz']),float(r['SvElevationDegrees']),mp))
            wls[ms]=np.array([float(r['WlsPositionXEcefMeters']),float(r['WlsPositionYEcefMeters']),float(r['WlsPositionZEcefMeters'])])
        except (ValueError,KeyError,TypeError): pass
    keep=[]
    for ms in sorted(ep):
        i=min(range(len(tk)),key=lambda k:abs(tk[k]-ms))
        if abs(tk[i]-ms)<600: keep.append((ms,ep[ms],T[tk[i]],wls[ms]))
    return keep

def enu(p,t):
    lat=math.atan2(t[2],math.hypot(t[0],t[1])); lon=math.atan2(t[1],t[0]); d=p-t
    sl,cl,so,co=math.sin(lat),math.cos(lat),math.sin(lon),math.cos(lon)
    return (-so*d[0]+co*d[1], -sl*co*d[0]-sl*so*d[1]+cl*d[2], cl*co*d[0]+cl*so*d[1]+sl*d[2])

def solve(obs,x0,P):
    obs=[o for o in obs if o[4]>=P['mask'] and o[3]>=P['cn0min']]
    cons=sorted({o[0] for o in obs})
    if len(obs)<4+len(cons): return None
    ci={c:i for i,c in enumerate(cons)}; nx=3+len(cons)
    x=np.zeros(nx); x[:3]=x0
    n=len(obs)
    sv=np.array([o[2] for o in obs]); prm=np.array([o[1] for o in obs])
    el=np.array([max(o[4],3.0) for o in obs]); cn0=np.array([o[3] for o in obs])
    cidx=np.array([3+ci[o[0]] for o in obs])
    sig=P['a']+P['b']/np.sin(np.radians(el))+P['c']*10**(-(cn0-P['cn0ref'])/20.0)
    w0=1.0/sig**2
    for _ in range(10):
        p=x[:3]
        tau=np.linalg.norm(sv-p,axis=1)/C; th=WE*tau
        sx=sv[:,0]*np.cos(th)+sv[:,1]*np.sin(th); sy=-sv[:,0]*np.sin(th)+sv[:,1]*np.cos(th)
        s=np.column_stack([sx,sy,sv[:,2]])
        d=p-s; rng=np.linalg.norm(d,axis=1); u=d/rng[:,None]
        A=np.zeros((n,nx)); A[:,:3]=u; A[np.arange(n),cidx]=1.0
        res=prm-(rng+x[cidx])
        s0=1.4826*np.median(np.abs(res-np.median(res)))+1e-6
        z=np.abs(res)/s0
        if P['cost']=='huber': rw=np.where(z<=P['k'],1.0,P['k']/np.maximum(z,1e-9))
        else: rw=np.where(z<=P['k'],(1-(z/P['k'])**2)**2,0.0)
        w=w0*rw
        W=np.sqrt(w)
        try: dx=np.linalg.lstsq(A*W[:,None],res*W,rcond=None)[0]
        except np.linalg.LinAlgError: return None
        x=x+dx
        if np.linalg.norm(dx[:3])<1e-3: break
    return x[:3]

def score(data,P):
    h=[];v=[];x0=None
    for ms,obs,t,w in data:
        if x0 is None: x0=w
        p=solve(obs,x0,P)
        if p is None: p=w
        else: x0=p
        e,nn,uu=enu(p,t); h.append(math.hypot(e,nn)); v.append(abs(uu))
    h=np.array(h);v=np.array(v)
    return math.sqrt((h**2).mean()), math.sqrt((v**2).mean()), np.percentile(h,95)

BASE=dict(a=0.6,b=8.0,c=0.0,cn0ref=40.0,k=1.5,cost='huber',mask=0.0,cn0min=0.0)
train=load('gsdc2023')
print("baseline (untuned):  h=%.3f v=%.3f p95=%.2f"%score(train,BASE))
best=(1e9,None)
grid=list(itertools.product([0.3,0.6,1.5],[3.0,8.0,16.0],[0.0,4.0,12.0],[1.0,1.5,2.5],['huber','tukey'],[0.0,10.0],[0.0,20.0]))
for a,b,c,k,cost,mask,cn0min in grid:
    P=dict(BASE,a=a,b=b,c=c,k=k if cost=='huber' else k*2.5,cost=cost,mask=mask,cn0min=cn0min)
    h,v,p95=score(train,P)
    obj=math.sqrt(h*h+0.25*v*v)
    if obj<best[0]: best=(obj,P,h,v,p95)
print("grid of %d evaluated on trace A only"%len(grid))
_,P,h,v,p95=best
print("best on A: a=%.1f b=%.1f c=%.1f k=%.2f %s mask=%.0f cn0min=%.0f  ->  h=%.3f v=%.3f p95=%.2f"
      %(P['a'],P['b'],P['c'],P['k'],P['cost'],P['mask'],P['cn0min'],h,v,p95))
print()
print(f"{'trace':<14}{'Google h/v':>18}{'untuned h/v':>18}{'TUNED h/v':>18}")
for d in ('gsdc2023','gsdc2023-b','gsdc2023-c','gsdc2023-d'):
    data=train if d=='gsdc2023' else load(d)
    gh=np.array([math.hypot(*enu(w,t)[:2]) for _,_,t,w in data])
    gv=np.array([abs(enu(w,t)[2]) for _,_,t,w in data])
    ub=score(data,BASE); tn=score(data,P)
    tag=' (fitted)' if d=='gsdc2023' else ''
    print(f"{d:<14}{math.sqrt((gh**2).mean()):8.2f} /{math.sqrt((gv**2).mean()):7.2f}"
          f"{ub[0]:9.2f} /{ub[1]:7.2f}{tn[0]:9.2f} /{tn[1]:7.2f}{tag}")
