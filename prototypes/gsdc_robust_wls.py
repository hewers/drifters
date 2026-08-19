import sys
D=sys.argv[1] if len(sys.argv)>1 else 'gsdc2023'
import csv, math, collections, numpy as np
C=299792458.0; WE=7.2921151467e-5
def lla2ecef(lat,lon,h):
    a=6378137.0; f=1/298.257223563; e2=f*(2-f)
    la,lo=math.radians(lat),math.radians(lon); N=a/math.sqrt(1-e2*math.sin(la)**2)
    return np.array([(N+h)*math.cos(la)*math.cos(lo),(N+h)*math.cos(la)*math.sin(lo),(N*(1-e2)+h)*math.sin(la)])
def ecef2enu_err(p,t):
    lat=math.atan2(t[2],math.hypot(t[0],t[1])); lon=math.atan2(t[1],t[0])
    d=p-t; sl,cl,so,co=math.sin(lat),math.cos(lat),math.sin(lon),math.cos(lon)
    e=-so*d[0]+co*d[1]; n=-sl*co*d[0]-sl*so*d[1]+cl*d[2]; u=cl*co*d[0]+cl*so*d[1]+sl*d[2]
    return e,n,u

T={}
for r in csv.DictReader(open('datasets/%s/ground_truth.csv'%D)):
    T[int(r['UnixTimeMillis'])]=lla2ecef(float(r['LatitudeDegrees']),float(r['LongitudeDegrees']),float(r['AltitudeMeters']))
tk=sorted(T)
def truth_at(ms):
    i=min(range(len(tk)),key=lambda k:abs(tk[k]-ms)); return T[tk[i]] if abs(tk[i]-ms)<600 else None

ep=collections.defaultdict(list); wls={}
for r in csv.DictReader(open('datasets/%s/device_gnss.csv'%D)):
    try:
        st=int(r['State'])
        if not (st&0x1) or not (st&0x8): continue
        ms=int(r['utcTimeMillis'])
        pr=float(r['RawPseudorangeMeters'])+float(r['SvClockBiasMeters'])-float(r['IonosphericDelayMeters'])-float(r['TroposphericDelayMeters'])
        isb=float(r.get('FullInterSignalBiasNanos') or 0)*C*1e-9
        sv=np.array([float(r['SvPositionXEcefMeters']),float(r['SvPositionYEcefMeters']),float(r['SvPositionZEcefMeters'])])
        ep[ms].append((int(r['ConstellationType']),pr-isb,sv,float(r['Cn0DbHz']),float(r['SvElevationDegrees'])))
        wls[ms]=np.array([float(r['WlsPositionXEcefMeters']),float(r['WlsPositionYEcefMeters']),float(r['WlsPositionZEcefMeters'])])
    except (ValueError,KeyError,TypeError): pass

def solve(obs, x0, sagnac=True, robust=True, elw=True, huber=1.5):
    cons=sorted({o[0] for o in obs})
    if len(obs) < 3+len(cons): return None
    ci={c:i for i,c in enumerate(cons)}; nx=3+len(cons)
    x=np.zeros(nx); x[:3]=x0
    for it in range(12):
        A=np.zeros((len(obs),nx)); res=np.zeros(len(obs)); sig=np.zeros(len(obs))
        p=x[:3]
        for k,(con,pr,sv,cn0,el) in enumerate(obs):
            s=sv.copy()
            if sagnac:
                tau=np.linalg.norm(s-p)/C; th=WE*tau
                s=np.array([ s[0]*math.cos(th)+s[1]*math.sin(th),
                            -s[0]*math.sin(th)+s[1]*math.cos(th), s[2]])
            d=p-s; rng=np.linalg.norm(d); u=d/rng
            A[k,:3]=u; A[k,3+ci[con]]=1.0
            res[k]=pr-(rng+x[3+ci[con]])
            sig[k]=(0.6+8.0/max(math.sin(math.radians(max(el,3.0))),0.05)) if elw else 5.0
        w=1.0/sig**2
        if robust:
            s0=1.4826*np.median(np.abs(res-np.median(res)))+1e-6
            z=np.abs(res/ s0)
            w=w*np.where(z<=huber,1.0,huber/np.maximum(z,1e-9))
        W=np.sqrt(w)
        try: dx=np.linalg.lstsq(A*W[:,None],res*W,rcond=None)[0]
        except np.linalg.LinAlgError: return None
        x=x+dx
        if np.linalg.norm(dx[:3])<1e-3: break
    return x[:3]

def score(**kw):
    eh=[];ev=[]
    x0=None
    for ms in sorted(ep):
        t=truth_at(ms)
        if t is None: continue
        if x0 is None: x0=wls[ms]
        p=solve(ep[ms], x0, **kw)
        if p is None: continue
        x0=p
        e,n,u=ecef2enu_err(p,t); eh.append(math.hypot(e,n)); ev.append(abs(u))
    a=np.array(eh); v=np.array(ev)
    return len(a), math.sqrt((a**2).mean()), math.sqrt((v**2).mean()), np.percentile(a,95)

# Google's own WLS as the baseline
bh=[];bv=[]
for ms in sorted(ep):
    t=truth_at(ms)
    if t is None: continue
    e,n,u=ecef2enu_err(wls[ms],t); bh.append(math.hypot(e,n)); bv.append(abs(u))
bh=np.array(bh);bv=np.array(bv)
print(f"{'solver':<44} {'n':>5} {'horiz RMS':>10} {'vert RMS':>9} {'h p95':>8}")
print(f"{'Google WlsPosition (current baseline)':<44} {len(bh):5d} {math.sqrt((bh**2).mean()):10.3f} {math.sqrt((bv**2).mean()):9.3f} {np.percentile(bh,95):8.2f}")
for lbl,kw in (("own solve, uniform weights, no robust",dict(elw=False,robust=False)),
               ("  + elevation weighting",dict(elw=True,robust=False)),
               ("  + robust IRLS (Huber)",dict(elw=True,robust=True)),
               ("  + elev + robust, no Sagnac",dict(elw=True,robust=True,sagnac=False))):
    n,h,v,p95=score(**kw)
    print(f"{lbl:<44} {n:5d} {h:10.3f} {v:9.3f} {p95:8.2f}")
