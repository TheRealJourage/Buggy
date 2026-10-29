"""Convert a TunerStudio .mlg datalog (MLVLG v2) to TSV on stdout. Usage: python3 -I tools/mlg2tsv.py log.mlg > log.tsv"""
import sys,struct
d=open(sys.argv[1],'rb').read()
fmt,ts,infoStart,dataBegin,recLen,nFields=struct.unpack('>hiiihh',d[6:24])
off=24; fields=[]
for i in range(nFields):
    t=d[off]; name=d[off+1:off+35].split(b'\0')[0].decode('latin1')
    scale,trans=struct.unpack('>ff',d[off+46:off+54])
    fields.append([t,name,scale,trans]); off+=89
sizes={0:1,1:1,2:2,3:2,4:4,5:4,6:8,7:4}; fmts={0:'>B',1:'>b',2:'>H',3:'>h',4:'>I',5:'>i',6:'>q',7:'>f'}
known=sum(sizes[f[0]] for f in fields if f[0] in sizes)
for f in fields:
    if f[0] not in sizes: f[0]=0; f[1]=f[1] or 'bits'; f[2]=1.0; f[3]=0.0
step=recLen+5
recs=[]; p=dataBegin
while p+step<=len(d):
    bt=d[p]
    if bt==0:
        r={}; q=p+4
        for t,name,scale,trans in fields:
            raw=struct.unpack(fmts[t],d[q:q+sizes[t]])[0]; q+=sizes[t]
            r[name]=raw*scale+trans
        recs.append(r); p+=step
    elif bt==1: p+=55
    else: print("unknown block",bt,"at",p,file=sys.stderr); break
cols=['Time','SecL','RPM','TPS','MAP','Baro Pressure','PW','VE _Current','Advance _Current','CLT','IAT','Battery V','Sync status','Sync Loss #','Hard Limiter','Engine','Gwarm','Accel Enrich','Gammae','Duty Cycle','FuelLoad','AFR','Error #','Dwell','Gego','Gbattery']
print('\t'.join(cols))
for r in recs: print('\t'.join(f"{r.get(c,float('nan')):.2f}" for c in cols))
