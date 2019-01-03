
import re
import os
import subprocess

import numpy as np
import scipy.stats as stats

from optparse import OptionParser 
from string import maketrans
from glob import glob

resultsdir="/local/evrr-results/results/"
#resultsdir="/local/evrr-results/2011-12-13/"

################################################################################

def gnuplot( name ):
    subprocess.call("PATH=$PATH:/home/mies/Scripts /bin/bash -c \"cd 0-simulation;/home/mies/Scripts/mkplot "+name+"\"", shell=True)

def confidence(vec, ci = 0.95):
    return stats.sem(vec) * stats.t._ppf( (1.0+ci)/2., len(vec) )

def avg(vec):
    avg=0;
    for i in vec: avg+=i
    return avg/len(vec)

def dict_str( d ):
    keys=sorted(d.keys())
    s=""; 
    for k in keys: s += str(k)+"="+str(d[k])+", ";
    return s[:-2] 

################################################################################

class om_vector:
    def __init__(self, id, mod, name, file):
        self.mod=mod
        self.name=name
        self.id=id
        self.file=file
        self.data=[]
        
    def vec(self):
        v=[]
        for x,y in self.data: v.append(y)
        return v;
    
    # returns a triple comprising average and confidence
    def average(self, start=0, end=None, ci = 0.95):
        if end==None: end=len(self.data)
        elif end<0: end=len(self.data)-end
        v=[x[1] for x in self.data[start:end]]; a=avg(v); b=confidence(v, ci=ci)
        return (a,a-b,a+b)
      
class om_histo:
    def __init__(self, mod, name, file):
        self.mod=mod
        self.name=name
        self.file=file
        self.count=0
        self.mean=0.0
        self.sum=0.0
        self.sqrsum=0.0
        self.min=0
        self.max=0
        self.bin=[]        

    def ccdf(self):
        value=0.0
        c=[]
        for a,b in self.bin:
            c.append( (a, value) );
            value += b; 
        x=[]
        for v in c: 
            y=1-(v[1]/value)
            x.append((v[0], y))
            if y<0.001: break
        
        return x[1:-1]; 

class om_file:
    def __init__(self):
        self.name=""
        self.attr={}
        self.vectors={}
        self.scalars={}
        self.histograms={}
        
    def par(self, key):
        if key in self.attr: return self.attr[key].strip()
        return None
    
    def iteration_vars(self):
        return self.attr["iterationvars2"]
    
    def iteration_dict(self):
        a=re.findall("\$([^=]+)=([^,\n]+)", self.iteration_vars());
        dict={}; 
        for x in a: dict[x[0]]=x[1]
        return dict
       
    def trigger(self, vecName, value, ignore=0, window=1 ):
        v=self.vectors[vecName].data
        for i in range(ignore,len(v)-window): 
            y=0; x=v[i][0]
            for j in range(0,window): y += v[i+j][1]
            y /= window            
            if y>=value and x>=ignore: return x
        if len(v)==0: return -1
        return v[-1][0]
    
    def average(self, vecName, start=0, end=None):
        return self.vectors[vecName].avg(start,end)
    
    def ccdf(self,name):
        return self.histograms[name].ccdf()
    
    def values(self, vecName ):
        return self.vectors[vecName].data

    def vector(self, vecName ):
        return self.vectors[vecName]
        
    def read(self,name):
        if not os.path.exists(name+".vec") or not os.path.exists(name+".sca"):
            return False 
        
        readIt=False
        self.name=name        
        # read vectors
        idmap={}
        f=open(name+".vec",'r');
        for line in f:
            s=line.split();
            if len(s)==0: continue
            if s[0]=="attr" and len(s)>=3:
                x=""; 
                for y in s[2:]: x += y+" "  
                self.attr[s[1]]=x
                readIt=True
            elif s[0]=="vector": 
                id=int(s[1])
                v=om_vector(id, s[2],s[3], self)
                idmap[id]=v
                self.vectors[v.name]=v
            elif s[0].isdigit():
                p=(float(s[2]),float(s[3]))
                idmap[int(s[0])].data.append(p)
        f.close()
        
        # read histograms
        curr=None
        f=open(name+".sca",'r');
        for line in f:
            s=line.split()
            if len(s)==0: continue
            elif s[0]=="scalar": self.scalars[s[2]]=s[3]
            elif s[0]=="statistic":
                curr=om_histo(s[1], s[2], self)
                self.histograms[s[2]]=curr
            elif s[0]=="field" and len(s)>=3:
                v = float(s[2])
                if s[1]=="count": curr.count=v
                elif s[1]=="mean": curr.mean=v
                elif s[1]=="stddev": curr.stddev=v
                elif s[1]=="sum": curr.sum=v
                elif s[1]=="sqrsum": curr.sqrsum=v
                elif s[1]=="min": curr.min=v
                elif s[1]=="max": curr.max=v
            elif s[0]=="bin" and len(s)>=3:
                p=(float(s[1]),float(s[2]));
                curr.bin.append(p)
        f.close()
        
        return readIt
        
class om_runs:
    def __init__(self, name = None):
        self.files=[]
        self.delta=None
        self.replace=[]
        if name==None: return
        f=glob( name+"*" ); s=set()
        of=[]
        for l in f: 
            if l.endswith(".vec"): s.add( l[:-4] )
        for l in s: 
            omf=om_file()
            if omf.read(l): self.files.append(omf)
            
    def include(self, match):
        r=om_runs(); exp=[];
        r.replace=self.replace
        for m in match: exp.append(re.compile(m))
        for l in self.files:
            i=l.attr["iterationvars2"]; inc=False
            for e in exp: 
                if not e.search(i): inc=True 
            if not inc: r.files.append(l)
        return r

    def get_delta_iteration_vars(self):
        if self.delta!=None: return self.delta
        delta=set(); old_dict=None
        for file in self.files:
            dict=file.iteration_dict()
            if old_dict!=None:
                for k in dict: 
                    if dict[k]!=old_dict[k]: delta.add(k)
            old_dict=dict
        self.delta=delta
        return delta
    
    def par_values(self, par):
        values=[]
        for file in self.files:
            dict=file.iteration_dict()
            if (par in dict) and (not dict[par] in values):
                values.append(dict[par])
        values.sort( key=lambda x: x.zfill(10) )
        return values
    
    def par_runs(self, par, value):
        r=om_runs(); r.replace=self.replace
        for file in self.files:
            dict=file.iteration_dict()
            if dict[par]==value: r.files.append(file)
        return r
    
    def get_title(self, i, use_string=True, use_value=True ):
        title={}; delta=self.get_delta_iteration_vars()
        file=self.files[i]; dict=file.iteration_dict();
        for k in delta: title[k]=dict[k]
        if use_value and use_string: title=float(title.values()[0])
        elif use_string: 
            title=dict_str(title)
            for s in self.replace: title=re.sub(s[0], s[1], title)
        return title
    
    def trigger(self, name, trigger = 1.0, ignore=0, window=2, use_string=True, use_value=True ):
        data=[]
        for i in range(0,len(self.files)):
            file=self.files[i]; 
            triggerPos=file.trigger(name, trigger, ignore, window )
            data.append([self.get_title(i,use_string, use_value), triggerPos])
        return data 
    
    def values(self, name,  use_string=True, use_value=True ):
        data=[]
        for i in range(0,len(self.files)):
            file=self.files[i];
            data.append([self.get_title(i, use_string, use_value), file.values(name)])
        return data 
    
    def ccdfs(self, name, use_string=True, use_value=True):
        data=[]
        for i in range(0,len(self.files)):
            data.append([self.get_title(i, use_string, use_value), 
                         self.files[i].ccdf(name)])
        return data 
 
################################################################################
    
def labeled_error_diagram(name, xticks, xlabel, ylabel, data, yrange=None, xexpand = 0, type="boxes", add=""):
    gp="""
set border 31 linewidth 0.3
set style data linespoints
set datafile separator ","
set xlabel "%(xlabel)s" 
set ylabel "%(ylabel)s" 
""" % {'xlabel':xlabel, 'ylabel':ylabel}
    if type=='boxes': gp+="set pointsize 0.0\n"

    xt=""; p=0.0;
    for tick in xticks: 
        p+=1.0; xt+='"'+str(tick)+'" '+str(p)+", "
    xt=xt[:-2]   
    gp += """
set xrange [0:%(xrange)d]
set xtics (%(xtics)s)
set style fill solid
set boxwidth 0.2 relative
set multiplot
""" % {'xrange': len(xticks)+1+xexpand*2, 'xtics': xt};
    if yrange!=None: gp+="set yrange ["+str(yrange)+"]\n";
    gp+=add+"\n"
    p=0
    x1=(-0.2*len(data))/2+0.1; x2=(0.1*len(data))/2
    gp1="plot\\\n"; gp2="plot\\\n"
    for group in data:
        if type=='boxes':
            gp1 +=("""  "%(name)s.dat" using ($1+(%(x1)f)):%(p1)d with \\\n"""+
                   """  boxes fs solid %(x2)f lt 1 title "%(title)s",\\\n""")\
                   % { 'type':type, 'name': name, 'x1':x1, 'p1':p*3+2, 'x2':x2, 'title':group[0] }
            gp2 +=("""  "%(name)s.dat" using ($1+(%(x1)f)):%(p1)d:%(p2)d:%(p3)d \\\n"""+
                   """  with errorbars lt 1 pt 1 title "%(title)s",\\\n""")\
                  % { 'name': name, 'x1':x1, 'p1':p*3+2, 'p2':p*3+3, 'p3':p*3+4, 'title':group[0] }
        elif type=='lines':
            gp1 = ""
            gp2 +=("""  "%(name)s.dat" using ($1):%(p1)d:%(p2)d:%(p3)d \\\n"""+
                   """  with errorlines title "%(title)s",\\\n""")\
                   % { 'name': name, 'p1':p*3+2, 'p2':p*3+3, 'p3':p*3+4, 'title':group[0] }

        x1 += 0.2; x2 += 0.3; p += 1
    gp1 = gp1[:-3]+"\n"
    gp2 = gp2[:-3]+"\n"
    
    gp += gp1 + gp2
    
    gd=""
    line=[]
    for group in data:
        i=0
        for d in group[1]:
            if len(line)<=i: 
                line.append([])
                line[i].append(i+1)
            line[i].append(d[0])        
            line[i].append(d[1])        
            line[i].append(d[2])
            i+=1
    for l in line:
        for v in l: gd += str(v)+","
        gd = gd[:-1]+"\n" 

    f=open("0-simulation/"+name+".gp",'w'); f.write(gp); f.close();
    f=open("0-simulation/"+name+".dat",'w'); f.write(gd); f.close();
    gnuplot(name+".gp")

################################################################################

'''
    data=[ [ Title1, [ [x,y,err1,err2], ... ] ], [ Title2, ... ] ]
'''
def plot(name, xlabel, ylabel, data, yrange=None, xrange=None, errorbars=False, add=[]):
    data.sort(key=lambda rec: rec[0].zfill(30));  
    gp=\
"""
set ylabel "%(ylabel)s"
set xlabel "%(xlabel)s"
set datafile separator ","
""" % {'ylabel':ylabel,'xlabel':xlabel}
    if xrange != None: gp += "set xrange ["+xrange+"]\n"
    if yrange != None: gp += "set yrange ["+yrange+"]\n"
    if errorbars: gp += "set style data errorlines\n"
    for a in add: gp += a+"\n"
    gp+="plot \\\n";
    lines=0
    for p in data:
        if len(p[1])>lines: lines=len(p[1])
    line=[]; i=0
    for x in range(0,lines): line.append([])
    for p in data:
        title=p[0]; vec=p[1];
        if errorbars:
            gp+="  \"%(name)s.dat\" using %(1)d:%(2)d:%(3)d:%(4)d title \"%(title)s\",\\\n"\
            % {'name':name, '1':i*4+1,'2':i*4+2,'3':i*4+3,'4':i*4+4, 'title':title}
        else:
            gp+="  \"%(name)s.dat\" using %(1)d:%(2)d title \"%(title)s\",\\\n"\
            % {'name':name, '1':i*2+1,'2':i*2+2, 'title':title }
        i += 1; j = 0;
        for d in vec:
            
            line[j].append(d[0])            
            line[j].append(d[1])            
            if errorbars:
                line[j].append(d[2])            
                line[j].append(d[3])
            j+=1            
        for k in range(j,len(line)):
            line[k].append(""); line[k].append("")
            if errorbars: line[k].append(""); line[k].append("")
    gp = gp[:-3]+"\n"

    gd=""    
    for l in line:
        for v in l: gd += str(v)+","
        gd = gd[:-1]+"\n" 
    
    if not os.path.exists("0-simulation/"): os.mkdir("0-simulation")
    f=open("0-simulation/"+name+".gp",'w'); f.write(gp); f.close();
    f=open("0-simulation/"+name+".dat",'w'); f.write(gd); f.close();
    gnuplot(name+".gp")

################################################################################

def data_scale( data, sx = 1.0, sy = 1.0 ):
    for x in data:
        l=list(x[1])
        for i in range(0,len(l)):  l[i] = (l[i][0]*sx,l[i][1]*sy)
        x[1]=l
    return data

def data_intify_x( data ):
    for x in data:
        l=list(x[1])
        for i in range(0,len(l)): l[i] = (int(l[i][0]),l[i][1])
        x[1]=l
    return data
    
def data_intify_y( data ):
    for x in data:
        l=list(x[1])
        for i in range(0,len(l)): l[i] = (l[i][0],int(l[i][1]))
        x[1]=l
    return data

def data_select( data, include ):
    data2=[]
    for x in data:
        if x[0] in include: data2.append(x) 
    return data2

################################################################################
   
def traffic( name, runs, add="" ):
    # traffic density
    data=runs.ccdfs("controlTrafficTotalDensity", use_string=True, use_value=False)
    data=data_intify_x(data_scale(data, sx= 1.0/1000.0))
    plot( name+"-traffic-ccdf", " Traffic per node [kbytes] "+add, "P( Traffic > x )",
          data, add=["set style data lines","set xrange [0:]"]);
          
    # bandwidth density 
    try:
        data=runs.ccdfs("controlBandwidthTotalDensity", use_string=True, use_value=False)
        data=data_intify_x(data_scale(data, sx= 1.0/1000.0))
        plot( name+"-bw-ccdf", " Bandwidth per node [kbytes] "+add, "P( Bandwidth > x )",
              data, add=["set style data lines","set xrange [0:]"]);
    except: pass
        
    # absolute traffic
    try:
        data=runs.values("controlTrafficTotalAvg", use_string=True, use_value=False)
        data=data_intify_y(data_scale(data, sy= 1.0/1000.0))
        plot( name+"-traffic", " Time [s] "+add, " Traffic per node [kbytes] ",
              data, add=["set style data lines","set xrange [0:]","set key top left"]);
    except: pass
          
    # absolute messages
    try:
        data=runs.values("controlMessagesTotalAvgDelta", use_string=True, use_value=False)
        plot( name+"-messages", " Time [s] "+add, "Messages per node [messages/s]", 
              data, add=["set style data lines","set xrange [0:]","set key top left"]);
    except: pass
          
def delivery( name, runs, type='general', add="" ):
    data=runs.values(type+"DeliveryRatio", use_string=True, use_value=False)
    plot( name+"-delivery-ratio", "Delivery ratio", " Time [s] "+add,
          data, add=["set style data lines","set xrange [0:]", "set yrange [0:1]", "set key bottom right"]);
          
def stretch( name, runs, add=""  ): 
    data=runs.ccdfs("generalStretchMultiplicative", use_string=True, use_value=False)
    plot( name+"-stretch-mul", " Stretch "+add, "P( Stretch > x )",
          data, add=["set style data lines","set xrange [0:]"]); 
    data=runs.ccdfs("generalStretchAdditive", use_string=True, use_value=False)
    plot( name+"-stretch-add", " Additive stretch "+add, "P( Add. stretch > x )",
          data, add=["set style data lines","set xrange [0:]"]); 

def rtsize( name, runs, add=""  ): 
    data=runs.ccdfs("routingTableSizeDensity", use_string=True, use_value=False)
    plot( name+"-rtsize-ccdf", " RT-size "+add, "P( RT-size > x )",
          data, add=["set style data lines","set xrange [0:]"]); 
    data=runs.values("routingTableSizeAvg", use_string=True, use_value=False)
    plot( name+"-rtsize", " Time [s] "+add, "Avg. RT-size",
          data, add=["set style data lines"]);

################################################################################

# returns the convergence time and needed traffic to converge          
def convergence_time_traffic( runs, par='k' ):
    vtraffic={}; vtime={}; par_values=runs.par_values(par);
    print par_values
    for pv in par_values:
        r=runs.par_runs(par, pv)
        for i in range(0,len(r.files)):
            file=r.files[i];title=r.get_title(i,use_string=True,use_value=False)
            if not title in vtime.keys(): 
                vtime[title]=[title,[]]; vtraffic[title]=[title,[]]
            try:
                time=file.trigger("routingRingAccuracyAvg",1.0, ignore=2, window=1)
                traffic=file.vectors["controlTrafficTotalAvg"].data[int(time)][1]
                vtime[title][1].append( (float(pv), time) );
                vtraffic[title][1].append( (float(pv), traffic/1000.0) )
            except:
                continue            
    for k in vtime.keys(): 
        vtime[k][1].sort(key=lambda x: x[0])
        vtraffic[k][1].sort(key=lambda x: x[0])
    return (vtime.values(), vtraffic.values())

def convergence( runs, prefix, mode='N' ):
    selr=runs.include(["immediateResponse=true"])
    prefix+="-"+mode
   
    tt=convergence_time_traffic(selr.include(["useAnycast=false"]), mode)
    sel=""
    if mode=='N':
        add=["set key top left", "set logscale x","set xrange [128:16384]"]
        sel=r"([$]N=512)|([$]N=8192)"
        plot(prefix+"-lin-vs-dis-time", 
             "Number of relays", "Convergence time [s]", tt[0], add=add)
        plot(prefix+"-lin-vs-dis-traffic", 
             "Number of relays", "Traffic [kbytes]", tt[1], add=add)
       
    if mode=='k':
        add=["set key bottom left", "set xrange [5:35]", "set yrange[0:]"]
        sel=r"([$]k=10)|([$]k=30)"
        plot(prefix+"-lin-vs-dis-time", 
             "Max. no. of neighbors", "Convergence time [s]", tt[0], add=add)
        plot(prefix+"-lin-vs-dis-traffic", 
             "Max. no. of neighbors", "Traffic [kbytes]", tt[1], add=add)

    selr1=selr.include(["useAnycast=false", "useLinearization=false"]);
    traffic( prefix+"-dis", selr1 )
    rtsize ( prefix+"-dis", selr1 )
    stretch( prefix+"-dis", selr1 ) 

    selr1=selr.include(["useAnycast=false", "useLinearization=true"]);
    traffic( prefix+"-lin", selr1 )
    rtsize ( prefix+"-lin", selr1 )
    stretch( prefix+"-lin", selr1 )
    
    selr2=selr.include([sel,"useLinearization=false"])
    traffic( prefix+"-any-comp", selr2 )
    rtsize ( prefix+"-any-comp", selr2 )
    
    selr3=selr.include([sel,"useAnycast=false"])
    traffic( prefix+"-mech-comp", selr3 )
    rtsize ( prefix+"-mech-comp", selr3 )
    stretch( prefix+"-mech-comp", selr3 )
    
################################################################################

def parallel_discovery( name ):
    runs=om_runs(resultsdir+name )
    plots_time=[]
    plots_traffic=[]
    for immediate in ["true","false"]:
        for topology in ["ba","star"]:
            selr=runs.include(["immediateResponse="+immediate,"topology="+str(topology)])
            data=selr.trigger("routingRingAccuracyAvg",1.0, ignore=2, window=1, use_value=True)
            data.sort(key=lambda a: a[0])
            name=""
            if topology=="ba": name+= "Internet-inspired";
            else: name+="Star"
            if immediate=="true": name += ", Immediately";
            else: name+=", In-round"
            plots_time.append([name,data])
            
    selr=runs.include(["immediateResponse=true","parDisc=(1|3|5|7)","topology=star"]);
    selr.replace=[("parDisc","No. Par.disc.")]
    plots_traffic=selr.ccdfs("controlTrafficTotalDensity", use_string=True, use_value=False)
                       
    plot("parallel-discovery-time", 
         "No. of parallel discoveries", "Convergence time [s]",
         plots_time,add=["set key top right","set xrange [0:8]","set yrange[1:]"])

    plots_traffic=data_intify_x(data_scale(plots_traffic, sx= 1.0/1000.0))
    plot( "parallel-discovery-traffic", " Traffic per node [kbytes] ", 
          "P( Traffic > x )",
          plots_traffic, add=["set style data lines","set xrange [0:]"]);
           
################################################################################

def ltchurn_delivery( prefix, files, anycast='true', linearization='false' ):
    LTset=files.par_values('Lifetime'); Nset=files.par_values('N');
    values=[]
    for lifetime in LTset:
        v=[]
        for N in Nset:
            sel=files.include(["useLinearization=false","useAnycast=true","N="+str(N),"Lifetime="+str(lifetime)+","])
            for f in sel.files:
                mtype=""
                if anycast=='true': mtype="anycast" 
                else: mtype="general";
                a=f.vector(mtype+"DeliveryRatio").average(start=50)
                v.append( (N, a[0], a[1], a[2]) )
        values.append([str(lifetime),v])
    type=""
    if anycast=='true': type+="-anycast";
    if linearization=='true': type+="-linear";
    
    plot(prefix+"-delivery-ratio"+type, 
        "Number of relays", "Delivery ratio", values, 
        errorbars=True, yrange="0.70:1.0", 
        add=["set logscale x","set key bottom right width -2"])
    
################################################################################

def ltchurn_bandwidth( prefix, files, linearization='false' ):
    LTset=files.par_values('Lifetime'); Nset=files.par_values('N');
    
    # bandwdith without anycast
    sel=files.include(["useLinearization="+linearization, "useAnycast=false"])
    values=[]
    for lifetime in LTset:
        v=[]
        for N in Nset:
            for f in sel.include(["N="+str(N),"Lifetime="+str(lifetime)+","]).files:
                a=f.vector("controlBandwidthTotalAvg").average()
                b=f.vector("controlBandwidthTotalMax").average()
                v.append( (N,a[0],a[1],a[2]) )
        values.append([str(lifetime),v])
    type=""
    if linearization=='true': type+="-linear";
    plot(prefix+"-bw"+type, 
         "Number of relays", "Avg. BW per node [bytes/s]", values, 
         errorbars=True, add=["set key bottom right width -1","set logscale x"])
    
    # traffic
    traffic( prefix+type, sel );

    # bandwidth comparision w/o anycast
    values=[]
    sel=files.include(["useLinearization="+linearization])
    for anycast in ['true','false']:
        for lifetime in [1000,10000]:
            v=[]
            for N in Nset:
                for f in sel.include(["useAnycast="+anycast,"N="+str(N),"Lifetime="+str(lifetime)+","]).files:
                    a=f.vector("controlBandwidthTotalAvg").average()
                    b=f.vector("controlBandwidthTotalMax").average()
                    v.append( (N,a[0],a[1],a[2]) )
            mtype=""
            if anycast=='false': mtype="Unicast"
            else: mtype="Anycast"
            values.append([str(lifetime)+" ("+mtype+")",v])
    type=""
    if linearization=='true': type+="-linear";
    plot(prefix+"-bw-anycast"+type,
         "Number of relays", "Avg. BW per node [bytes/s]", values,
         errorbars=True, 
         add=["set key top left","set logscale x","set key bottom right width -1"]) 

################################################################################

def ltchurn_rtentries( prefix, files, linearization='false' ):
    LTset=files.par_values('Lifetime'); Nset=files.par_values('N');
    
    values=[]
    sel=files.include(["useLinearization="+linearization, "useAnycast=false"])
    for lt in LTset:
        v=[]
        for N in Nset:
            for f in sel.include(["N="+str(N),"Lifetime="+str(lt)+","]).files:
                a=f.vector("routingTableSizeAvg").average()
                b=f.vector("routingTableSizeMax").average()
                v.append( (N,a[0],a[1],a[2]) )
        values.append([str(lt),v])
    type=""
    if linearization=='true': type+="-linear";
    plot(prefix+"-rtsize-avg"+type,  
         "Number of relays", "Avg. entries per node", values, 
         errorbars=True, add=["set key bottom right width -2","set logscale x"])

    values=[]
    sel=files.include(["useLinearization="+linearization])
    for ac in ['true','false']:
        for lt in [1000,10000]:
            v=[]
            for N in Nset:
                for f in files.include(["useAnycast="+ac,"N="+str(N),"Lifetime="+str(lt)+","]).files:
                    a=f.vector("routingTableSizeAvg").average()
                    v.append( (N,a[0],a[1],a[2]) )
            mtype=""
            if ac=='false': mtype="Unicast"
            else: mtype="Anycast"
            values.append([str(lt)+" ("+mtype+")",v])
    type=""
    if linearization=='true': type+="-linear";

    plot(prefix+"-rtsize-avg-anycast"+type,  
         "Number of relays", "Avg. entries per node", values, 
         errorbars=True, add=["set key top left width -2","set logscale x"])
           
################################################################################

def ltchurn( topology='star' ):
    files=om_runs(resultsdir+"k20-lt-churn-*")
    files=files.include(["topology="+topology ])
    prefix="lt-churn-"+topology
    ltchurn_delivery(prefix, files, anycast='true', linearization='false')
    ltchurn_delivery(prefix, files, anycast='false', linearization='false')
    ltchurn_delivery(prefix, files, anycast='true', linearization='true')
    ltchurn_delivery(prefix, files, anycast='false', linearization='true')
    
    for linear in ['true','false']:
        ltchurn_bandwidth(prefix, files, linearization=linear)
        ltchurn_rtentries(prefix, files, linearization=linear)

################################################################################

#[Config k20-cf-churn]
#extends=k20
#sim-time-limit=300s
#**.dynamicsModel=${Churn="CatastrophicFailure"}
#**.useLinearization=${useLinearization=false,true}
#**.useAnycast=${useAnycast=true,false}
#**.fraction=${F=0.05,0.10,0.20,0.40} 
def cfchurn( replace ):
    files=om_runs(resultsdir+"k20-cf-churn-*")
    files.replace=replace
    for topology in ['star','ba']:
        runs=files.include(["topology="+topology,"useAnycast=true","N=2048","[$]F=0.10"])
        pre="cf-churn-"+topology+"-lin-vs-dis-n2048-k20-f10-any"
        traffic( pre, runs ); delivery( pre, runs ); rtsize( pre, runs )
        
        runs=files.include(["topology="+topology,"useLinearization=false","useAnycast=true","[$]F=0.10"])
        pre="cf-churn-"+topology+"-n-k20-f10-dis-any" 
        traffic( pre, runs ); delivery( pre, runs ); rtsize( pre, runs )
        
        runs=files.include(["topology="+topology,"useLinearization=false","useAnycast=true","[$]N=2048"])
        pre="cf-churn-"+topology+"-f-n2048-k20-dis-any"
        traffic( pre, runs ); delivery( pre, runs ); rtsize( pre, runs )
    
    
################################################################################

replace=[
         ("useAnycast=true","Anycast"),
         ("useAnycast=false","Unicast"),
         ("useLinearization=true","Linearization"),
         ("useLinearization=false","Discovery"),
         ("immediateResponse=false","In-round"),
         ("topology=ba","Internet-inspired"),
         ("topology=star", "Star"),
         ("N=256","N=255"), ("N=512","N=511"),
         ("N=1024","N=1023"), ("N=2048","N=2047"),
         ("N=4096","N=4095"), ("N=8192","N=8191")
        ]

################################################################################

cfchurn(replace)

################################################################################

runs=om_runs(resultsdir+"k20-convergence" )
runs.replace=replace 
convergence( runs.include(["topology=ba"]), "convergence-ba", 'N' )
convergence( runs.include(["topology=star"]), "convergence-star", 'N' )

################################################################################

runs=om_runs(resultsdir+"n2048-convergence" )
runs.replace=replace 
convergence( runs.include(["topology=star"]), "convergence-star", 'k' )
convergence( runs.include(["topology=ba"]), "convergence-ba", 'k' )

################################################################################

parallel_discovery("k20-n2048-discovery")

################################################################################

ltchurn('star')
ltchurn('ba')

