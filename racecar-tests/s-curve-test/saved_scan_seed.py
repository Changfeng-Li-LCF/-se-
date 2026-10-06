"""Local scan/map seeding; original AMCL and pose monitor verify the result."""
import math
import time

def refine_initial_prior(grid,scan,laser_in_base,prior):
    import numpy as np
    from saved_map_monitor import transform_endpoint
    begun=time.perf_counter()
    width,height,res=grid['width'],grid['height'],grid['resolution']
    values=np.asarray(grid['data'],dtype=np.int8).reshape(height,width)
    occupied=values>=65
    if not occupied.any():return dict(used=False,reason='static map has no occupied cells')
    cap=.30;distances=np.full((height,width),cap,dtype=np.float32)
    reach=int(math.ceil(cap/res))
    for dy in range(-reach,reach+1):
        for dx in range(-reach,reach+1):
            d=math.hypot(dx,dy)*res
            if d>cap:continue
            y0,y1=max(0,-dy),min(height,height-dy);x0,x1=max(0,-dx),min(width,width-dx)
            if y0>=y1 or x0>=x1:continue
            view=distances[y0:y1,x0:x1]
            np.minimum(view,np.where(occupied[y0+dy:y1+dy,x0+dx:x1+dx],d,cap),out=view)
    points=[];ranges=scan['ranges'];stride=max(1,math.ceil(len(ranges)/90))
    for i in range(0,len(ranges),stride):
        r=ranges[i]
        if not math.isfinite(r) or not max(.1,scan['range_min'])<r<min(10.,scan['range_max']):continue
        angle=scan['angle_min']+i*scan['angle_increment']
        points.append(transform_endpoint(laser_in_base,r*math.cos(angle),r*math.sin(angle)))
    if len(points)<12:return dict(used=False,reason='insufficient raw scan endpoints')
    points=np.asarray(points,dtype=float)
    ox,oy,oa=grid['origin'];oc,os=math.cos(oa),math.sin(oa)
    def evaluate(poses):
        scores=[];coverages=[];ratios=[];means=[]
        for start in range(0,len(poses),128):
            ps=poses[start:start+128];c=np.cos(ps[:,2,None]);s=np.sin(ps[:,2,None])
            x=ps[:,0,None]+c*points[:,0]-s*points[:,1];y=ps[:,1,None]+s*points[:,0]+c*points[:,1]
            mx=(oc*(x-ox)+os*(y-oy))/res;my=(-os*(x-ox)+oc*(y-oy))/res
            gx=np.floor(mx).astype(int);gy=np.floor(my).astype(int)
            inside=(gx>=0)&(gx<width)&(gy>=0)&(gy<height)
            known=inside&(values[np.clip(gy,0,height-1),np.clip(gx,0,width-1)]>=0)
            fx,fy=mx-.5,my-.5;ix=np.floor(fx).astype(int);iy=np.floor(fy).astype(int)
            ax,ay=fx-ix,fy-iy
            x0=np.clip(ix,0,width-1);x1=np.clip(ix+1,0,width-1)
            y0=np.clip(iy,0,height-1);y1=np.clip(iy+1,0,height-1)
            d=(distances[y0,x0]*(1-ax)*(1-ay)+distances[y0,x1]*ax*(1-ay)
               +distances[y1,x0]*(1-ax)*ay+distances[y1,x1]*ax*ay)
            counts=known.sum(axis=1);safe=np.maximum(counts,1)
            scores.extend((np.exp(-.5*(d/.08)**2)*known).sum(axis=1)/len(points))
            coverages.extend(counts/len(points))
            ratios.extend(((d<=max(.1,2*res))&known).sum(axis=1)/safe)
            means.extend((d*known).sum(axis=1)/safe)
        return tuple(np.asarray(v) for v in (scores,coverages,ratios,means))
    center=np.array([prior['x'],prior['y'],prior['yaw']],dtype=float)
    def candidates(center,xy_step,xy_span,angle_step,angle_span):
        xy=np.arange(-xy_span,xy_span+xy_step*.5,xy_step)
        angles=np.arange(-angle_span,angle_span+angle_step*.5,angle_step)
        x,y,a=np.meshgrid(xy,xy,angles,indexing='ij')
        return center+np.column_stack((x.ravel(),y.ravel(),a.ravel()))
    coarse=candidates(center,.1,.6,math.radians(5),math.radians(30))
    coarse_scores,*_=evaluate(coarse);coarse_best=coarse[int(np.argmax(coarse_scores))]
    fine=candidates(coarse_best,.025,.1,math.radians(1),math.radians(5))
    scores,coverage,ratio,mean=evaluate(fine);best=int(np.argmax(scores));pose=fine[best]
    away=(np.linalg.norm(coarse[:,:2]-pose[:2],axis=1)>.20)|(
        np.abs(np.arctan2(np.sin(coarse[:,2]-pose[2]),np.cos(coarse[:,2]-pose[2])))>math.radians(8))
    competing=float(coarse_scores[away].max()) if away.any() else 0.
    gap=float(scores[best])-competing;before=evaluate(center[None,:])
    edge=bool(np.any(np.abs(coarse_best[:2]-center[:2])>=.599)
              or abs(coarse_best[2]-center[2])>=math.radians(29.9))
    confident=bool(coverage[best]>=.80 and ratio[best]>=.80 and mean[best]<=.07
                   and gap>=.025 and not edge)
    return dict(used=confident,reason='unambiguous local scan/map seed' if confident else
        'local scan/map peak is ambiguous or outside the search interior; retain broad AMCL prior',
        prior=dict(prior),pose=dict(x=float(pose[0]),y=float(pose[1]),
            yaw=math.atan2(math.sin(float(pose[2])),math.cos(float(pose[2])))),
        covariance_std=[.12,.12,math.radians(6)],endpoints=len(points),
        candidate_count=len(coarse)+len(fine),search_xy_m=.6,search_yaw_deg=30.,
        known_fraction=float(coverage[best]),match_ratio=float(ratio[best]),
        mean_distance_m=float(mean[best]),score=float(scores[best]),competing_score=competing,
        peak_gap=gap,prior_match_ratio=float(before[2][0]),prior_mean_distance_m=float(before[3][0]),
        compute_ms=1000*(time.perf_counter()-begun),map_version=grid['version'])
