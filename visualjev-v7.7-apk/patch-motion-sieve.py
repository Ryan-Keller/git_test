from pathlib import Path
import sys

p = Path(sys.argv[1])
s = p.read_text()

def replace_once(old, new, label):
    global s
    if old not in s:
        raise SystemExit(f"missing patch target: {label}")
    s = s.replace(old, new, 1)

replace_once(
    "MotionJev:{hz:'master',status:'idle'}, EgoMotionJev",
    "MotionJev:{hz:'master',status:'idle'}, SieveJev:{hz:'master',status:'idle'}, EgoMotionJev",
    "SieveJev head"
)

old = """function blockFlow(curr,prev,w,h){if(!prev)return[];const cells=[],cols=12,rows=7,bw=w/cols,bh=h/rows,R=3,S=2;for(let gy=0;gy<rows;gy++){for(let gx=0;gx<cols;gx++){const cx=Math.floor((gx+.5)*bw),cy=Math.floor((gy+.5)*bh);let best={err:1e12,dx:0,dy:0};for(let dy=-S;dy<=S;dy++)for(let dx=-S;dx<=S;dx++){let err=0,c=0;for(let oy=-R;oy<=R;oy+=2)for(let ox=-R;ox<=R;ox+=2){const x=cx+ox,y=cy+oy,px=x-dx,py=y-dy;if(x<0||y<0||px<0||py<0||x>=w||y>=h||px>=w||py>=h)continue;err+=Math.abs(curr[y*w+x]-prev[py*w+px]);c++;}err/=Math.max(1,c);const move=Math.hypot(dx,dy),bestMove=Math.hypot(best.dx,best.dy);if(err<best.err-.01||(Math.abs(err-best.err)<.01&&move<bestMove))best={err,dx,dy}}cells.push({gx,gy,x:cx,y:cy,dx:best.dx,dy:best.dy,mag:Math.hypot(best.dx,best.dy),err:best.err})}}return cells}
function flowFeatures(cells){if(!cells.length)return{global_dx:0,global_dy:0,camera_motion:0,residual:0,coherence:1,near:0,mid:0,far:0,motion_density:0,content_motion:0,cells:[]};const gdx=median(cells.map(c=>c.dx)),gdy=median(cells.map(c=>c.dy)),res=cells.map(c=>Math.hypot(c.dx-gdx,c.dy-gdy)),mags=cells.map(c=>c.mag);const cam=clamp(Math.hypot(gdx,gdy)/4,0,1),resMed=median(res),q75=quantile(res,.75),q25=quantile(res,.25),coh=cells.filter((c,i)=>res[i]<1.2).length/cells.length,dens=cells.filter(c=>c.mag>.7).length/cells.length,content=clamp(mean(res)/3,0,1);return{global_dx:gdx,global_dy:gdy,camera_motion:cam,residual:clamp(resMed/3,0,1),coherence:+coh.toFixed(3),near:clamp(q75/3,0,1),mid:clamp(median(res)/3,0,1),far:clamp(q25/3,0,1),motion_density:+dens.toFixed(3),content_motion:content,cells:cells.map((c,i)=>({...c,res:res[i]}))}}
function clusterEntities(flow){const active=flow.cells.filter(c=>c.res>1.0||c.mag>1.5);const seen=new Set(),clusters=[];for(const c of active){const key=c.gx+','+c.gy;if(seen.has(key))continue;const q=[c],group=[];seen.add(key);while(q.length){const a=q.pop();group.push(a);for(const b of active){const k=b.gx+','+b.gy;if(seen.has(k))continue;if(Math.abs(a.gx-b.gx)+Math.abs(a.gy-b.gy)===1){seen.add(k);q.push(b)}}}const cx=mean(group.map(v=>v.x)),cy=mean(group.map(v=>v.y)),speed=mean(group.map(v=>v.mag)),residual=mean(group.map(v=>v.res));clusters.push({x:cx,y:cy,size:group.length,speed,residual})}clusters.sort((a,b)=>b.size-a.size);return clusters.slice(0,8)}"""

new = """const MOTION_THRESHOLDS=[6,12,18,28]; // lower number = MORE sensitive; higher = LESS sensitive. Never invert this.
function blockFlow(curr,prev,w,h){if(!prev)return[];const cells=[],cols=12,rows=7,bw=w/cols,bh=h/rows,R=3,S=2;for(let gy=0;gy<rows;gy++){for(let gx=0;gx<cols;gx++){const cx=Math.floor((gx+.5)*bw),cy=Math.floor((gy+.5)*bh);let best={err:1e12,dx:0,dy:0},rawChange=0,rawCount=0;for(let oy=-R;oy<=R;oy+=2)for(let ox=-R;ox<=R;ox+=2){const x=cx+ox,y=cy+oy;if(x<0||y<0||x>=w||y>=h)continue;rawChange+=Math.abs(curr[y*w+x]-prev[y*w+x]);rawCount++;}rawChange/=Math.max(1,rawCount);for(let dy=-S;dy<=S;dy++)for(let dx=-S;dx<=S;dx++){let err=0,c=0;for(let oy=-R;oy<=R;oy+=2)for(let ox=-R;ox<=R;ox+=2){const x=cx+ox,y=cy+oy,px=x-dx,py=y-dy;if(x<0||y<0||px<0||py<0||x>=w||y>=h||px>=w||py>=h)continue;err+=Math.abs(curr[y*w+x]-prev[py*w+px]);c++;}err/=Math.max(1,c);const move=Math.hypot(dx,dy),bestMove=Math.hypot(best.dx,best.dy);if(err<best.err-.01||(Math.abs(err-best.err)<.01&&move<bestMove))best={err,dx,dy}}const passes=MOTION_THRESHOLDS.map(t=>rawChange>=t),survival=passes.filter(Boolean).length;cells.push({gx,gy,x:cx,y:cy,dx:best.dx,dy:best.dy,mag:Math.hypot(best.dx,best.dy),err:best.err,change:+rawChange.toFixed(2),passes,survival})}}return cells}
function flowFeatures(cells){if(!cells.length)return{global_dx:0,global_dy:0,camera_motion:0,residual:0,coherence:1,near:0,mid:0,far:0,motion_density:0,content_motion:0,sieve:{thresholds:MOTION_THRESHOLDS,pass_fraction:[0,0,0,0],micro_motion_density:0,robust_motion_density:0,mean_survival:0},cells:[]};const gdx=median(cells.map(c=>c.dx)),gdy=median(cells.map(c=>c.dy)),res=cells.map(c=>Math.hypot(c.dx-gdx,c.dy-gdy));const cam=clamp(Math.hypot(gdx,gdy)/4,0,1),resMed=median(res),q75=quantile(res,.75),q25=quantile(res,.25),coh=cells.filter((c,i)=>res[i]<1.2).length/cells.length,dens=cells.filter(c=>c.mag>.7).length/cells.length,content=clamp(mean(res)/3,0,1),passFraction=MOTION_THRESHOLDS.map((_,i)=>cells.filter(c=>c.passes[i]).length/cells.length),meanSurvival=mean(cells.map(c=>c.survival/MOTION_THRESHOLDS.length));return{global_dx:gdx,global_dy:gdy,camera_motion:cam,residual:clamp(resMed/3,0,1),coherence:+coh.toFixed(3),near:clamp(q75/3,0,1),mid:clamp(median(res)/3,0,1),far:clamp(q25/3,0,1),motion_density:+dens.toFixed(3),content_motion:content,sieve:{thresholds:MOTION_THRESHOLDS,pass_fraction:passFraction.map(v=>+v.toFixed(3)),micro_motion_density:+passFraction[0].toFixed(3),robust_motion_density:+passFraction[3].toFixed(3),mean_survival:+meanSurvival.toFixed(3)},cells:cells.map((c,i)=>({...c,res:res[i]}))}}
function clusterEntities(flow){const active=flow.cells.filter(c=>c.survival>=2||c.res>.8||c.mag>1.0);const seen=new Set(),clusters=[];for(const c of active){const key=c.gx+','+c.gy;if(seen.has(key))continue;const q=[c],group=[];seen.add(key);while(q.length){const a=q.pop();group.push(a);for(const b of active){const k=b.gx+','+b.gy;if(seen.has(k))continue;if(Math.abs(a.gx-b.gx)+Math.abs(a.gy-b.gy)===1){seen.add(k);q.push(b)}}}const cx=mean(group.map(v=>v.x)),cy=mean(group.map(v=>v.y)),speed=mean(group.map(v=>v.mag)),residual=mean(group.map(v=>v.res)),survival=mean(group.map(v=>v.survival/MOTION_THRESHOLDS.length));clusters.push({x:cx,y:cy,size:group.length,speed,residual,survival})}clusters.sort((a,b)=>b.size-a.size);return clusters.slice(0,8)}"""
replace_once(old, new, "threshold bank")

replace_once(
    "setHead('EgoMotionJev',flow.camera_motion.toFixed(2));",
    "setHead('SieveJev','micro '+flow.sieve.micro_motion_density.toFixed(2)+' · robust '+flow.sieve.robust_motion_density.toFixed(2));setHead('EgoMotionJev',flow.camera_motion.toFixed(2));",
    "SieveJev status"
)

replace_once(
    "far:+flow.far.toFixed(3)},range,appearance",
    "far:+flow.far.toFixed(3),sieve:flow.sieve},range,appearance",
    "sieve export"
)

replace_once(
    "separate_observation_evidence_hypothesis_query_buses:true,dual_2d_3d_tracks:true",
    "separate_observation_evidence_hypothesis_query_buses:true,parallel_motion_threshold_bank:true,lower_motion_threshold_means_more_sensitive:true,no_single_threshold_can_gate_other_heads:true,dual_2d_3d_tracks:true",
    "manifest contracts"
)

replace_once(
    "bus_separation:true,camera_capability_reported",
    "bus_separation:true,parallel_motion_threshold_bank:MOTION_THRESHOLDS.join(',')==='6,12,18,28',threshold_direction_correct:MOTION_THRESHOLDS[0]<MOTION_THRESHOLDS.at(-1),camera_capability_reported",
    "self test"
)

p.write_text(s)
print("patched", p, len(s))
