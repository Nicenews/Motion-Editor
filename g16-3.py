<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>스틱맨 구조물 에디터</title>
<style>
:root{--bg:#fafaf7;--fg:#1d1d1b;--mut:#6b6b66;--line:#2b2b28;--fix:#d93025;--shape:rgba(60,120,200,.14);--bd:#dcdcd4;--acc:#2f6fd0;--btn:#fff;
box-sizing:border-box;padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px)}
@media(prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#17181a;--fg:#ececea;--mut:#9a9a95;--line:#d8d8d2;--fix:#ff5b50;--shape:rgba(110,160,240,.2);--bd:#34363a;--acc:#6fa0f0;--btn:#232427}}
:root[data-theme="dark"]{--bg:#17181a;--fg:#ececea;--mut:#9a9a95;--line:#d8d8d2;--fix:#ff5b50;--shape:rgba(110,160,240,.2);--bd:#34363a;--acc:#6fa0f0;--btn:#232427}
html,body{height:100%;margin:0}
body{display:flex;flex-direction:column;background:var(--bg);color:var(--fg);font:14px system-ui,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
header{display:flex;flex-wrap:wrap;gap:8px 12px;align-items:center;padding:10px 14px;border-bottom:1px solid var(--bd)}
h1{font-size:15px;margin:0 6px 0 0}
button{font:inherit;color:var(--fg);background:var(--btn);border:1px solid var(--bd);border-radius:8px;padding:6px 12px;cursor:pointer}
button[aria-pressed="true"]{border-color:var(--acc);color:var(--acc);font-weight:600}
.hint{color:var(--mut);font-size:12px;flex:1 1 260px}
main{flex:1;min-height:0}
canvas{display:block;width:100%;height:100%;touch-action:none}
</style></head><body>
<header>
<h1>구조물 에디터</h1>
<button id="free" aria-pressed="false">자유 이동 (Alt)</button>
<button id="reset">초기화</button><span id="modes" style="display:contents"></span><button id="save">저장</button><button id="load">불러오기</button>
<span class="hint" id="hint">노드·링크 드래그 / 더블클릭(탭)으로 고정 / Alt 또는 버튼으로 길이 이내 자유 이동</span>
</header>
<main><canvas id="cv"></canvas></main>
<script>
/* ============ DATA (순수 모델) ============ */
const dist=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
class Scene{
  constructor(){this.nodes=new Map();this.links=[];this.circles=[];this.shapes=[];this.rules=[];this.joints=[];this.obj='A';this._i=1000;}
  add_node(id,x,y,internal=false){this.nodes.set(id,{id,x,y,fixed:false,internal,obj:this.obj});}
  add_link(a,b,internal=false){const d=dist(this.nodes.get(a),this.nodes.get(b));this.links.push({a,b,L:d,Lmax:d,internal});}
  add_circle(up,down,p3,p4){
    const U=this.nodes.get(up),D=this.nodes.get(down),c=this._i++;
    this.add_node(c,(U.x+D.x)/2,(U.y+D.y)/2,true);
    let l=p3,r=p4;
    if(p4===undefined){ // ratio = 가로/세로
      const H=dist(U,D),W=p3*H,px=(U.y-D.y)/H,py=-(U.x-D.x)/H,C=this.nodes.get(c);
      l=this._i++;r=this._i++;
      this.add_node(l,C.x-px*W/2,C.y-py*W/2,true);this.add_node(r,C.x+px*W/2,C.y+py*W/2,true);
    }
    [up,down,l,r].forEach(n=>this.add_link(c,n,true));
    [[up,r],[r,down],[down,l],[l,up]].forEach(([a,b])=>this.add_link(a,b,true));
    this.circles.push({up,down,left:l,right:r,c});this.links.slice(-8).forEach(k=>k.circle=this.circles.length-1);
  }
  add_shape(list){this.shapes.push(list);}
  add_rule(type,n1,n2,n3){
    const [a,b,c]=[n1,n2,n3].map(i=>this.nodes.get(i));let s={};
    if(type==='F')s.angle=wrap(Math.atan2(c.y-b.y,c.x-b.x)-Math.atan2(a.y-b.y,a.x-b.x));
    if(type==='R'){ // 보정: n3를 n1 대칭 위치로
      s.L1=dist(a,b);s.L3=dist(c,b);s.l1=s.L1;s.l3=s.L3;const u=[(a.x-b.x)/s.L1,(a.y-b.y)/s.L1];
      c.x=b.x-u[0]*s.L3;c.y=b.y-u[1]*s.L3;
    }
    if(type==='A'){
      const dx=c.x-b.x,dy=c.y-b.y;s.t=Math.max(0,Math.min(1,((a.x-b.x)*dx+(a.y-b.y)*dy)/(dx*dx+dy*dy)));
      a.x=b.x+dx*s.t;a.y=b.y+dy*s.t;s.tLocked=false;
    }
    this.rules.push({type,n:[n1,n2,n3],s});
  }
  join(type,a,b,ref,far){ // 관절/용접: 자식 오브젝트를 부모 노드 위치로 이동 후 연결
    const A=this.nodes.get(a),B=this.nodes.get(b);if(A.obj===B.obj)return false;
    const dx=A.x-B.x,dy=A.y-B.y;
    for(const n of this.nodes.values())if(n.obj===B.obj&&!n.fixed){n.x+=dx;n.y+=dy;}
    const j={type,a,b,bi:false,off:false};
    if(type==='weld'){const f=this.nodes.get(far),r=this.nodes.get(ref);
      Object.assign(j,{far,ref,angle:wrap(Math.atan2(f.y-B.y,f.x-B.x)-Math.atan2(r.y-B.y,r.x-B.x))});}
    this.joints.push(j);return true;
  }
  dup(o){
    const no=o+'+',m=new Map(),ns=[...this.nodes.values()].filter(n=>n.obj===o);
    ns.forEach(n=>{const id=this._i++;m.set(n.id,id);this.nodes.set(id,{...n,id,x:n.x+40,y:n.y+40,obj:no,fixed:false});});
    const cm=new Map();
    this.circles.filter(c=>m.has(c.c)).forEach(c=>{cm.set(this.circles.indexOf(c),this.circles.length);
      this.circles.push({up:m.get(c.up),down:m.get(c.down),left:m.get(c.left),right:m.get(c.right),c:m.get(c.c)});});
    this.links.filter(l=>m.has(l.a)&&m.has(l.b)).forEach(l=>this.links.push({...l,a:m.get(l.a),b:m.get(l.b),circle:cm.get(l.circle)}));
    this.shapes.filter(s=>s.every(i=>m.has(i))).forEach(s=>this.shapes.push(s.map(i=>m.get(i))));
    this.rules.filter(r=>r.n.every(i=>m.has(i))).forEach(r=>this.rules.push({type:r.type,n:r.n.map(i=>m.get(i)),s:{...r.s}}));
  }
  serialize(){return JSON.stringify({...this,nodes:[...this.nodes]});}
  static from(t){const o=JSON.parse(t),s=new Scene();Object.assign(s,o);s.nodes=new Map(o.nodes);return s;}
  apply(res){ // 이동 결과 반영
    if(res.pos)for(const [id,p] of res.pos){const n=this.nodes.get(id);n.x=p.x;n.y=p.y;}
    if(res.rules)res.rules.forEach((s,i)=>this.rules[i].s=s);
  }
  isLocked(id){return this.nodes.get(id).fixed||this.rules.some(r=>r.type==='A'&&r.n[0]===id&&r.s.tLocked);}
  toggleFix(id){
    const r=this.rules.find(r=>r.type==='A'&&r.n[0]===id);
    if(r)r.s.tLocked=!r.s.tLocked;else{const n=this.nodes.get(id);n.fixed=!n.fixed;}
  }
}

/* ============ MOVE CONTROLLER (순수 함수, DOM 무관) ============ */
const MoveController={
  // req: {kind:'node',id,target,free} | {kind:'link',i,t,target}
  solve(sc,req){
    const P=new Map();
    for(const n of sc.nodes.values())P.set(n.id,{x:n.x,y:n.y,w:n.fixed?0:1,lw:n.fixed?0:1,fx:n.fixed});
    const RS=sc.rules.map(r=>({...r.s}));
    const drag=[];
    const none={pos:P,rules:RS,violated:false};
    if(req.kind==='node'){
      if(sc.isLocked(req.id))return none;
      drag.push({id:req.id,...req.target});
    }else if(req.kind==='obj'){
      const ns=[...sc.nodes.values()].filter(n=>n.obj===req.obj);
      if(ns.some(n=>n.fixed))return none;
      ns.forEach(n=>drag.push({id:n.id,x:n.x+req.dx,y:n.y+req.dy}));
    }else{
      const l=sc.links[req.i],a=P.get(l.a),b=P.get(l.b);
      if(a.fx&&b.fx)return none;
      if(!a.fx&&!b.fx){ // 평행이동
        const dx=req.target.x-(a.x+(b.x-a.x)*req.t),dy=req.target.y-(a.y+(b.y-a.y)*req.t);
        drag.push({id:l.a,x:a.x+dx,y:a.y+dy},{id:l.b,x:b.x+dx,y:b.y+dy});
      }else{ // 한쪽 고정: 회전 이동
        const [pv,fid]=a.fx?[a,l.b]:[b,l.a];
        const dx=req.target.x-pv.x,dy=req.target.y-pv.y,m=Math.hypot(dx,dy)||1e-6;
        drag.push({id:fid,x:pv.x+dx/m*l.L,y:pv.y+dy/m*l.L});
      }
    }
    const ids=new Set(drag.map(d=>d.id)),DM=new Map(drag.map(d=>[d.id,d]));
    const free=req.kind==='node'&&req.free;
    let circ=null; // Alt + 원 노드: 중심 기준 대칭 + 다른 축 수직
    if(free){
      const d0=drag[0],ci=sc.circles.find(c=>[c.up,c.down,c.left,c.right].includes(d0.id));
      if(ci){
        const opp={[ci.up]:ci.down,[ci.down]:ci.up,[ci.left]:ci.right,[ci.right]:ci.left}[d0.id];
        const horiz=d0.id===ci.left||d0.id===ci.right,[ia,ib]=horiz?[ci.up,ci.down]:[ci.left,ci.right];
        const c=P.get(ci.c),p0=P.get(d0.id),A=P.get(ia),B=P.get(ib);
        if(!P.get(opp).fx){
          const fl=(x,y)=>sc.links.find(l=>(l.a===x&&l.b===y)||(l.a===y&&l.b===x)).Lmax;
          const m0=Math.hypot(p0.x-c.x,p0.y-c.y)||1e-6,nx=-(p0.y-c.y)/m0,ny=(p0.x-c.x)/m0;
          const sg=r=>Math.sign((r.x-c.x)*nx+(r.y-c.y)*ny)||1;
          circ={c:ci.c,d:d0.id,opp,ia,ib,l1:fl(ci.c,d0.id),l3:fl(ci.c,opp),idx:sc.circles.indexOf(ci),perp:!A.fx&&!B.fx,
            ka:Math.hypot(A.x-c.x,A.y-c.y),kb:Math.hypot(B.x-c.x,B.y-c.y),ga:sg(A),gb:sg(B)};
        }
      }
    }
    for(const d of drag)P.get(d.id).w=0;
    const sliders=free?new Set():new Set(sc.rules.filter(r=>r.type==='A'&&ids.has(r.n[0])).map(r=>r.n[0]));
    const R0=sc.rules.map(r=>{ // R: 시작 시점 방향/스케일
      if(r.type!=='R')return null;const a=P.get(r.n[0]),b=P.get(r.n[1]);
      const v=Math.hypot(a.x-b.x,a.y-b.y)||1e-6;return{ux:(a.x-b.x)/v,uy:(a.y-b.y)/v,s:Math.min(v/r.s.L1,1)};
    });
    const rot=(c,p,ang)=>{const co=Math.cos(ang),si=Math.sin(ang),x=p.x-c.x,y=p.y-c.y;p.x=c.x+x*co-y*si;p.y=c.y+x*si+y*co;};
    const DR=drag.filter(d=>!sliders.has(d.id)).map(d=>({p:P.get(d.id),x:d.x,y:d.y}));
    const LK=sc.links.map(l=>({A:P.get(l.a),B:P.get(l.b),L:l.L,Lmax:l.Lmax,soft:free&&(!l.internal||(circ&&l.circle===circ.idx))})).filter(l=>l.A.lw+l.B.lw>0);
    const ord=sc.rules.map((_,k)=>k).sort((x,y)=>'FRA'.indexOf(sc.rules[x].type)-'FRA'.indexOf(sc.rules[y].type));
    const RR=ord.map(k=>{const r=sc.rules[k],[i1,i2,i3]=r.n;return{k,r,i1,i2,i3,a:P.get(i1),b:P.get(i2),c:P.get(i3),st:RS[k]};});
    const NL=[...P.values()],OX=new Float64Array(NL.length),OY=new Float64Array(NL.length),EPS2=1e-5;
    for(let i=0;i<NL.length;i++){OX[i]=NL[i].x;OY[i]=NL[i].y;}
    for(let it=0;it<60;it++){
      if(it<45)for(const d of DR){d.p.x=d.x;d.p.y=d.y;}
      // 우선순위 낮은 것부터: F → R → A → 링크 (링크가 마지막에 길이 보장)
      if(circ&&it<45){
        const c=P.get(circ.c),p=P.get(circ.d),q=P.get(circ.opp);
        const vx=p.x-c.x,vy=p.y-c.y,m=Math.hypot(vx,vy)||1e-6,ux=vx/m,uy=vy/m;
        const e=Math.min(m/circ.l1,1)*circ.l3;q.x=c.x-ux*e;q.y=c.y-uy*e;
        if(circ.perp){const A=P.get(circ.ia),B=P.get(circ.ib);
          A.x=c.x-uy*circ.ga*circ.ka;A.y=c.y+ux*circ.ga*circ.ka;B.x=c.x-uy*circ.gb*circ.kb;B.y=c.y+ux*circ.gb*circ.kb;}
      }
      RR.forEach(({k,r,i1,i2,i3,a,b,c,st})=>{
        if(r.type==='F'){
          if(ids.has(i1))return;
          const cur=wrap(Math.atan2(c.y-b.y,c.x-b.x)-Math.atan2(a.y-b.y,a.x-b.x)),dl=wrap(st.angle-cur),W=a.w+c.w;
          if(!W)return;rot(b,a,-dl*a.w/W);rot(b,c,dl*c.w/W);
        }else if(r.type==='R'){
          if(!b.fx&&c.fx!==a.fx){ // 한쪽 끝 고정: n2가 비율 지점으로 이동, 자유단은 일직선 유지
            const cf=c.fx,p=cf?c:a,q=cf?a:c,pl=cf?st.L3:st.L1,ql=cf?st.L1:st.L3;
            if(ids.has(q===a?i1:i3)){b.x+=(p.x+(q.x-p.x)*pl/(pl+ql)-b.x)*0.5;b.y+=(p.y+(q.y-p.y)*pl/(pl+ql)-b.y)*0.5;}
            const db=Math.hypot(b.x-p.x,b.y-p.y);
            if(db>pl){b.x=p.x+(b.x-p.x)*pl/db;b.y=p.y+(b.y-p.y)*pl/db;}
            q.x=b.x+(b.x-p.x)*ql/pl;q.y=b.y+(b.y-p.y)*ql/pl;
            st.l1=Math.hypot(a.x-b.x,a.y-b.y);st.l3=Math.hypot(c.x-b.x,c.y-b.y);
            return;
          }
          let ux,uy,s;
          if(ids.has(i3)&&!ids.has(i1)){const v=Math.hypot(c.x-b.x,c.y-b.y)||1e-6;ux=-(c.x-b.x)/v;uy=-(c.y-b.y)/v;s=Math.min(v/st.L3,1);}
          else if(ids.has(i2)&&!ids.has(i1)){ux=R0[k].ux;uy=R0[k].uy;s=R0[k].s;}
          else{const v=Math.hypot(a.x-b.x,a.y-b.y)||1e-6;ux=(a.x-b.x)/v;uy=(a.y-b.y)/v;s=Math.min(v/st.L1,1);}
          s=Math.max(s,0.02);const lead=ids.has(i1)||ids.has(i3),e1=lead?s*st.L1:st.l1,e3=lead?s*st.L3:st.l3;
          if(!a.fx){a.x=b.x+ux*e1;a.y=b.y+uy*e1;}
          if(!c.fx){c.x=b.x-ux*e3;c.y=b.y-uy*e3;}
        }else if(r.type==='A'){
          const dx=c.x-b.x,dy=c.y-b.y,q=dx*dx+dy*dy||1e-6;
          if(sliders.has(i1)&&!st.tLocked){
            const d=DM.get(i1);
            st.t=Math.max(0,Math.min(1,((d.x-b.x)*dx+(d.y-b.y)*dy)/q));
          }
          if(!sliders.has(i1)&&!st.tLocked){ // n1 자유: 현재 위치를 지키며 선분에 가장 가까운 점으로 이동
            st.t=Math.max(0,Math.min(1,((a.x-b.x)*dx+(a.y-b.y)*dy)/q));
            if(!a.fx){a.x=b.x+dx*st.t;a.y=b.y+dy*st.t;return;}
          }
          const t=st.t,tx=b.x+dx*t,ty=b.y+dy*t;
          if(sliders.has(i1)){a.x=tx;a.y=ty;return;}
          const cx=tx-a.x,cy=ty-a.y,S=a.w+(1-t)**2*b.w+t*t*c.w;
          if(!S)return;
          a.x+=a.w*cx/S;a.y+=a.w*cy/S;
          b.x-=(1-t)*b.w*cx/S;b.y-=(1-t)*b.w*cy/S;
          c.x-=t*c.w*cx/S;c.y-=t*c.w*cy/S;
        }
      });
      for(const j of sc.joints){ // 관절/용접 (부모 -> 자식 방향, bi면 양방향)
        if(j.off)continue;
        const a=P.get(j.a),b=P.get(j.b),wa=j.bi?a.w:0,wb=b.fx?0:(j.bi?b.w:1),W=wa+wb;
        if(W){const dx=a.x-b.x,dy=a.y-b.y;a.x-=dx*wa/W;a.y-=dy*wa/W;b.x+=dx*wb/W;b.y+=dy*wb/W;}
        if(j.type==='weld'){
          const fa=P.get(j.far),rf=P.get(j.ref),wf=fa.fx?0:(j.bi?fa.w:1),wr=j.bi?rf.w:0,V=wf+wr;
          if(V){const dl=wrap(j.angle-wrap(Math.atan2(fa.y-b.y,fa.x-b.x)-Math.atan2(rf.y-b.y,rf.x-b.x)));
            rot(b,fa,dl*wf/V);rot(b,rf,-dl*wr/V);}
        }
      }
      for(const l of LK){
        const a=l.A,b=l.B,w=a.lw+b.lw,dx=b.x-a.x,dy=b.y-a.y,m=Math.sqrt(dx*dx+dy*dy)||1e-6,T=l.soft?l.Lmax:l.L;
        if(l.soft&&m<=T)continue;
        const k=(m-T)/m,ka=k*a.lw/w,kb=k*b.lw/w;a.x+=dx*ka;a.y+=dy*ka;b.x-=dx*kb;b.y-=dy*kb;
      }
      let mx=0; // 이번 반복에서 노드 최대 이동량(제곱)이 충분히 작으면 수렴으로 판단
      for(let i=0;i<NL.length;i++){const p=NL[i],ex=p.x-OX[i],ey=p.y-OY[i],dd=ex*ex+ey*ey;if(dd>mx)mx=dd;OX[i]=p.x;OY[i]=p.y;}
      if(mx<EPS2){if(it<45)it=44;else break;}
    }
    let violated=false;
    for(const l of sc.links){const a=P.get(l.a),b=P.get(l.b);if(Math.abs(dist(a,b)-l.L)>2)violated=true;}
    return {pos:P,rules:RS,violated};
  },
  // 이동 종료: F규칙은 n1이 이동점이었을 때만 각도 저장
  end(sc,ids){
    return sc.rules.map(r=>{
      if(r.type==='R'&&(ids.includes(r.n[0])||ids.includes(r.n[2]))){const [a,b,c]=r.n.map(i=>sc.nodes.get(i));return{...r.s,l1:dist(a,b),l3:dist(c,b)};}
      if(r.type!=='F'||!ids.includes(r.n[0]))return r.s;
      const [a,b,c]=r.n.map(i=>sc.nodes.get(i));
      return {angle:wrap(Math.atan2(c.y-b.y,c.x-b.x)-Math.atan2(a.y-b.y,a.x-b.x))};
    });
  }
};

/* ============ VIEW (Canvas2D 렌더러) ============ */
class CanvasView{
  constructor(cv){this.cv=cv;this.g=cv.getContext('2d');this.C={};this.resize();
    const inv=()=>{this.C={};this.onChange&&this.onChange();};
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change',inv);
    new MutationObserver(inv).observe(document.documentElement,{attributes:true});}
  resize(){const d=devicePixelRatio||1,r=this.rect=this.cv.getBoundingClientRect();
    this.cv.width=r.width*d;this.cv.height=r.height*d;this.g.setTransform(d,0,0,d,0,0);}
  render(sc,hot,pk=[]){
    const g=this.g,col=n=>this.C[n]||(this.C[n]=getComputedStyle(document.documentElement).getPropertyValue(n).trim());
    g.clearRect(0,0,this.cv.width,this.cv.height);
    const N=i=>sc.nodes.get(i);
    g.fillStyle=col('--shape');g.strokeStyle=col('--line');g.lineWidth=1.5;
    for(const s of sc.shapes){g.beginPath();s.forEach((i,k)=>k?g.lineTo(N(i).x,N(i).y):g.moveTo(N(i).x,N(i).y));g.closePath();g.fill();}
    for(const c of sc.circles){
      const C=N(c.c),L=N(c.left),R=N(c.right),U=N(c.up),D=N(c.down);
      g.beginPath();g.ellipse(C.x,C.y,dist(L,R)/2,dist(U,D)/2,Math.atan2(R.y-L.y,R.x-L.x),0,7);g.stroke();
    }
    g.lineWidth=2;
    for(const l of sc.links){if(l.internal)continue;g.beginPath();g.moveTo(N(l.a).x,N(l.a).y);g.lineTo(N(l.b).x,N(l.b).y);g.stroke();}
    g.lineWidth=1;g.setLineDash([4,4]);
    for(const r of sc.rules){ // 규칙 보조선
      if(r.type==='R'){g.beginPath();g.moveTo(N(r.n[0]).x,N(r.n[0]).y);g.lineTo(N(r.n[2]).x,N(r.n[2]).y);g.stroke();}
    }
    g.setLineDash([]);
    g.lineWidth=1.5;g.strokeStyle=col('--acc');
    for(const j of sc.joints){const B=N(j.b);g.beginPath();
      if(j.type==='pin')g.arc(B.x,B.y,10,0,7);else g.rect(B.x-9,B.y-9,18,18);g.stroke();
      if(j.type==='weld'){g.setLineDash([3,3]);g.beginPath();g.moveTo(B.x,B.y);g.lineTo(N(j.ref).x,N(j.ref).y);g.stroke();g.setLineDash([]);}}
    const sl=new Set(sc.rules.filter(r=>r.type==='A').map(r=>r.n[0]));
    g.font='10px system-ui';
    for(const n of sc.nodes.values()){
      if(n.internal)continue;
      g.beginPath();g.arc(n.x,n.y,5,0,7);
      g.fillStyle=sc.isLocked(n.id)?col('--fix'):col('--line');g.fill();
      if(sl.has(n.id)||n.id===hot||pk.includes(n.id)){g.strokeStyle=col('--acc');g.lineWidth=2;g.beginPath();g.arc(n.x,n.y,8,0,7);g.stroke();}
      g.fillStyle=col('--mut');g.fillText(n.id,n.x+9,n.y-7);
    }
  }
}

/* ============ INPUT ADAPTER + INTERACTION ============ */
class Editor{
  constructor(cv,sc){
    this.sc=sc;this.view=new CanvasView(cv);this.cv=cv;this.freeBtn=false;this.alt=false;this.d=null;this.last=null;this.mode='move';this.pick=[];
    cv.addEventListener('pointerdown',e=>this.down(e));
    cv.addEventListener('pointermove',e=>this.move(e));
    cv.addEventListener('pointerup',e=>this.up(e));
    cv.addEventListener('pointercancel',e=>this.up(e));
    this.view.onChange=()=>this.draw();new ResizeObserver(()=>{this.view.resize();this.draw();}).observe(cv);
    this.draw();
  }
  pos(e){const r=this.view.rect;return{x:e.clientX-r.left,y:e.clientY-r.top};}
  hitNode(p,tol){let best=null,bd=tol;for(const n of this.sc.nodes.values()){if(n.internal)continue;const d=dist(n,p);if(d<bd){bd=d;best=n.id;}}return best;}
  hitLink(p,tol){
    let best=-1,bd=tol;
    this.sc.links.forEach((l,i)=>{
      if(l.internal)return;const a=this.sc.nodes.get(l.a),b=this.sc.nodes.get(l.b),dx=b.x-a.x,dy=b.y-a.y;
      const t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy||1)));
      const d=Math.hypot(p.x-a.x-dx*t,p.y-a.y-dy*t);if(d<bd){bd=d;best=i;}
    });
    return best;
  }
  down(e){
    this.view.rect=this.cv.getBoundingClientRect();const p=this.pos(e),tol=e.pointerType==='touch'?18:10,now=performance.now();
    const n=this.hitNode(p,tol);if(this.mode!=='move'&&this.mode!=='obj'){if(n!==null)this.tool(n);return;}
    if(n!==null){
      if(this.last&&this.last.id===n&&now-this.last.t<350){this.sc.toggleFix(n);this.last=null;this.draw();return;}
      this.last={id:n,t:now};const ob=this.sc.nodes.get(n).obj;this.d=this.mode==='obj'?{kind:'obj',obj:ob,id:n,last:p,ids:[...this.sc.nodes.values()].filter(x=>x.obj===ob).map(x=>x.id)}:{kind:'node',id:n,ids:[n]};
    }else{
      const i=this.hitLink(p,6);this.last=null;if(i<0)return;
      const l=this.sc.links[i],a=this.sc.nodes.get(l.a),b=this.sc.nodes.get(l.b),dx=b.x-a.x,dy=b.y-a.y;
      const t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy||1)));
      this.d={kind:'link',i,t,ids:[l.a,l.b]};
    }
    this.cv.setPointerCapture(e.pointerId);
  }
  move(e){
    if(!this.d)return;
    this.pt={clientX:e.clientX,clientY:e.clientY,altKey:e.altKey};
    if(!this.raf)this.raf=requestAnimationFrame(()=>{this.raf=0;this.flush();});
  }
  flush(){
    if(this.raf){cancelAnimationFrame(this.raf);this.raf=0;}
    if(this.d&&this.pt){const e=this.pt;this.pt=null;this.step(e);}
  }
  step(e){
    if(!this.d)return;
    const free=e.altKey||this.freeBtn;this.d.free=free;
    const tp=this.pos(e),req={...this.d,target:tp,free};if(this.d.kind==='obj'){req.dx=tp.x-this.d.last.x;req.dy=tp.y-this.d.last.y;this.d.last=tp;}
    const res=MoveController.solve(this.sc,req);this.d.sym=res.sym;this.sc.apply(res);this.draw();
  }
  up(){this.flush();
    if(!this.d)return;
    if(this.d.free&&this.d.kind==='node'){const N=this.sc.nodes;
      for(const l of this.sc.links)l.L=dist(N.get(l.a),N.get(l.b));}
    this.sc.apply({rules:MoveController.end(this.sc,this.d.ids)});this.d=null;this.draw();
  }
  msg(t){document.getElementById('hint').textContent=t;}
  setMode(k){this.mode=k;this.pick=[];this.msg(HINT[k]);this.draw();}
  tool(n){
    const m=this.mode,sc=this.sc;
    if(m==='cut'){sc.joints=sc.joints.filter(j=>![j.a,j.b,j.far,j.ref].includes(n));}
    else if(m==='dup'){sc.dup(sc.nodes.get(n).obj);}
    else{
      this.pick.push(n);const steps=m==='pin'?['부모 노드','자식 노드']:['부모 노드','자식 노드','기준 노드(부모 쪽)','각도 노드(자식 쪽)'];
      if(this.pick.length<steps.length){this.msg(steps[this.pick.length]+'를 클릭');}
      else{const [a,b,ref,far]=this.pick;this.pick=[];
        this.msg(sc.join(m,a,b,ref,far)?'연결됨':'서로 다른 오브젝트의 노드를 선택하세요');}
    }
    this.draw();
  }
  draw(){this.view.render(this.sc,this.d&&this.d.id,this.pick);}
}

/* ============ 데모 장면 (스크립트 API) ============ */
function buildDemo(){
  const s=new Scene();s.obj='A';
  s.add_node(1,300,70);s.add_node(2,300,115);s.add_circle(1,2,1);      // 머리 (가로/세로 = 1)
  s.add_node(3,300,230);s.add_link(2,3);                                // 몸통
  s.add_node(4,300,150);s.add_rule('A',4,2,3);                          // 어깨: 몸통 위 슬라이더
  s.add_node(5,225,150);s.add_node(6,365,150);s.add_rule('R',5,4,6);    // 팔: 180° 대칭, 길이 비율 연동
  s.add_node(7,255,320);s.add_node(8,345,320);
  s.add_link(3,7);s.add_link(3,8);s.add_rule('F',7,3,8);                // 다리 사이각 유지
  s.obj='B';s.add_node(10,470,330);s.add_node(11,570,330);s.add_node(12,670,330);
  s.add_node(13,520,250);s.add_node(14,620,250);                        // 트러스
  [[10,11],[11,12],[10,13],[13,11],[13,14],[11,14],[14,12]].forEach(l=>s.add_link(...l));
  s.add_shape([10,12,14,13]);
  s.add_node(20,520,70);s.add_node(21,520,150);s.add_node(22,480,110);s.add_node(23,560,110);
  s.add_circle(20,21,22,23);s.add_link(21,13);                          // 4노드 원
  s.obj='C';s.add_node(30,60,330);s.add_node(31,60,290);s.add_node(32,60,200); // 칼: 30-31 손잡이, 31-32 칼날
  s.add_link(30,31);s.add_link(31,32);s.add_rule('F',30,31,32);
  s.join('weld',5,31,4,32);                                              // 손(5)에 용접, 기준 노드 4
  return s;
}
const cv=document.getElementById('cv');
let ed=new Editor(cv,buildDemo());
const fb=document.getElementById('free');
fb.onclick=()=>{ed.freeBtn=!ed.freeBtn;fb.setAttribute('aria-pressed',ed.freeBtn);};
const HINT={move:'노드·링크 드래그 / 더블클릭(탭)으로 고정 / Alt 또는 버튼으로 자유 이동',obj:'노드를 드래그하면 오브젝트 전체가 이동',pin:'부모 노드 → 자식 노드 순서로 클릭',weld:'부모 노드 → 자식 노드 → 기준 노드(부모 쪽) → 각도 노드(자식 쪽) 순서로 클릭',cut:'연결에 속한 노드를 클릭하면 해제',dup:'복제할 오브젝트의 노드를 클릭'};
const modes=[['move','이동'],['obj','오브젝트 이동'],['pin','관절'],['weld','용접'],['cut','연결 해제'],['dup','복제']];
const mb=document.getElementById('modes');
modes.forEach(([k,t])=>{const b=document.createElement('button');b.textContent=t;b.setAttribute('aria-pressed',k==='move');
  b.onclick=()=>{ed.setMode(k);[...mb.children].forEach((c,i)=>c.setAttribute('aria-pressed',modes[i][0]===k));};mb.appendChild(b);});
document.getElementById('save').onclick=()=>{try{localStorage.setItem('pose',ed.sc.serialize());ed.msg('저장됨');}catch(e){ed.msg('저장 실패');}};
document.getElementById('load').onclick=()=>{try{const t=localStorage.getItem('pose');if(t){ed.sc=Scene.from(t);ed.draw();ed.msg('불러옴');}else ed.msg('저장된 데이터 없음');}catch(e){ed.msg('불러오기 실패');}};
document.getElementById('reset').onclick=()=>{ed.sc=buildDemo();ed.draw();};
</script></body></html>
