import * as THREE from 'three';
import {OrbitControls} from './lib/OrbitControls.js';
const $=id=>document.getElementById(id), view=$('viewport');
let renderer;
try { renderer=new THREE.WebGLRenderer({antialias:true,alpha:true}); } catch(e) { $('loading').textContent='WebGL을 사용할 수 없습니다. 브라우저 하드웨어 가속을 확인하세요.'; throw e; }
renderer.setPixelRatio(Math.min(devicePixelRatio,2));view.appendChild(renderer.domElement);
const scene=new THREE.Scene(), camera=new THREE.PerspectiveCamera(45,1,.01,100);
camera.up.set(0,0,1);camera.position.set(2,-2,1.5);
const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,.45);controls.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xc5e4ff,0x526172,2.4));const sun=new THREE.DirectionalLight(0xffffff,3);sun.position.set(3,-4,6);scene.add(sun);
const grid=new THREE.GridHelper(10,50,0x46647c,0x213247);grid.rotation.x=Math.PI/2;grid.position.z=.002;scene.add(grid);
let group=new THREE.Group();scene.add(group);let objects=new Map(), socket, meta, state, history=[], log=[], timers=new Set(), generation=0, received=0, rateTime=performance.now();
const matrix=new THREE.Matrix4(), rotation=new THREE.Quaternion();
// 로봇 크기별 카메라 시점(target=바라보는 점, position=카메라 위치, 단위 m)
const CAMERA_PRESETS={
 go2:{target:[1.0,.4,.2],position:[-.9,-2.0,1.6]},
 rby1:{target:[.3,-.1,.85],position:[2.0,-1.7,1.7]},
 g1:{target:[.8,.6,.5],position:[-.9,-2.2,2.1]},
 allegro:{target:[0,0,.08],position:[.35,-.35,.3]},
 default:{target:[0,0,.45],position:[2,-2,1.5]},
};
const ROBOT_NAMES={go2:'Unitree Go2',rby1:'Rainbow Robotics RB-Y1',g1:'Unitree G1',allegro:'Wonik Allegro Hand'};
// Allegro 관절 이름 해석: 손가락 접두어 + 관절 번호(0=옆벌림, 1~3=기저→끝 굽힘, 엄지는 0=회전)
const FINGER_NAMES={ff:'검지',mf:'중지',rf:'약지',th:'엄지'};
const FINGER_JOINT_NAMES=['옆벌림','기저 굽힘','중간 굽힘','끝 굽힘'];
const THUMB_JOINT_NAMES=['회전','옆벌림','기저 굽힘','끝 굽힘'];
function jointLabel(robot,name){
 if(robot==='allegro'){const match=/^(ff|mf|rf|th)j(\d)$/.exec(name);if(match){const names=match[1]==='th'?THUMB_JOINT_NAMES:FINGER_JOINT_NAMES;return `${FINGER_NAMES[match[1]]} · ${names[Number(match[2])]??match[2]}`;}}
 if(robot==='rby1'){
  let match=/^torso_(\d)$/.exec(name);if(match)return `몸통 ${Number(match[1])+1}`;
  match=/^(right|left)_arm_(\d)$/.exec(name);if(match)return `${match[1]==='right'?'오른팔':'왼팔'} ${Number(match[2])+1}`;
  match=/^gripper_finger_(r|l)1$/.exec(name);if(match)return `${match[1]==='r'?'오른쪽':'왼쪽'} 그리퍼`;
  if(name==='head_0')return '머리 좌우';if(name==='head_1')return '머리 상하';
 }
 return null;
}
// 손 자세 프리셋: 관절 이름 → 관절 범위 내 비율(0=최소, 1=최대). 목록에 없는 관절은 현재 목표 유지
const FLEX_JOINTS=['ffj1','ffj2','ffj3','mfj1','mfj2','mfj3','rfj1','rfj2','rfj3'];
const fill=(names,ratio)=>Object.fromEntries(names.map(n=>[n,ratio]));
const HAND_POSES={
 '손 펴기':{...fill(FLEX_JOINTS,.1),...fill(['ffj0','mfj0','rfj0'],.5),thj0:.1,thj1:.1,thj2:.1,thj3:.1},
 '주먹 쥐기':{...fill(FLEX_JOINTS,.9),...fill(['ffj0','mfj0','rfj0'],.5),thj0:1,thj1:.8,thj2:.9,thj3:.7},
 '집기(엄지+검지)':{ffj0:.5,ffj1:.55,ffj2:.45,ffj3:.4,...fill(['mfj1','mfj2','mfj3','rfj1','rfj2','rfj3'],.1),thj0:.95,thj1:.45,thj2:.45,thj3:.45},
 '브이(V)':{ffj0:.35,mfj0:.65,...fill(['ffj1','ffj2','ffj3','mfj1','mfj2','mfj3'],.1),...fill(['rfj1','rfj2','rfj3'],.9),thj0:.9,thj1:.5,thj2:.6,thj3:.6},
};
function applyHandPose(poseName){
 const pose=HAND_POSES[poseName];if(!pose||!meta)return;
 for(const joint of meta.joints){
  const ratio=pose[joint.name];if(ratio===undefined)continue;
  // 서버가 범위 밖 목표를 거절하므로 반드시 관절 한계 안으로 계산
  const target=Math.min(joint.max,Math.max(joint.min,joint.min+ratio*(joint.max-joint.min)));
  send({type:'joint',joint:joint.id,target});
 }
 send({type:'play'});record('POSE',poseName);
}
// 미션 카드: 모델 수신 시 구성(build), 상태 수신마다 요약·성공 표시 갱신(update)
function buildMission(mission){
 const card=$('missionCard');card.hidden=!mission;if(!mission)return;
 $('missionTitle').textContent=mission.title;$('missionGoal').textContent=mission.goal;$('missionSummary').textContent='';
 $('assistRow').hidden=!mission.has_assist;
 const steps=$('missionSteps');steps.replaceChildren();for(const step of mission.steps){const li=document.createElement('li');li.textContent=step;steps.append(li);}
 $('missionExamples').textContent=mission.examples.map(e=>JSON.stringify(e)).join('\n');
 const quick=$('missionQuick');quick.replaceChildren();
 for(const item of mission.quick){const button=document.createElement('button');button.textContent=item.label;button.title=item.commands.map(c=>JSON.stringify(c)).join('\n');button.onclick=()=>{for(const cmd of item.commands)send(cmd,['stop','pause','reset'].includes(cmd.type));};quick.append(button);}
}
function updateMission(status){
 if(!status)return;
 $('missionSummary').textContent=status.summary??'';
 const badge=$('missionBadge');badge.textContent=status.success?'성공 ✓':'진행 중';badge.classList.toggle('done',!!status.success);
 if(document.activeElement!==$('assist'))$('assist').checked=!!status.assist;
}
function buildPosePresets(robot){
 const box=$('posePresets');box.replaceChildren();box.hidden=robot!=='allegro';if(box.hidden)return;
 for(const poseName of Object.keys(HAND_POSES)){const button=document.createElement('button');button.textContent=poseName;button.onclick=()=>applyHandPose(poseName);box.append(button);}
}
function resize(){renderer.setSize(view.clientWidth,view.clientHeight);camera.aspect=view.clientWidth/view.clientHeight;camera.updateProjectionMatrix();}
new ResizeObserver(resize).observe(view);
function record(kind,payload){log.push({at:new Date().toISOString(),kind,payload});if(log.length>500)log.shift();$('log').textContent=log.slice(-35).map(x=>`${x.at.slice(11,23)} ${x.kind} ${JSON.stringify(x.payload)}`).join('\n');$('log').scrollTop=$('log').scrollHeight;}
function clearPending(){for(const t of timers)clearTimeout(t);timers.clear();generation++;}
function send(msg,immediate=false){if(demoMode){handleDemoCommand(msg);return;}if(!socket||socket.readyState!==WebSocket.OPEN){record('ERROR','서버 연결이 필요합니다');return;}
 if(immediate)clearPending();
 if(!immediate&&Math.random()*100<Number($('loss').value)){record('DROP',msg);return;}
 const epoch=generation, currentSocket=socket;
 const perform=()=>{if(epoch!==generation||currentSocket!==socket||socket.readyState!==WebSocket.OPEN)return;socket.send(JSON.stringify(msg));record('TX',msg);};
 const delay=immediate?0:Number($('delay').value);if(delay){const t=setTimeout(()=>{timers.delete(t);perform();},delay);timers.add(t);record('QUEUE',{delay,msg});}else perform();
}
function geometry(g,m){const s=g.size;switch(g.type){case 0:return new THREE.PlaneGeometry(12,12);case 2:return new THREE.SphereGeometry(s[0],16,12);case 3:{const geo=new THREE.CapsuleGeometry(s[0],s[1]*2,6,12);geo.rotateX(Math.PI/2);return geo;}case 4:return new THREE.SphereGeometry(1,16,12).scale(s[0],s[1],s[2]);case 5:{const geo=new THREE.CylinderGeometry(s[0],s[0],s[1]*2,16);geo.rotateX(Math.PI/2);return geo;}case 6:return new THREE.BoxGeometry(s[0]*2,s[1]*2,s[2]*2);case 7:{const mesh=m.meshes[g.mesh],geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(mesh.vertices,3));geo.setIndex(mesh.faces);geo.computeVertexNormals();return geo;}default:return null;}}
function build(m){meta=m;history=[];state=null;for(const o of objects.values()){o.geometry.dispose();o.material.dispose();}scene.remove(group);group=new THREE.Group();scene.add(group);objects.clear();
 for(const g of m.geoms){const geo=geometry(g,m);if(!geo)continue;const c=g.color,mat=new THREE.MeshStandardMaterial({color:new THREE.Color(c[0],c[1],c[2]),roughness:.65,metalness:.12,side:THREE.DoubleSide,transparent:c[3]<1,opacity:c[3]});const obj=new THREE.Mesh(geo,mat);group.add(obj);objects.set(g.id,obj);}
 $('robotName').textContent=ROBOT_NAMES[m.robot]??m.robot;buildPosePresets(m.robot);buildMission(m.mission);$('actuation').textContent=m.actuation;$('loading').style.display='none';$('joints').replaceChildren();$('chartJoint').replaceChildren();
 for(const j of m.joints){const row=document.createElement('div');row.className='joint';const title=document.createElement('div');title.className='jointtitle';const name=document.createElement('span');const label=jointLabel(m.robot,j.name);name.textContent=label??j.name;if(label){const raw=document.createElement('small');raw.textContent=j.name;name.append(raw);}const output=document.createElement('output');output.id=`out${j.id}`;title.append(name,output);const input=document.createElement('input');input.type='range';input.min=j.min;input.max=j.max;input.step=(j.max-j.min)/1000||.001;input.id=`joint${j.id}`;input.setAttribute('aria-label',j.name);input.addEventListener('input',()=>{output.textContent=`${Number(input.value).toFixed(3)} ${j.unit}`;send({type:'joint',joint:j.id,target:Number(input.value)});$('chartJoint').value=j.id;history=[];});row.append(title,input);$('joints').append(row);const option=document.createElement('option');option.value=j.id;option.textContent=label?`${label} (${j.name})`:j.name;$('chartJoint').append(option);}
 document.querySelectorAll('[data-robot]').forEach(b=>b.classList.toggle('active',b.dataset.robot===m.robot));const cameraPreset=CAMERA_PRESETS[m.robot]||CAMERA_PRESETS.default;controls.target.set(...cameraPreset.target);camera.position.set(...cameraPreset.position);controls.update();record('MODEL',{robot:m.robot,joints:m.joints.length});
}
function update(s){state=s;received++;const now=performance.now();if(now-rateTime>1000){$('hz').textContent=`${(received*1000/(now-rateTime)).toFixed(1)} Hz`;received=0;rateTime=now;record('RX',{seq:s.seq,time:s.time,joint:s.joints[0],contacts:s.contacts});}
 $('time').textContent=`${s.time.toFixed(2)} s`;$('contacts').textContent=s.contacts;$('anchor').checked=s.anchored;$('play').textContent=s.running?'▶ 실행 중':'▶ 실행';updateMission(s.mission);if(s.recording)renderRecording(s.recording);
 for(const [id,obj] of objects){const r=s.rotations[id],p=s.positions[id];matrix.set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);rotation.setFromRotationMatrix(matrix);obj.position.fromArray(p);obj.quaternion.copy(rotation);}
 for(const j of s.joints){const input=$(`joint${j.id}`);if(document.activeElement!==input){input.value=j.target;$(`out${j.id}`).textContent=j.target.toFixed(3);}}
 const selected=s.joints.find(j=>j.id===Number($('chartJoint').value));if(selected){history.push([selected.target,selected.q]);if(history.length>200)history.shift();$('telemetry').textContent=`실제 ${selected.q.toFixed(4)} · 속도 ${selected.dq.toFixed(4)} · 구동력 ${selected.torque.toFixed(3)} ${meta.joints.find(j=>j.id===selected.id).unit==='m'?'N':'N·m'} · 센서 ${s.sensors.length}개`;}drawChart();
}
function drawChart(){const c=$('chart'),ctx=c.getContext('2d');c.width=Math.max(200,c.clientWidth)*devicePixelRatio;c.height=130*devicePixelRatio;const w=c.width,h=c.height;ctx.clearRect(0,0,w,h);if(!history.length)return;const vals=history.flat(),lo=Math.min(...vals)-.1,hi=Math.max(...vals)+.1;ctx.strokeStyle='#293a50';ctx.lineWidth=1;for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke();}for(let k=0;k<2;k++){ctx.strokeStyle=k?'#77e7be':'#b696ff';ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();history.forEach((v,i)=>{const x=i*w/199,y=h-((v[k]-lo)/(hi-lo))*h;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}ctx.fillStyle='#9aaec6';ctx.font=`${10*devicePixelRatio}px sans-serif`;ctx.fillText(hi.toFixed(2),8*devicePixelRatio,14*devicePixelRatio);ctx.fillText(lo.toFixed(2),8*devicePixelRatio,h-6*devicePixelRatio);}
// 서버 주소 결정 순서: 직접 입력(저장) → 공유 링크 ?server= → GitHub에 게시된 현재 공개 서버 → (실습 서버가 직접 제공한 페이지면) 현재 주소
const SERVER_SETTINGS_KEY='robotLab.server.v2';
const LEGACY_SETTINGS_KEY='robotLab.server';   // 예전 버전은 바뀌는 터널 주소까지 저장했으므로 토큰만 이전
const IS_STATIC_HOSTING=location.hostname.endsWith('github.io');
// start_public.bat(public.py)가 실행될 때마다 터널 주소를 tunnel 브랜치의 server.json에 게시
const PUBLISHED_SERVER_URL='https://api.github.com/repos/MetaSonGi/robot-lab/contents/server.json?ref=tunnel';
const CLOSE_REASONS={4401:'접속 토큰이 올바르지 않습니다. 서버 설정에서 토큰을 확인하세요',4429:'동시 접속 인원이 가득 찼습니다. 잠시 후 다시 연결하세요'};
function loadServerSettings(){
 let saved={};
 try{
  saved=JSON.parse(localStorage.getItem(SERVER_SETTINGS_KEY)||'null');
  if(!saved){const legacy=JSON.parse(localStorage.getItem(LEGACY_SETTINGS_KEY)||'{}')||{};saved={server:'',token:legacy.token||''};localStorage.removeItem(LEGACY_SETTINGS_KEY);}
 }catch{saved={};}
 const params=new URLSearchParams(location.search);
 const settings={server:typeof saved.server==='string'?saved.server:'',token:params.get('token')??(typeof saved.token==='string'?saved.token:'')};
 // 링크의 서버 주소는 실행마다 바뀌므로 이번 접속에만 사용하고 저장하지 않음
 linkServer=params.get('server')??'';
 if(params.has('token'))saveServerSettings(settings);
 if(params.has('server')||params.has('token')){
  // 토큰이 주소창·공유 화면에 남지 않도록 URL에서 제거
  params.delete('server');params.delete('token');const rest=params.toString();
  window.history.replaceState(null,'',location.pathname+(rest?`?${rest}`:'')+location.hash);
 }
 return settings;
}
function saveServerSettings(settings){try{localStorage.setItem(SERVER_SETTINGS_KEY,JSON.stringify(settings));}catch{/* 저장 불가(사생활 모드)여도 이번 접속은 진행 */}}
async function fetchPublishedServer(){
 const response=await fetch(PUBLISHED_SERVER_URL,{cache:'no-store'});
 if(response.status===404)return null;
 if(!response.ok)throw new Error(`공개 서버 정보 조회 실패 (HTTP ${response.status})`);
 const file=await response.json();
 const published=JSON.parse(atob(String(file.content||'').replace(/\s/g,''))||'{}');
 return typeof published.server==='string'&&published.server?published.server:null;
}
async function resolveServerAddress(){
 if(serverSettings.server)return serverSettings.server;
 if(linkServer)return linkServer;
 if(!IS_STATIC_HOSTING)return '';
 return fetchPublishedServer();
}
function buildSocketUrl(server,token){
 let url;
 if(server.trim()){
  let address=server.trim();
  // 스킴 생략 시: 로컬 주소는 http, 그 외(터널 주소)는 https로 간주
  if(!/^[a-z]+:\/\//i.test(address))address=(/^(localhost|127\.|\[::1\])/i.test(address)?'http://':'https://')+address;
  url=new URL(address);
  const schemes={'http:':'ws:','https:':'wss:','ws:':'ws:','wss:':'wss:'};
  if(!schemes[url.protocol])throw new Error(`지원하지 않는 주소 형식: ${url.protocol}`);
  url.protocol=schemes[url.protocol];url.pathname='/ws';url.search='';url.hash='';
 }else if(!IS_STATIC_HOSTING){
  url=new URL(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);
 }else return null;
 if(token.trim())url.searchParams.set('token',token.trim());
 return url.toString();
}
// ── 데모 모드: 서버 없이 미리 녹화한 미션 자동 풀이(tools/record_demos.py)를 보간 재생 ──
const DEMO_HOLD_AT_END_S=2.5;
const demoCache=new Map();
let demoMode=false,demo=null,demoRequest=0,demoRobot='go2';
async function loadDemo(robot){
 if(!demoCache.has(robot)){
  const response=await fetch(`demo/${robot}.json`);
  if(!response.ok)throw new Error(`데모 파일이 없습니다 (${robot}, HTTP ${response.status})`);
  const data=await response.json();
  if(data.version!==1||!Array.isArray(data.frames)||data.frames.length<2)throw new Error('데모 파일 형식이 올바르지 않습니다');
  demoCache.set(robot,data);
 }
 return demoCache.get(robot);
}
function quatToRowMatrix(w,x,y,z){return[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];}
function stopLiveConnection(){clearPending();connectAttempt++;apiBase=null;lastRecordRunId=null;if(socket){socket.onclose=null;socket.close();socket=null;}}
async function startDemo(robot=demoRobot,reason=''){
 stopLiveConnection();demoMode=true;demoRobot=robot;const request=++demoRequest;demo=null;
 $('connection').textContent='● 데모 모드';$('demoReason').textContent=reason;$('demoBanner').hidden=false;
 $('loading').style.display='';$('loading').textContent='데모 불러오는 중…';
 lastRecordText='';$('recordStatus').textContent='데모 모드 · 수집 안 함';$('recordStatus').classList.remove('live');refreshDataList();
 try{
  const data=await loadDemo(robot);
  if(request!==demoRequest)return;   // 그사이 다른 로봇/실시간 연결을 고른 경우
  build(data.model);
  const positions=[],rotations=[];
  data.geom_ids.forEach((id,i)=>{const p=data.initial_poses[i];positions[id]=p.slice(0,3);rotations[id]=quatToRowMatrix(p[3],p[4],p[5],p[6]);});
  demo={data,positions,rotations,playhead:0,lastTick:performance.now(),playing:true,commandIndex:0,seq:0};
  record('DEMO',{robot,seconds:+((data.frames.length-1)/data.fps).toFixed(1),commands:data.commands.length,note:'녹화된 JSON 명령이 시간에 맞춰 TX/ACK로 표시됩니다'});
 }catch(e){if(request!==demoRequest)return;$('loading').textContent=`데모를 불러오지 못했습니다: ${e.message}`;record('ERROR',e.message);}
}
function stopDemo(){demoMode=false;demo=null;demoRequest++;$('demoBanner').hidden=true;}
function restartDemo(){demo.playhead=0;demo.commandIndex=0;history=[];record('DEMO','처음부터 다시 재생');}
function handleDemoCommand(msg){
 if(msg.type==='select')startDemo(msg.robot);
 else if(!demo)record('DEMO','데모를 불러오는 중입니다');
 else if(msg.type==='play')demo.playing=true;
 else if(msg.type==='pause'||msg.type==='stop')demo.playing=false;
 else if(msg.type==='reset')restartDemo();
 else record('DEMO',{ignored:msg,reason:'데모 재생 중에는 명령이 실행되지 않습니다. 실시간 조작은 [서버 설정]에서 연결하세요'});
}
function tickDemo(){
 const now=performance.now(),dt=Math.min(0.1,(now-demo.lastTick)/1000);demo.lastTick=now;
 const {frames,fps,moving,geom_ids,commands}=demo.data,lastIndex=frames.length-1;
 if(demo.playing)demo.playhead+=dt*fps;
 if(demo.playhead>lastIndex+DEMO_HOLD_AT_END_S*fps)restartDemo();
 const position=Math.min(demo.playhead,lastIndex),index=Math.min(Math.floor(position),lastIndex-1),alpha=position-index;
 const a=frames[index],b=frames[index+1];
 moving.forEach((geomIndex,k)=>{
  const o=k*7,pa=a.p,pb=b.p,id=geom_ids[geomIndex];
  demo.positions[id]=[0,1,2].map(i=>pa[o+i]+(pb[o+i]-pa[o+i])*alpha);
  // 쿼터니언 보간(최단 경로 부호 맞춘 nlerp)
  const sign=(pa[o+3]*pb[o+3]+pa[o+4]*pb[o+4]+pa[o+5]*pb[o+5]+pa[o+6]*pb[o+6])<0?-1:1;
  const q=[3,4,5,6].map(i=>pa[o+i]+(sign*pb[o+i]-pa[o+i])*alpha),norm=Math.hypot(...q)||1;
  demo.rotations[id]=quatToRowMatrix(q[0]/norm,q[1]/norm,q[2]/norm,q[3]/norm);
 });
 while(demo.commandIndex<commands.length&&commands[demo.commandIndex].f<=position){const c=commands[demo.commandIndex++];record('TX',c.msg);if(c.result)record('ACK',c.result);}
 const shown=alpha<0.5?a:b;
 $('demoProgress').textContent=`${(position/fps).toFixed(1)} / ${(lastIndex/fps).toFixed(1)} s${demo.playing?'':' · 일시정지'}`;
 update({seq:++demo.seq,time:a.t+(b.t-a.t)*alpha,running:demo.playing&&shown.run,anchored:shown.anc,contacts:shown.con,joints:meta.joints.map((j,i)=>({id:j.id,q:shown.j[i][0],dq:shown.j[i][1],target:shown.j[i][2],torque:shown.j[i][3]})),positions:demo.positions,rotations:demo.rotations,sensors:[],mission:shown.m});
 $('hz').textContent=`녹화 ${fps} fps`;
}
// ── 실행 데이터 수집: 서버(recorder.py)가 data/<로봇>/<실행>/에 누적 저장, 여기서는 상태 표시·목록·다운로드 ──
let apiBase=null,lastRecordText='',lastRecordRunId=null;   // apiBase: 실시간 서버 HTTP 주소(데모 중 null)
const RECORD_STATE_TEXT={off:'기록 꺼짐',waiting:'대기 · 실행(▶)하면 기록 시작'};
function socketToApiBase(socketUrl){const url=new URL(socketUrl);return `${url.protocol==='wss:'?'https:':'http:'}//${url.host}`;}
function recordCommand(){return{type:'record',enabled:$('recordToggle').checked,label:$('recordLabel').value.trim()};}
async function fetchData(path){
 if(!apiBase)throw new Error('실시간 서버에 연결해야 합니다');
 const response=await fetch(apiBase+path,{headers:serverSettings.token?{'X-Lab-Token':serverSettings.token}:{},cache:'no-store'});
 if(!response.ok){let detail='';try{detail=(await response.json()).detail;}catch{/* 본문이 JSON이 아니면 상태 코드만 표시 */}throw new Error(detail||`HTTP ${response.status}`);}
 return response;
}
function formatBytes(bytes){return bytes>=1048576?`${(bytes/1048576).toFixed(1)} MB`:`${Math.ceil(bytes/1024)} KB`;}
function renderRecording(recording){
 const text=recording.state==='recording'?`● 기록 중 · ${recording.frames.toLocaleString()} 프레임 (${(recording.frames/30).toFixed(1)} s)${recording.label?` · ${recording.label}`:''}`
  :recording.state==='stopped'?`기록 멈춤 · ${recording.reason}`:RECORD_STATE_TEXT[recording.state]??recording.state;
 if(text!==lastRecordText){lastRecordText=text;$('recordStatus').textContent=text;$('recordStatus').classList.toggle('live',recording.state==='recording');}
 if(document.activeElement!==$('recordToggle'))$('recordToggle').checked=recording.state!=='off';
 // 실행 하나가 끝나면(초기화·로봇 변경·한도) 목록 갱신
 if(lastRecordRunId&&recording.run_id!==lastRecordRunId)refreshDataList();
 lastRecordRunId=recording.run_id;
}
async function downloadData(path,fallbackName){
 try{
  const response=await fetchData(path);
  const disposition=response.headers.get('Content-Disposition')||'';
  const filename=/filename="?([^";]+)"?/.exec(disposition)?.[1]||fallbackName;
  const url=URL.createObjectURL(await response.blob());const a=document.createElement('a');a.href=url;a.download=filename;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  record('DATA',{downloaded:filename});
 }catch(e){record('ERROR',`데이터 다운로드 실패: ${e.message}`);}
}
async function refreshDataList(){
 const summaryBox=$('dataSummary'),runList=$('dataRuns');
 if(!apiBase){summaryBox.textContent='데모 모드에서는 수집하지 않습니다. 실시간 서버에 연결하면 자동 기록됩니다';runList.replaceChildren();return;}
 try{
  const [summary,{runs}]=await Promise.all([fetchData('/api/data/summary').then(r=>r.json()),fetchData('/api/data/runs?limit=8').then(r=>r.json())]);
  const table=document.createElement('table');
  table.insertRow().append(...['로봇','실행','누적 시간','성공'].map(text=>Object.assign(document.createElement('th'),{textContent:text})));
  for(const [robot,totals] of Object.entries(summary.robots)){const row=table.insertRow();for(const text of [ROBOT_NAMES[robot]??robot,totals.runs,`${totals.sim_seconds.toFixed(1)} s`,totals.successes])row.insertCell().textContent=text;}
  const usage=document.createElement('small');usage.textContent=`저장 ${formatBytes(summary.used_bytes)} / 한도 ${formatBytes(summary.max_bytes)}`;
  summaryBox.replaceChildren(table,usage);
  runList.replaceChildren(...runs.map(run=>{
   const item=document.createElement('li'),info=document.createElement('span');
   info.textContent=`${ROBOT_NAMES[run.robot]??run.robot} · ${run.started_at.slice(5,16).replace('T',' ')} · ${run.sim_seconds.toFixed(1)}s${run.success?' · 성공':''}${run.status==='recording'?' · 기록 중':''}${run.label?` · ${run.label}`:''}`;
   const buttons=[['CSV','states.csv'],['명령','commands.jsonl']].map(([text,file])=>{const b=document.createElement('button');b.type='button';b.textContent=text;b.onclick=()=>downloadData(`/api/data/runs/${run.robot}/${run.run_id}/${file}`,`${run.robot}_${run.run_id}_${file}`);return b;});
   item.append(info,...buttons);return item;
  }));
  if(!runs.length)runList.append(Object.assign(document.createElement('li'),{textContent:'아직 기록이 없습니다. ▶ 실행하면 저장됩니다'}));
 }catch(e){summaryBox.textContent=`데이터 조회 실패: ${e.message}`;runList.replaceChildren();}
}
let linkServer='',connectAttempt=0;
let serverSettings=loadServerSettings();
$('serverUrl').value=serverSettings.server;$('serverToken').value=serverSettings.token;
$('serverForm').onsubmit=e=>{e.preventDefault();serverSettings={server:$('serverUrl').value.trim(),token:$('serverToken').value.trim()};saveServerSettings(serverSettings);$('serverPanel').open=false;connect();};
function showConnectionProblem(status,message){$('connection').textContent=status;$('loading').style.display='';$('loading').textContent=message;}
// 웹버전(정적 호스팅)은 실시간 서버를 못 쓰면 데모로 대체, 실습 서버가 직접 띄운 페이지는 원인만 표시
function fallBack(status,message){record('INFO',message);if(IS_STATIC_HOSTING)startDemo(demoRobot,message);else showConnectionProblem(status,message);}
async function connect(){stopLiveConnection();stopDemo();meta=null;
 const attempt=connectAttempt;$('connection').textContent='서버 찾는 중';
 let socketUrl;
 try{
  const server=await resolveServerAddress();
  if(attempt!==connectAttempt)return;   // 그사이 재연결·데모를 다시 고른 경우 이전 시도는 버림
  if(server===null){fallBack('● 서버 꺼짐','실습 서버가 꺼져 있어 데모를 재생합니다');return;}
  socketUrl=buildSocketUrl(server,serverSettings.token);
 }catch(e){if(attempt!==connectAttempt)return;fallBack('● 연결 실패',e.message);return;}
 if(!socketUrl){showConnectionProblem('● 서버 미설정','오른쪽 위 [서버 설정]에 실습 서버 주소와 토큰을 입력하세요');$('serverPanel').open=true;return;}
 if(IS_STATIC_HOSTING&&!serverSettings.token){fallBack('● 토큰 필요','접속 토큰이 없어 데모를 재생합니다. 실시간 조작은 [서버 설정]에 토큰을 입력하세요');return;}
 let opened=false;
 apiBase=socketToApiBase(socketUrl);
 const s=new WebSocket(socketUrl);socket=s;$('connection').textContent='연결 중';s.onopen=()=>{opened=true;$('connection').textContent='● SIM CONNECTED';record('CONNECTED','독립 시뮬레이션 세션');
  // 서버 기본값(기록 켬·라벨 없음)과 다르게 설정해 둔 경우 새 세션에도 적용
  if(!$('recordToggle').checked||$('recordLabel').value.trim())send(recordCommand(),true);
  refreshDataList();};s.onclose=e=>{clearPending();$('connection').textContent='● 연결 끊김';$('hz').textContent='0 Hz';record('DISCONNECTED',CLOSE_REASONS[e.code]||'재연결 버튼을 누르세요');
  if(e.code===4401){$('serverPanel').open=true;fallBack('● 토큰 오류',CLOSE_REASONS[4401]);}
  else if(e.code===4429)fallBack('● 정원 초과',CLOSE_REASONS[4429]);
  else if(!opened)fallBack('● 연결 실패','실습 서버에 연결할 수 없어 데모를 재생합니다');};s.onerror=()=>record('ERROR','서버 실행 상태를 확인하세요');s.onmessage=e=>{const msg=JSON.parse(e.data);if(msg.type==='model'){demoRobot=msg.robot;build(msg);}else if(msg.type==='state'&&meta)update(msg);else record(msg.type.toUpperCase(),msg);};}
for(const type of ['play','pause','reset','stop'])$(type).onclick=()=>send({type},type!=='play');
document.querySelectorAll('[data-robot]').forEach(b=>b.onclick=()=>send({type:'select',robot:b.dataset.robot},true));$('anchor').onchange=()=>send({type:'anchor',enabled:$('anchor').checked},true);$('assist').onchange=()=>send({type:'assist',enabled:$('assist').checked},true);$('reconnect').onclick=connect;$('chartJoint').onchange=()=>history=[];
for(const id of ['delay','loss'])$(id).oninput=()=>$(id+'Value').textContent=$(id).value+(id==='delay'?' ms':'%');
$('send').onclick=()=>{try{const cmd=JSON.parse($('command').value);if(!cmd||typeof cmd!=='object'||Array.isArray(cmd))throw new Error('JSON object를 입력하세요');send(cmd,['stop','pause','reset','select'].includes(cmd.type));}catch(e){record('ERROR',e.message);}};
$('export').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify({robot:meta?.robot,protocol:'lab-websocket-v1',log,state},null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='robot-lab-log.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
window.addEventListener('keydown',e=>{if(e.code==='Escape')send({type:'stop'},true);});
$('recordToggle').onchange=()=>send(recordCommand(),true);
$('recordLabel').onchange=()=>send(recordCommand(),true);
$('dataRefresh').onclick=refreshDataList;
$('dataExport').onclick=()=>downloadData('/api/data/export','robot-lab-data.zip');
$('demoStart').onclick=()=>{$('serverPanel').open=false;startDemo(demoRobot,'직접 선택한 데모');};
$('demoLive').onclick=()=>{$('serverPanel').open=true;$('serverToken').focus();};
function frame(){requestAnimationFrame(frame);if(demo&&meta)tickDemo();controls.update();renderer.render(scene,camera);}frame();connect();
