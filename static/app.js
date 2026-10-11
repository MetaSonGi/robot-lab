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
function send(msg,immediate=false){if(!socket||socket.readyState!==WebSocket.OPEN){record('ERROR','서버 연결이 필요합니다');return;}
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
 $('time').textContent=`${s.time.toFixed(2)} s`;$('contacts').textContent=s.contacts;$('anchor').checked=s.anchored;$('play').textContent=s.running?'▶ 실행 중':'▶ 실행';updateMission(s.mission);
 for(const [id,obj] of objects){const r=s.rotations[id],p=s.positions[id];matrix.set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);rotation.setFromRotationMatrix(matrix);obj.position.fromArray(p);obj.quaternion.copy(rotation);}
 for(const j of s.joints){const input=$(`joint${j.id}`);if(document.activeElement!==input){input.value=j.target;$(`out${j.id}`).textContent=j.target.toFixed(3);}}
 const selected=s.joints.find(j=>j.id===Number($('chartJoint').value));if(selected){history.push([selected.target,selected.q]);if(history.length>200)history.shift();$('telemetry').textContent=`실제 ${selected.q.toFixed(4)} · 속도 ${selected.dq.toFixed(4)} · 구동력 ${selected.torque.toFixed(3)} ${meta.joints.find(j=>j.id===selected.id).unit==='m'?'N':'N·m'} · 센서 ${s.sensors.length}개`;}drawChart();
}
function drawChart(){const c=$('chart'),ctx=c.getContext('2d');c.width=Math.max(200,c.clientWidth)*devicePixelRatio;c.height=130*devicePixelRatio;const w=c.width,h=c.height;ctx.clearRect(0,0,w,h);if(!history.length)return;const vals=history.flat(),lo=Math.min(...vals)-.1,hi=Math.max(...vals)+.1;ctx.strokeStyle='#293a50';ctx.lineWidth=1;for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke();}for(let k=0;k<2;k++){ctx.strokeStyle=k?'#77e7be':'#b696ff';ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();history.forEach((v,i)=>{const x=i*w/199,y=h-((v[k]-lo)/(hi-lo))*h;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}ctx.fillStyle='#9aaec6';ctx.font=`${10*devicePixelRatio}px sans-serif`;ctx.fillText(hi.toFixed(2),8*devicePixelRatio,14*devicePixelRatio);ctx.fillText(lo.toFixed(2),8*devicePixelRatio,h-6*devicePixelRatio);}
// 서버 주소·토큰: URL(?server=&token=) → 저장값 → (실습 서버가 직접 제공한 페이지면) 현재 주소
const SERVER_SETTINGS_KEY='robotLab.server';
const IS_STATIC_HOSTING=location.hostname.endsWith('github.io');
const CLOSE_REASONS={4401:'접속 토큰이 올바르지 않습니다. 서버 설정에서 토큰을 확인하세요',4429:'동시 접속 인원이 가득 찼습니다. 잠시 후 다시 연결하세요'};
function loadServerSettings(){
 let saved={};try{saved=JSON.parse(localStorage.getItem(SERVER_SETTINGS_KEY)||'{}')||{};}catch{saved={};}
 const params=new URLSearchParams(location.search);
 const settings={server:params.get('server')??saved.server??'',token:params.get('token')??saved.token??''};
 if(params.has('server')||params.has('token')){
  saveServerSettings(settings);
  // 토큰이 주소창·공유 화면에 남지 않도록 URL에서 제거
  params.delete('server');params.delete('token');const rest=params.toString();
  window.history.replaceState(null,'',location.pathname+(rest?`?${rest}`:'')+location.hash);
 }
 return settings;
}
function saveServerSettings(settings){try{localStorage.setItem(SERVER_SETTINGS_KEY,JSON.stringify(settings));}catch{/* 저장 불가(사생활 모드)여도 이번 접속은 진행 */}}
function buildSocketUrl({server,token}){
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
let serverSettings=loadServerSettings();
$('serverUrl').value=serverSettings.server;$('serverToken').value=serverSettings.token;
$('serverForm').onsubmit=e=>{e.preventDefault();serverSettings={server:$('serverUrl').value.trim(),token:$('serverToken').value.trim()};saveServerSettings(serverSettings);$('serverPanel').open=false;connect();};
function connect(){clearPending();if(socket){socket.onclose=null;socket.close();socket=null;}meta=null;
 let socketUrl;try{socketUrl=buildSocketUrl(serverSettings);}catch(e){record('ERROR',`서버 주소 오류: ${e.message}`);$('serverPanel').open=true;return;}
 if(!socketUrl){$('connection').textContent='● 서버 미설정';$('loading').textContent='오른쪽 위 [서버 설정]에 실습 서버 주소와 토큰을 입력하세요';$('serverPanel').open=true;return;}
 const s=new WebSocket(socketUrl);socket=s;$('connection').textContent='연결 중';s.onopen=()=>{$('connection').textContent='● SIM CONNECTED';record('CONNECTED','독립 시뮬레이션 세션');};s.onclose=e=>{clearPending();$('connection').textContent='● 연결 끊김';$('hz').textContent='0 Hz';record('DISCONNECTED',CLOSE_REASONS[e.code]||'재연결 버튼을 누르세요');if(e.code===4401)$('serverPanel').open=true;};s.onerror=()=>record('ERROR','서버 실행 상태를 확인하세요');s.onmessage=e=>{const msg=JSON.parse(e.data);if(msg.type==='model')build(msg);else if(msg.type==='state'&&meta)update(msg);else record(msg.type.toUpperCase(),msg);};}
for(const type of ['play','pause','reset','stop'])$(type).onclick=()=>send({type},type!=='play');
document.querySelectorAll('[data-robot]').forEach(b=>b.onclick=()=>send({type:'select',robot:b.dataset.robot},true));$('anchor').onchange=()=>send({type:'anchor',enabled:$('anchor').checked},true);$('assist').onchange=()=>send({type:'assist',enabled:$('assist').checked},true);$('reconnect').onclick=connect;$('chartJoint').onchange=()=>history=[];
for(const id of ['delay','loss'])$(id).oninput=()=>$(id+'Value').textContent=$(id).value+(id==='delay'?' ms':'%');
$('send').onclick=()=>{try{const cmd=JSON.parse($('command').value);if(!cmd||typeof cmd!=='object'||Array.isArray(cmd))throw new Error('JSON object를 입력하세요');send(cmd,['stop','pause','reset','select'].includes(cmd.type));}catch(e){record('ERROR',e.message);}};
$('export').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify({robot:meta?.robot,protocol:'lab-websocket-v1',log,state},null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='robot-lab-log.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
window.addEventListener('keydown',e=>{if(e.code==='Escape')send({type:'stop'},true);});
function frame(){requestAnimationFrame(frame);controls.update();renderer.render(scene,camera);}frame();connect();
