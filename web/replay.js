'use strict';
const $=id=>document.getElementById(id);
let dataset=null, seconds=0, videoReady=false, demoPlaying=false, reviews={}, objectURL=null;
const video=$('video');
const clamp=(n,a,b)=>Math.max(a,Math.min(b,n));
const format=t=>`${String(Math.floor(t/60)).padStart(2,'0')}:${String(Math.floor(t%60)).padStart(2,'0')}`;
const safe=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function validate(data){
 if(!data||!Array.isArray(data.observations)||!data.observations.length||!Array.isArray(data.incidents))throw Error('Dataset needs observations[] and incidents[] arrays.');
 const observations=data.observations.map((r,i)=>{
  const time=Number(r.time_sec),p=Number(r.person_count),v=Number(r.vehicle_count);
  if(!Number.isFinite(time)||time<0||!Number.isFinite(p)||p<0||!Number.isFinite(v)||v<0)throw Error('Invalid observation at row '+(i+1));
  return {...r,time_sec:time,person_count:p,vehicle_count:v,latitude:Number(r.latitude)||0,longitude:Number(r.longitude)||0};
 }).sort((a,b)=>a.time_sec-b.time_sec);
 const incidents=data.incidents.map((r,i)=>{
  if(!Number.isFinite(Number(r.time_sec))||!r.id)throw Error('Invalid incident at row '+(i+1));
  return {...r,time_sec:Number(r.time_sec),latitude:Number(r.latitude)||0,longitude:Number(r.longitude)||0};
 }).sort((a,b)=>a.time_sec-b.time_sec);
 return {...data,observations,incidents,duration_sec:Math.max(Number(data.duration_sec)||0,observations.at(-1).time_sec,1)};
}
function applyDataset(raw){
 dataset=validate(raw);seconds=0;reviews={};
 $('datasetName').textContent=dataset.name||'Imported observations';
 $('rows').textContent=dataset.observations.length;
 $('range').textContent=format(dataset.duration_sec);
 $('samples').textContent=dataset.observations.length;
 $('peak').textContent=Math.max(...dataset.observations.map(x=>x.person_count));
 $('dataSource').textContent=(dataset.origin||'UNKNOWN').toUpperCase().includes('SYNTHETIC')?'SYNTHETIC':'IMPORTED';
 $('sourceNote').textContent=dataset.origin||'Imported dataset. Confirm provenance before presenting results.';
 $('dataNote').textContent=dataset.coordinate_note||'Coordinates are supplied by the dataset; accuracy is not independently verified.';
 $('seek').max=dataset.duration_sec;$('totalTime').textContent=format(dataset.duration_sec);$('chartEnd').textContent=format(dataset.duration_sec);
 if(videoReady){video.currentTime=0;video.pause()};renderChart();render();
}
function observationAt(t){let arr=dataset.observations;let lo=0,hi=arr.length-1;while(lo<hi){let mid=Math.ceil((lo+hi)/2);if(arr[mid].time_sec<=t)lo=mid;else hi=mid-1}return arr[lo]}
function mapPoint(lat,lng){const base=dataset.observations[0];const x=clamp(51+(lng-base.longitude)*140000,9,91),y=clamp(47-(lat-base.latitude)*130000,12,88);return {x,y}}
function render(){if(!dataset)return;const o=observationAt(seconds), visible=dataset.incidents.filter(x=>x.time_sec<=seconds);
 $('people').textContent=o.person_count;$('vehicles').textContent=o.vehicle_count;$('crowd').textContent=o.crowd_level||'UNCLASSIFIED';
 $('crowd').style.color=(o.crowd_level==='HIGH'||o.crowd_level==='VERY HIGH')?'#f2b06d':o.crowd_level==='MODERATE'?'#f3d48d':'#59d8b5';
 $('alerts').textContent=visible.filter(x=>!['DISMISSED'].includes(reviews[x.id])).length;
 $('incidentBadge').textContent=visible.length+' EVENTS';$('currentTime').textContent=format(seconds);$('seek').value=seconds;
 $('coords').textContent=`${o.latitude.toFixed(5)}° N · ${o.longitude.toFixed(5)}° E`;
 const pt=mapPoint(o.latitude,o.longitude);$('dronePin').style.left=pt.x+'%';$('dronePin').style.top=pt.y+'%';
 $('mapPins').innerHTML=visible.map(x=>{let p=mapPoint(x.latitude,x.longitude);return `<button class="incident-pin" data-jump="${x.time_sec}" style="left:${p.x}%;top:${p.y}%" title="${safe(x.type)}">◆</button>`}).join('');
 $('mapPins').querySelectorAll('[data-jump]').forEach(b=>b.onclick=()=>seekTo(Number(b.dataset.jump)));
 $('incidentList').innerHTML=visible.length?visible.slice().reverse().map(x=>{let status=reviews[x.id]||'PENDING REVIEW';return `<div class="incident-card"><div class="incident-top"><h3>${safe(x.type)}</h3><span class="severity">${safe(x.severity)}</span></div><p>${safe(x.id)} · ${format(x.time_sec)} · ${safe(x.zone||'UNSPECIFIED')}</p><p>${safe(x.description||'Dataset observation')}</p><p>STATUS: <b style="color:#a4e5d6">${safe(status)}</b></p><div class="incident-actions"><button data-id="${safe(x.id)}" data-status="VERIFIED">Verify</button><button data-id="${safe(x.id)}" data-status="DISMISSED">Dismiss</button><button data-id="${safe(x.id)}" data-status="ASSIGNED">Assign</button></div></div>`}).join(''):'<div class="empty">◌ No incidents at this timestamp.<br>Play or scrub forward to review observations.</div>';
 $('incidentList').querySelectorAll('[data-status]').forEach(b=>b.onclick=()=>{reviews[b.dataset.id]=b.dataset.status;render()});
 $('hud').textContent=`ZONE: ${o.zone||'A'}  |  GEO: DATASET  |  T+${format(seconds)}  |  ANONYMOUS COUNTS`;
 updateChartCursor();
}
function renderChart(){const rows=dataset.observations,max=Math.max(5,...rows.map(x=>x.person_count));const w=640,h=158,p=15;
 const pts=rows.map(x=>[p+(x.time_sec/dataset.duration_sec)*(w-p*2),h-p-(x.person_count/max)*(h-p*2)]);
 const path=pts.map((pt,i)=>(i?'L':'M')+pt[0].toFixed(1)+' '+pt[1].toFixed(1)).join(' ');
 const area=path+` L ${pts.at(-1)[0]} ${h-p} L ${pts[0][0]} ${h-p} Z`;
 $('chart').innerHTML=`<svg viewBox="0 0 ${w} ${h}" preserveAspectRatio="none"><defs><linearGradient id="chartfill" x1="0" y1="0" x2="0" y2="1"><stop stop-color="#39d4c5" stop-opacity=".3"/><stop offset="1" stop-color="#39d4c5" stop-opacity="0"/></linearGradient></defs>${[.25,.5,.75,1].map(y=>`<line x1="${p}" y1="${h*y-p/2}" x2="${w-p}" y2="${h*y-p/2}" stroke="#284458" stroke-dasharray="3 7"/>`).join('')}<path d="${area}" fill="url(#chartfill)"/><path d="${path}" fill="none" stroke="#42d9ce" stroke-width="2.8" vector-effect="non-scaling-stroke"/><line id="chartCursor" x1="15" x2="15" y1="0" y2="${h}" stroke="#f0c181" stroke-width="1.4" stroke-dasharray="4 4"/></svg>`;
}
function updateChartCursor(){let c=$('chartCursor');if(c){let x=15+(seconds/dataset.duration_sec)*610;c.setAttribute('x1',x);c.setAttribute('x2',x)}}
function seekTo(t){seconds=clamp(t,0,videoReady&&video.duration?Math.min(video.duration,dataset.duration_sec):dataset.duration_sec);if(videoReady)video.currentTime=seconds;render()}
function download(name,text,type='application/json'){let blob=new Blob([text],{type}),u=URL.createObjectURL(blob),a=document.createElement('a');a.href=u;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(u),1000)}
$('seek').addEventListener('input',e=>seekTo(Number(e.target.value)));
$('backBtn').onclick=()=>seekTo(seconds-10);$('forwardBtn').onclick=()=>seekTo(seconds+10);
$('playBtn').onclick=()=>{if(videoReady){video.paused?video.play().catch(e=>alert(e.message)):video.pause()}else{demoPlaying=!demoPlaying;$('videoStatus').textContent=demoPlaying?'DATASET TIMELINE PLAYING':'NO VIDEO LOADED'}};
$('videoFile').onchange=e=>{let file=e.target.files[0];if(!file)return;if(objectURL)URL.revokeObjectURL(objectURL);objectURL=URL.createObjectURL(file);video.src=objectURL;videoReady=true;demoPlaying=false;$('placeholder').style.display='none';$('videoName').textContent=file.name;$('videoStatus').textContent='VIDEO LOADED';video.load()};
video.addEventListener('loadedmetadata',()=>{$('videoStatus').textContent='VIDEO READY · '+format(video.duration);if(dataset&&Math.abs(video.duration-dataset.duration_sec)>5)$('videoStatus').textContent+=' · CHECK TIMING'});
video.addEventListener('timeupdate',()=>{seconds=video.currentTime;render()});
$('datasetFile').onchange=async e=>{const file=e.target.files[0];if(!file)return;try{applyDataset(JSON.parse(await file.text()))}catch(err){alert('Could not load dataset: '+err.message)}};
$('exportBtn').onclick=()=>download('garuda_ops_observations.json',JSON.stringify(dataset,null,2));
$('reportBtn').onclick=()=>{let report={generated_at:new Date().toISOString(),dataset:dataset.name,provenance:dataset.origin,incidents:dataset.incidents.map(x=>({...x,review_status:reviews[x.id]||'PENDING REVIEW'}))};download('garuda_ops_incident_report.json',JSON.stringify(report,null,2))};
$('resetBtn').onclick=()=>{reviews={};render()};
setInterval(()=>{if(demoPlaying&&!videoReady&&dataset){seconds+=.25;if(seconds>dataset.duration_sec){seconds=0;demoPlaying=false}$('videoStatus').textContent=demoPlaying?'DATASET TIMELINE PLAYING':'TIMELINE FINISHED';render()}},250);
setInterval(()=>{$('clock').textContent=new Date().toLocaleTimeString('en-IN',{hour12:false})+' IST'},1000);
fetch('/api/dataset').then(r=>{if(!r.ok)throw Error('Dataset endpoint failed');return r.json()}).then(applyDataset).catch(e=>{$('sourceNote').textContent='Dataset load failed: '+e.message});
// --- added for the DRDO bridge: replay flights recorded from the live mission-control page
function loadVideo(src,name){video.src=src;videoReady=true;demoPlaying=false;$('placeholder').style.display='none';$('videoName').textContent=name;$('videoStatus').textContent='VIDEO LOADED';video.load()}
fetch('/api/sessions').then(r=>r.ok?r.json():[]).then(list=>{const sel=$('sessionSel');list.forEach(s=>{const o=document.createElement('option');o.value=s.name;o.textContent=`${s.name.replace('session_','Flight ')} · ${format(s.duration||0)} · ${s.incidents} inc.`;sel.appendChild(o)});const want=new URLSearchParams(location.search).get('session');if(want&&list.some(s=>s.name===want)){sel.value=want;sel.onchange()}}).catch(()=>{});
$('sessionSel').onchange=async e=>{const name=$('sessionSel').value;try{const r=await fetch('/api/dataset'+(name?'?session='+encodeURIComponent(name):''));if(!r.ok)throw Error(r.status);const d=await r.json();applyDataset(d);if(name&&d.video_filename){loadVideo('/media/'+d.video_filename,d.video_filename)}else if(!name){fetch('/media/airsim.mp4',{method:'HEAD'}).then(r=>{if(r.ok)loadVideo('/media/airsim.mp4','airsim.mp4');else{video.removeAttribute('src');videoReady=false;$('placeholder').style.display='';$('videoName').textContent='Not loaded'}})}}catch(err){alert('Could not load session: '+err.message)}};
fetch('/media/airsim.mp4',{method:'HEAD'}).then(r=>{if(r.ok&&!videoReady){video.src='/media/airsim.mp4';videoReady=true;$('placeholder').style.display='none';$('videoName').textContent='airsim.mp4';$('videoStatus').textContent='VIDEO LOADED';}}).catch(()=>{});
