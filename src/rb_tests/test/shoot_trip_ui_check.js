const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8'),script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
function element(){return {textContent:'',dataset:{},disabled:true,scrollHeight:0,scrollTop:0,clientHeight:0,handlers:{},addEventListener(event,fn){this.handlers[event]=fn;}};}
const elements={},buttons=[];for(const side of ['A','B'])for(let round=1;round<=3;round++){const b=element();b.dataset={side,round:String(round)};buttons.push(b);}
const get=id=>elements[id]||(elements[id]=element());
let lidar=false,ready=true,activity='idle',motion='idle',broken=false,now=1000,posts=[];
const intervals=[];
function trip(){return {can_run:ready&&activity==='idle',chassis:{ready,reason:'fixture chassis'},reason:activity==='shoot'?'投篮回合正在执行':'fixture reason',activity,motion:{status:motion,phase:motion==='running'?'右移':'待执行',message:'fixture motion'},settings:{speed_mps:.25}};}
const context={console,Date:{now:()=>now},document:{getElementById:get,querySelectorAll:()=>buttons},localStorage:{getItem:()=>null,setItem(){},removeItem(){}},setInterval(fn){intervals.push(fn);return intervals.length;},setTimeout(){return 1;},clearTimeout(){},AbortController:class{constructor(){this.signal={};}abort(){}},
async fetch(url,options){
 assert(url.startsWith('/api/shoot/'),url);
 if(broken)throw Error('fixture disconnected');
 if(options&&options.method==='POST'){
  posts.push(url);
  if(url.endsWith('chassis-trip/run')){activity='chassis_trip';motion='running';return {ok:true,status:202,json:async()=>trip()};}
  if(url.endsWith('chassis-trip/stop')){activity='idle';motion='stopped';return {ok:true,status:200,json:async()=>trip()};}
  activity='shoot';return {ok:true,status:202,json:async()=>({id:'shoot-fixture'})};
 }
 return {ok:true,status:200,json:async()=>({id:activity==='shoot'?'shoot-fixture':null,status:activity==='shoot'?'running':'idle',side:'A',round:1,lines:[],cursor:0,hardware:{can_run:lidar&&ready,lidar:{ready:lidar,reason:'fixture lidar'},chassis:{ready,reason:'fixture chassis'}},chassis_trip:trip()})};
}};
vm.createContext(context);vm.runInContext(script,context);
const settle=async()=>{for(let i=0;i<5;i++)await new Promise(setImmediate);};
const invoke=async(code)=>{await vm.runInContext(code,context);await settle();};
(async()=>{
 await settle();
 assert(buttons.every(b=>b.disabled));assert(!get('trip-run').disabled);assert(get('trip-stop').disabled);
 await get('trip-run').handlers.click();await settle();
 assert(activity==='chassis_trip'&&get('trip-run').disabled&&!get('trip-stop').disabled);
 const count=posts.length;await get('trip-run').handlers.click();await settle();assert(posts.length===count);
 assert(buttons.every(b=>b.disabled));
 await get('trip-stop').handlers.click();await settle();assert(activity==='idle'&&!get('trip-run').disabled&&get('trip-stop').disabled);
 lidar=true;await invoke('poll()');assert(buttons.every(b=>!b.disabled));
 await buttons[0].handlers.click();await settle();assert(activity==='shoot');assert(buttons.every(b=>b.disabled)&&get('trip-run').disabled);
 const afterShoot=posts.length;await get('trip-run').handlers.click();await settle();assert(posts.length===afterShoot);
 activity='idle';await invoke('poll()');assert(!get('trip-run').disabled);
 ready=false;await invoke('poll()');assert(get('trip-run').disabled&&buttons.every(b=>b.disabled));
 ready=true;await invoke('poll()');assert(!get('trip-run').disabled);
 now+=2500;intervals[1]();assert(get('trip-run').disabled&&buttons.every(b=>b.disabled));
 broken=true;await invoke('poll()');assert(get('trip-run').disabled&&buttons.every(b=>b.disabled));
 assert(html.includes('距离按速度和时间估算'));assert(html.indexOf('aria-label="底盘往返"')>html.indexOf('aria-label="执行结果"'));
 console.log('shooting + integrated chassis trip UI checks passed');
})().catch(error=>{console.error(error);process.exitCode=1;});