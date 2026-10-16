// Execute the actual inline script with a minimal DOM and fake network. No robot.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const nodes=new Map();
function element(){return {textContent:'',dataset:{},disabled:true,scrollHeight:0,scrollTop:0,clientHeight:0,listeners:{},addEventListener(k,f){this.listeners[k]=f;}};}
const buttons=['A','B'].flatMap(side=>[1,2,3].map(n=>Object.assign(element(),{dataset:{side,round:String(n)}})));
const intervals=[],posts=[];
let now=10000,mode='bad';
const hardware=ok=>({can_run:ok,reasons:ok?[]:['底盘：电调反馈不齐'],lidar:{ready:ok,reason:ok?'雷达已就绪':'等待雷达同步'},chassis:{ready:ok,reason:ok?'底盘已就绪':'电调反馈不齐'}});
const context=vm.createContext({document:{getElementById(id){if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);},querySelectorAll(){return buttons;}},
  localStorage:{getItem(){return null;},setItem(){},removeItem(){}},Date:{now:()=>now},AbortController:class{constructor(){this.signal={};}abort(){}},
  setInterval(fn,ms){intervals.push({fn,ms});},setTimeout(){return 1;},clearTimeout(){},
  async fetch(url,options){
    if(mode==='offline')throw Error('connection lost');
    if(options.method==='POST'){
      posts.push(JSON.parse(options.body));
      if(mode==='reject')return {ok:false,status:409,json:async()=>({error:'底盘：电调反馈不齐',hardware:hardware(false)})};
      return {ok:true,status:202,json:async()=>({id:'fake-'+posts.length})};
    }
    return {ok:true,status:200,json:async()=>({status:'idle',lines:[],cursor:0,hardware:hardware(mode==='good')})};
  }});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const flush=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
  await flush();
  assert(buttons.every(b=>b.disabled));
  assert(nodes.get('chassis-reason').textContent.includes('反馈不齐'));
  await buttons[0].listeners.click();assert.equal(posts.length,0);
  mode='good';await vm.runInContext('poll()',context);
  assert(buttons.every(b=>!b.disabled));
  assert.equal(nodes.get('lidar-status').textContent,'正常');
  for(const b of buttons){await b.listeners.click();await flush();}
  assert.deepEqual(posts,['A','B'].flatMap(side=>[1,2,3].map(round=>({side,round}))));
  mode='reject';await buttons[0].listeners.click();await flush();
  assert(buttons.every(b=>b.disabled));
  assert(nodes.get('status').textContent.includes('电调反馈不齐'));
  mode='good';await vm.runInContext('poll()',context);
  now+=2500;intervals.find(i=>i.ms===250).fn();
  assert(buttons.every(b=>b.disabled),'stale UI must not keep an old green state');
  mode='good';await vm.runInContext('poll()',context);
  mode='offline';await vm.runInContext('poll()',context);
  assert(buttons.every(b=>b.disabled));
  console.log('PASS: six healthy actions, initial/failed/stale/disconnected gating, backend rejection reasons');
})().catch(error=>{console.error(error);process.exitCode=1;});
