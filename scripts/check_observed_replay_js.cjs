// Exercise the actual inspector script against real runtime data without a
// browser or network. Canvas calls must remain finite; unavailable states clear.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(__dirname+'/observed_replay_inspector.html','utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const source=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
function boot(data){
 const nodes=new Map();let draws=0,clears=0;
 const ctx=new Proxy({}, {get:(t,key)=>t[key]??((...args)=>{for(const x of args)if(typeof x==='number')assert(Number.isFinite(x),key+' nonfinite');if(key==='clearRect')clears++;else if(['stroke','fill','fillText'].includes(key))draws++;}),set:(t,k,v)=>(t[k]=v,true)});
 const canvases=[0,1,2].map(()=>({width:540,height:540,getContext:()=>ctx}));
 const get=id=>{if(!nodes.has(id))nodes.set(id,{textContent:'',value:'0',checked:true,addEventListener(){}});return nodes.get(id);};
 get('replay-data').textContent=JSON.stringify(data);get('centre').value='socket';get('width').value='0.01';
 const context=vm.createContext({document:{getElementById:get,querySelectorAll:()=>canvases},setInterval(){return 1;},clearInterval(){}});
 vm.runInContext(script,context,{timeout:20000});
 return {context,get,counts:()=>({draws,clears})};
}
const live=boot(source);assert.equal(live.get('error').textContent,'');assert(live.counts().draws>0);
assert.match(live.get('state').textContent,/tick 0/);
assert(vm.runInContext('availableEstimates(0).every(e=>!e.estimate)',live.context));
live.get('sample').value=String(source.frames.length-1);vm.runInContext('draw()',live.context);
assert.equal(live.get('error').textContent,'');assert.match(live.get('state').textContent,new RegExp('tick '+source.frames.at(-1).scene.tick));
const future=source.frames.at(-1).scene.tick+100000;
assert(vm.runInContext(`availableEstimates(${future}).every(e=>!e.estimate)`,live.context),'stale estimates must not be replaced by truth');
const validEstimate=d=>d.report.estimator_updates.find(u=>u.estimate.validity==='valid').estimate;
const mutations=[
 d=>d.length_unit='mm',d=>d.frames[0].scene.truth=null,d=>d.frames[1].scene.tick=0,d=>d.frames[0].bodies=[],d=>d.report.schema_version=99,
 d=>d.frames[0].physical_jaws=[],d=>d.frames[0].physical_jaws[1]=d.frames[0].physical_jaws[0],
 d=>validEstimate(d).pose.axis_world_unit=[0,0,0],d=>delete validEstimate(d).uncertainty.axis_tangent_sigma_rad,
 d=>validEstimate(d).uncertainty.axis_tangent_sigma_rad=[-1,0],
 d=>validEstimate(d).newest_available_tick=validEstimate(d).controller_tick+1,
 d=>validEstimate(d).oldest_capture_tick=null,d=>d.report.estimator_updates[0].accepted_by_controller='true',
 d=>d.report.estimator_updates.reverse(),d=>d.report.decisions[0].sequence=1,
 d=>delete d.frames[0].contact_packet.grip_force_proxy_n
];
for(const mutate of mutations){
 const bad=structuredClone(source);mutate(bad);const rejected=boot(bad);
 assert.match(rejected.get('error').textContent,/unavailable/i);
 assert.equal(rejected.counts().draws,0,'malformed input rendered geometry');
}
console.log(`Inspector JS: real data mapping, finite projections, stale estimates, and ${mutations.length} malformed states verified`);
