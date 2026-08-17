/* Standalone map editor. The only external input is open(mapName). */
(function () {
  "use strict";
  const root = document.getElementById("standalone-map-editor");
  if (!root) return;
  const $ = (name) => root.querySelector(`[data-map-editor="${name}"]`);
  const canvas = $("canvas"), ctx = canvas.getContext("2d");
  const names = ["floor","map-name","load-map","loading","layer-section","layer","tool","brush","raster-value","semantic-picker","semantic-trigger","semantic-menu","semantic-label","semantic-selected-swatch","semantic-selected-name","semantic-selected-value","point-section","pick-point","point-name","point-type","point-list","layer-undo","layer-discard","layer-save","point-undo","point-discard","point-save","metadata","message","show-grid","show-meter-grid","show-semantic","show-points","coordinates"];
  const ui = Object.fromEntries(names.map((name) => [name, $(name)]));
  const s = { name:"", pgm:null, meta:null, raster:null, savedRaster:null, semantic:null, savedSemantic:null, points:[], savedPoints:[], dirty:new Set(), undo:[], activePanel:"layers", selectedPointId:"", pointPickMode:"", pointPickStep:0, pointPickTargetId:"", pointPickAnchor:null, pointPickCursor:null, moveSnapshotTaken:false, scale:1, panX:0, panY:0, painting:false, panning:false, movingId:"", lastX:0, lastY:0, loadToken:0 };
  const clonePoints = (rows) => rows.map((row) => ({ ...row }));
  const esc = (value) => { const span=document.createElement("span"); span.textContent=String(value??""); return span.innerHTML; };
  const message = (text,error=false) => { ui.message.textContent=text; ui.message.classList.toggle("is-error",error); };
  function apiUrl(path) {
    const configured=String(window.OPEN_DELIVERY_API_BASE_URL||window.API_BASE_URL||"").replace(/\/$/,"");
    if(configured)return configured+path;
    const port=Number(window.location.port||0);
    const fallbackPort=port>0?port+1:8001;
    return `${window.location.protocol}//${window.location.hostname}:${fallbackPort}${path}`;
  }
  async function json(url, options) {
    const response=await fetch(apiUrl(url),options);
    if (!response.ok) { let detail=`HTTP ${response.status}`; try { detail=(await response.json()).error||detail; } catch (_) {} throw new Error(detail); }
    return response.json();
  }
  function parseYaml(text) {
    const meta={resolution:.05,origin:[0,0,0],occupied_thresh:.65,free_thresh:.196};
    String(text||"").split(/\r?\n/).forEach((line)=>{
      const match=line.match(/^\s*([\w_]+)\s*:\s*(.*?)\s*(?:#.*)?$/); if(!match)return;
      if(match[1]==="origin"){const values=match[2].replace(/[\[\]]/g,"").split(",").map(Number);if(values.length>=2&&values.every(Number.isFinite))meta.origin=values;}
      else if(Object.prototype.hasOwnProperty.call(meta,match[1])&&Number.isFinite(Number(match[2])))meta[match[1]]=Number(match[2]);
    });
    return meta;
  }
  function cloneCanvas(source) {
    const copy=document.createElement("canvas"); copy.width=source.width; copy.height=source.height;
    copy.getContext("2d").drawImage(source,0,0); return copy;
  }
  function rasterFromPgm(pgm) {
    const out=document.createElement("canvas"); out.width=pgm.width; out.height=pgm.height;
    const target=out.getContext("2d"), image=target.createImageData(out.width,out.height), max=Number(pgm.maxVal||pgm.max_val||255)||255;
    for(let i=0;i<out.width*out.height;i+=1){const value=Math.round(Number(pgm.data[i])/max*255),p=i*4;image.data[p]=value;image.data[p+1]=value;image.data[p+2]=value;image.data[p+3]=255;}
    target.putImageData(image,0,0); return out;
  }
  function semanticCanvas(dataUrl,width,height) {
    if(!dataUrl){const blank=document.createElement("canvas");blank.width=width;blank.height=height;return Promise.resolve(blank);}
    return new Promise((resolve,reject)=>{const image=new Image();image.onload=()=>{const out=document.createElement("canvas");out.width=width;out.height=height;out.getContext("2d").drawImage(image,0,0,width,height);resolve(out);};image.onerror=()=>reject(new Error("语义地图图片加载失败"));image.src=dataUrl;});
  }
  function sync() {
    const dirty=s.dirty.size>0;
    ui.floor.textContent=s.name?s.name+(dirty?" · 未保存":""):"未加载地图";
    const layerDirty=s.dirty.has("raster")||s.dirty.has("semantic"),pointDirty=s.dirty.has("points");
    ui["layer-undo"].disabled=!s.undo.some((entry)=>entry.layer!=="points");ui["layer-discard"].disabled=!layerDirty;ui["layer-save"].disabled=!s.pgm||!layerDirty;
    ui["point-undo"].disabled=!s.undo.some((entry)=>entry.layer==="points");ui["point-discard"].disabled=!pointDirty;ui["point-save"].disabled=!s.pgm||!pointDirty;
    ui["layer-section"].classList.toggle("has-unsaved",layerDirty);ui["point-section"].classList.toggle("has-unsaved",pointDirty);
  }
  function snapshot(layer) {
    const source=layer==="semantic"?s.semantic:s.raster;
    s.undo.push({layer,dirty:[...s.dirty],value:layer==="points"?clonePoints(s.points):source.getContext("2d").getImageData(0,0,source.width,source.height)});
    if(s.undo.length>12)s.undo.shift(); sync();
  }
  function mark(layer){s.dirty.add(layer);sync();}
  function tools() {
    const choices=[["paint","画笔"],["erase","擦除"]];
    ui.tool.innerHTML=choices.map(([value,label])=>`<option value="${value}">${label}</option>`).join("");
    ui.brush.disabled=false;ui["raster-value"].disabled=ui.layer.value!=="raster";ui["semantic-label"].disabled=ui.layer.value!=="semantic";
  }
  function activatePanel(panel, announce=true) {
    s.activePanel=panel;
    const layers=panel==="layers";
    ui["layer-section"].classList.toggle("is-active",layers);
    ui["point-section"].classList.toggle("is-active",!layers);
    if(layers){
      cancelPointPick(false);
      s.movingId="";
      if(ui.layer.value==="raster")ui["show-grid"].checked=true;
      else ui["show-semantic"].checked=true;
    }else{
      ui["show-points"].checked=true;
    }
    if(announce)message(layers?`已启用${ui.layer.value==="raster"?"栅格":"语义"}图层工具；点位操作已暂停`:"已启用点位设置；图层绘制已暂停");
    render();
  }
  function cancelPointPick(announce=true){s.pointPickMode="";s.pointPickStep=0;s.pointPickTargetId="";s.pointPickAnchor=null;s.pointPickCursor=null;ui["pick-point"].textContent="点击取点";ui["pick-point"].classList.remove("is-picking");canvas.classList.remove("is-point-picking");if(announce)message("已取消点位取点");render();}
  function startPointPick(mode,targetId=""){
    activatePanel("points",false);
    if(mode==="add"&&!ui["point-name"].value.trim()){message("请先填写点位名称",true);ui["point-name"].focus();return;}
    s.pointPickMode=mode;s.pointPickStep=0;s.pointPickTargetId=targetId;s.pointPickAnchor=null;s.pointPickCursor=null;
    ui["pick-point"].textContent="取消取点";ui["pick-point"].classList.add("is-picking");canvas.classList.add("is-point-picking");
    if(targetId)s.selectedPointId=targetId;
    pointList();render();message(mode==="replace"?"重选点位：请先点击新位置":"新增点位：请先点击地图位置");
  }
  function resetToolState() {
    ui.layer.value="raster";
    tools();
    ui.tool.value="paint";
    ui["raster-value"].value="0";
    activatePanel("layers",false);
  }
  function semanticOptions(labels) {
    const rows=labels.length?labels:[{name:"语义区域",color:"#ef4444"}];
    const normalized=rows.map((row,index)=>{
      const value=/^#[0-9a-f]{6}$/i.test(row.color||row.hex||"")?(row.color||row.hex):"#ef4444";
      return{value,name:String(row.name||row.label||`标签 ${index+1}`)};
    });
    ui["semantic-label"].replaceChildren(...normalized.map(({value,name})=>{const option=document.createElement("option");
      option.value=value;option.dataset.name=name;option.textContent=`■  ${name}  ${value}`;
      return option;
    }));
    ui["semantic-menu"].replaceChildren(...normalized.map(({value,name})=>{const button=document.createElement("button");button.type="button";button.role="option";button.dataset.semanticValue=value;button.innerHTML=`<i style="background:${value}"></i><span>${esc(name)}</span><code>${value}</code>`;return button;}));
    semanticSelectionStyle();
  }
  function contrastColor(value){const hex=String(value).replace("#","");const r=parseInt(hex.slice(0,2),16),g=parseInt(hex.slice(2,4),16),b=parseInt(hex.slice(4,6),16);return(r*299+g*587+b*114)/1000>150?"#0f172a":"#f8fafc";}
  function semanticSelectionStyle(){const option=ui["semantic-label"].selectedOptions[0],value=ui["semantic-label"].value;ui["semantic-selected-swatch"].style.backgroundColor=value;ui["semantic-selected-name"].textContent=option?.dataset.name||"语义区域";ui["semantic-selected-value"].textContent=value;ui["semantic-menu"].querySelectorAll("[data-semantic-value]").forEach((button)=>{const selected=button.dataset.semanticValue===value;button.classList.toggle("is-selected",selected);button.setAttribute("aria-selected",String(selected));});}
  function setSemanticMenu(open){ui["semantic-menu"].hidden=!open;ui["semantic-trigger"].setAttribute("aria-expanded",String(open));}
  function drawMeterGrid(){
    if(!ui["show-meter-grid"].checked||!s.meta?.resolution)return;
    const step=.5/Number(s.meta.resolution);if(!Number.isFinite(step)||step<=0)return;
    ctx.save();ctx.lineWidth=1/s.scale;
    for(let x=0,index=0;x<=s.pgm.width;x+=step,index+=1){ctx.strokeStyle=index%2===0?"rgba(34,197,94,.38)":"rgba(34,197,94,.2)";ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,s.pgm.height);ctx.stroke();}
    for(let y=0,index=0;y<=s.pgm.height;y+=step,index+=1){ctx.strokeStyle=index%2===0?"rgba(34,197,94,.38)":"rgba(34,197,94,.2)";ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(s.pgm.width,y);ctx.stroke();}
    ctx.restore();
  }
  function resize(){const rect=$("canvas-wrap").getBoundingClientRect(),dpr=devicePixelRatio||1;canvas.width=Math.max(1,Math.round(rect.width*dpr));canvas.height=Math.max(1,Math.round(rect.height*dpr));canvas.style.width=`${rect.width}px`;canvas.style.height=`${rect.height}px`;render();}
  function fit(){if(!s.pgm)return;const rect=canvas.getBoundingClientRect();s.scale=Math.min(rect.width/s.pgm.width,rect.height/s.pgm.height)*.94;s.panX=(rect.width-s.pgm.width*s.scale)/2;s.panY=(rect.height-s.pgm.height*s.scale)/2;render();}
  function pixel(event){const rect=canvas.getBoundingClientRect();return{x:(event.clientX-rect.left-s.panX)/s.scale,y:(event.clientY-rect.top-s.panY)/s.scale};}
  function toWorld(p){return{x:s.meta.origin[0]+p.x*s.meta.resolution,y:s.meta.origin[1]+(s.pgm.height-p.y)*s.meta.resolution};}
  function toPixel(point){return{x:(Number(point.x)-s.meta.origin[0])/s.meta.resolution,y:s.pgm.height-(Number(point.y)-s.meta.origin[1])/s.meta.resolution};}
  function pointTypeLabel(type){return type==="elevator"||type==="elevator_inside"?"电梯内点":type==="elevator_waiting"?"电梯等待点":type==="standby"?"待机点":type==="relocalization"?"重定位点":"自定义点位";}
  function selectPoint(point){s.selectedPointId=point?.id||"";pointList();render();}
  function drawPoint(point) {
    const p=toPixel(point),unit=1/s.scale;
    const pointColor=point.type==="elevator"||point.type==="elevator_inside"?"#a78bfa":point.type==="elevator_waiting"?"#f472b6":point.type==="standby"?"#22c55e":point.type==="relocalization"?"#38bdf8":"#fb923c";
    const radius=(point.type==="custom"||point.type==="relocalization"?5:7)*unit,selected=point.id===s.selectedPointId;
    ctx.save();ctx.translate(p.x,p.y);
    if(selected){ctx.strokeStyle="#facc15";ctx.lineWidth=3*unit;ctx.beginPath();ctx.arc(0,0,radius+4*unit,0,Math.PI*2);ctx.stroke();}
    ctx.fillStyle=pointColor;ctx.strokeStyle="#0f172a";ctx.lineWidth=2*unit;ctx.beginPath();ctx.arc(0,0,radius,0,Math.PI*2);ctx.fill();ctx.stroke();
    ctx.rotate(-Number(point.yaw||0));
    const arrowScale=radius/(7*unit);ctx.scale(arrowScale,arrowScale);ctx.fillStyle="#0f172a";ctx.beginPath();ctx.moveTo(5*unit,0);ctx.lineTo(-3*unit,-3.5*unit);ctx.lineTo(-1*unit,0);ctx.lineTo(-3*unit,3.5*unit);ctx.closePath();ctx.fill();ctx.restore();
    const label=String(point.name||point.id),labelX=p.x+9*unit,labelBaseline=p.y-7*unit;
    ctx.save();ctx.font=`${11*unit}px "Fira Code", monospace`;const labelWidth=ctx.measureText(label).width+10*unit,labelTop=labelBaseline-12*unit;
    ctx.fillStyle="rgba(15, 23, 42, 0.88)";ctx.strokeStyle=selected?"#facc15":pointColor;ctx.lineWidth=1*unit;ctx.beginPath();
    if(typeof ctx.roundRect==="function")ctx.roundRect(labelX-5*unit,labelTop,labelWidth,17*unit,5*unit);else ctx.rect(labelX-5*unit,labelTop,labelWidth,17*unit);
    ctx.fill();ctx.stroke();ctx.fillStyle="#f8fafc";ctx.fillText(label,labelX,labelBaseline);ctx.restore();
  }
  function drawPointPickPreview(){
    if(s.pointPickStep!==1||!s.pointPickAnchor)return;
    const cursor=s.pointPickCursor||s.pointPickAnchor,yaw=Math.atan2(cursor.y-s.pointPickAnchor.y,cursor.x-s.pointPickAnchor.x);
    const target=s.points.find((point)=>point.id===s.pointPickTargetId);
    const preview={id:"__point_pick_preview__",name:target?.name||ui["point-name"].value.trim()||"新点位",type:target?.type||ui["point-type"].value,x:s.pointPickAnchor.x,y:s.pointPickAnchor.y,yaw};
    const a=toPixel(s.pointPickAnchor),b=toPixel(cursor),unit=1/s.scale;
    ctx.save();ctx.strokeStyle="#38bdf8";ctx.lineWidth=2*unit;ctx.setLineDash([6*unit,4*unit]);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.restore();
    drawPoint(preview);
  }
  function render() {
    const rect=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle="#020617";ctx.fillRect(0,0,rect.width,rect.height);if(!s.raster)return;
    ctx.save();ctx.translate(s.panX,s.panY);ctx.scale(s.scale,s.scale);ctx.imageSmoothingEnabled=false;if(ui["show-grid"].checked)ctx.drawImage(s.raster,0,0);
    if(ui["show-semantic"].checked){ctx.globalAlpha=.55;ctx.drawImage(s.semantic,0,0);ctx.globalAlpha=1;}
    drawMeterGrid();
    if(ui["show-points"].checked)s.points.forEach(drawPoint);
    drawPointPickPreview();
    ctx.restore();
  }
  function pointList() {
    ui["point-list"].innerHTML=s.points.map((point)=>`<div class="map-waypoint-item${point.id===s.selectedPointId?" is-selected":""}" data-select-point="${esc(point.id)}"><div><strong>${esc(point.name||point.id)}</strong><span>${pointTypeLabel(point.type)}</span></div><div class="map-waypoint-item__actions"><button type="button" class="btn-secondary" data-repick-point="${esc(point.id)}">重选</button><button type="button" data-delete-point="${esc(point.id)}">删除</button></div></div>`).join("")||'<span class="reloc-message-block">本地图暂无点位</span>';
    ui.metadata.innerHTML=[["尺寸",`${s.pgm.width} × ${s.pgm.height} px`],["分辨率",`${s.meta.resolution} m/px`],["原点",s.meta.origin.join(", ")],["点位",s.points.length]].map(([key,value])=>`<div><dt>${key}</dt><dd>${esc(value)}</dd></div>`).join("");
  }
  function nearest(p){let found=null,distance=16/s.scale;s.points.forEach((point)=>{const at=toPixel(point),d=Math.hypot(at.x-p.x,at.y-p.y);if(d<=distance){found=point;distance=d;}});return found;}
  function pointId(name){
    const base=String(name||"point").trim().toLowerCase().replace(/[^a-z0-9_-]+/g,"_").replace(/^_+|_+$/g,"").slice(0,48)||"point";
    const stamp=Date.now().toString(36);let id=`${base}_${stamp}`,suffix=2;
    while(s.points.some((point)=>point.id===id)){id=`${base}_${stamp}_${suffix}`;suffix+=1;}
    return id.slice(0,64);
  }
  function edit(event,first) {
    const p=pixel(event);if(!s.pgm||p.x<0||p.y<0||p.x>=s.pgm.width||p.y>=s.pgm.height)return;
    const pointMode=s.activePanel==="points";
    if(pointMode){
      const world=toWorld(p);
      if(s.pointPickMode){
        if(!first)return;
        if(s.pointPickStep===0){s.pointPickAnchor=world;s.pointPickCursor=world;s.pointPickStep=1;message("位置已确定，请再次点击地图设置方向");render();return;}
        const yaw=Math.atan2(world.y-s.pointPickAnchor.y,world.x-s.pointPickAnchor.x);snapshot("points");
        if(s.pointPickMode==="replace"){
          const target=s.points.find((point)=>point.id===s.pointPickTargetId);if(target){Object.assign(target,s.pointPickAnchor,{yaw});s.selectedPointId=target.id;}
        }else{
          const name=ui["point-name"].value.trim(),point={id:pointId(name),name,type:ui["point-type"].value,...s.pointPickAnchor,yaw};s.points.push(point);s.selectedPointId=point.id;ui["point-name"].value="";
        }
        mark("points");cancelPointPick(false);pointList();render();message("点位位置和方向已更新，保存后生效");return;
      }
      if(first){const target=nearest(p);if(target){s.movingId=target.id;s.moveSnapshotTaken=false;selectPoint(target);message(`已选中点位：${target.name||target.id}；按住拖动可移动`);}return;}
      const target=s.points.find((point)=>point.id===s.movingId);if(target){if(!s.moveSnapshotTaken){snapshot("points");s.moveSnapshotTaken=true;}Object.assign(target,world);mark("points");pointList();render();}return;
    }
    const layer=ui.layer.value,tool=ui.tool.value;
    const target=layer==="semantic"?s.semantic:s.raster;if(first)snapshot(layer);const draw=target.getContext("2d");draw.beginPath();draw.arc(p.x,p.y,Math.max(1,Number(ui.brush.value)),0,Math.PI*2);
    if(layer==="raster"){const value=tool==="erase"?254:Number(ui["raster-value"].value);draw.fillStyle=`rgb(${value},${value},${value})`;}
    else{draw.globalCompositeOperation=tool==="erase"?"destination-out":"source-over";draw.fillStyle=ui["semantic-label"].value;}
    draw.fill();draw.globalCompositeOperation="source-over";mark(layer);render();
  }
  function pgmUrl(source) {
    const pixels=source.getContext("2d").getImageData(0,0,source.width,source.height).data,header=`P5\n${source.width} ${source.height}\n255\n`,bytes=new Uint8Array(header.length+source.width*source.height);
    for(let i=0;i<header.length;i+=1)bytes[i]=header.charCodeAt(i);for(let i=0;i<source.width*source.height;i+=1)bytes[header.length+i]=pixels[i*4];
    let binary="";for(let i=0;i<bytes.length;i+=0x8000)binary+=String.fromCharCode(...bytes.subarray(i,i+0x8000));return"data:image/x-portable-graymap;base64,"+btoa(binary);
  }
  async function saveLayers() {
    const layers=["raster","semantic"].filter((layer)=>s.dirty.has(layer));if(!layers.length)return;
    ui["layer-save"].disabled=true;message("正在保存图层…");
    try{
      for(const layer of layers){
        const body=layer==="raster"?{pgm_data:pgmUrl(s.raster)}:{png_data:s.semantic.toDataURL("image/png")};
        await json(`/api/maps/${encodeURIComponent(s.name)}/assets/${layer}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
        if(layer==="raster")s.savedRaster=cloneCanvas(s.raster);else s.savedSemantic=cloneCanvas(s.semantic);
        s.dirty.delete(layer);s.undo=s.undo.filter((entry)=>entry.layer!==layer);
      }
      sync();message("图层修改已保存");
    }catch(error){message(`图层保存失败：${error.message}`,true);sync();}
  }
  async function savePoints() {
    if(!s.dirty.has("points"))return;
    ui["point-save"].disabled=true;message("正在保存点位…");
    try{
      const out=await json(`/api/maps/${encodeURIComponent(s.name)}/assets/points`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({points:s.points})});
      s.points=clonePoints(Array.isArray(out.points)?out.points:s.points);s.savedPoints=clonePoints(s.points);s.dirty.delete("points");s.undo=s.undo.filter((entry)=>entry.layer!=="points");
      pointList();sync();render();message("点位修改已保存");
    }catch(error){message(`点位保存失败：${error.message}`,true);sync();}
  }
  function undoScope(scope){
    let index=-1;for(let i=s.undo.length-1;i>=0;i-=1){const pointEntry=s.undo[i].layer==="points";if((scope==="points"&&pointEntry)||(scope==="layers"&&!pointEntry)){index=i;break;}}
    if(index<0)return;const [entry]=s.undo.splice(index,1),wasDirty=entry.dirty.includes(entry.layer);
    if(entry.layer==="points"){s.points=clonePoints(entry.value);if(wasDirty)s.dirty.add("points");else s.dirty.delete("points");pointList();}
    else{(entry.layer==="semantic"?s.semantic:s.raster).getContext("2d").putImageData(entry.value,0,0);if(wasDirty)s.dirty.add(entry.layer);else s.dirty.delete(entry.layer);}
    sync();render();message(scope==="points"?"已撤销上一步点位修改":"已撤销上一步图层修改");
  }
  function discardLayers(){
    if(!(s.dirty.has("raster")||s.dirty.has("semantic"))||!confirm("确定放弃全部未保存的图层修改吗？"))return;
    s.raster=cloneCanvas(s.savedRaster);s.semantic=cloneCanvas(s.savedSemantic);s.dirty.delete("raster");s.dirty.delete("semantic");s.undo=s.undo.filter((entry)=>entry.layer==="points");sync();render();message("已取消图层修改");
  }
  function discardPoints(){
    if(!s.dirty.has("points")||!confirm("确定放弃全部未保存的点位修改吗？"))return;
    cancelPointPick(false);s.points=clonePoints(s.savedPoints);s.selectedPointId="";s.dirty.delete("points");s.undo=s.undo.filter((entry)=>entry.layer!=="points");pointList();sync();render();message("已取消点位修改");
  }
  async function loadMap(mapName) {
    mapName=String(mapName||"").trim();
    if(!mapName){message("请输入地图名称",true);ui["map-name"].focus();return;}
    if(mapName.endsWith("_mapping")){message("建图中的临时地图不可编辑，请输入已保存地图名称",true);ui["map-name"].focus();return;}
    if(s.dirty.size&&mapName!==s.name&&!confirm("当前地图有未保存修改，确定切换地图吗？"))return;
    ui.loading.hidden=false;ui.loading.textContent=`正在加载 ${mapName}…`;s.name=mapName;s.pgm=null;s.dirty.clear();s.undo.length=0;sync();const token=++s.loadToken;
    try{const[map,assets]=await Promise.all([json(`/api/maps/${encodeURIComponent(mapName)}`,{cache:"no-store"}),json(`/api/maps/${encodeURIComponent(mapName)}/assets`,{cache:"no-store"})]);if(token!==s.loadToken)return;if(!map.pgm||typeof map.yaml!=="string")throw new Error("地图数据格式不正确");
      s.pgm=map.pgm;s.meta=parseYaml(map.yaml);s.raster=rasterFromPgm(map.pgm);s.semantic=await semanticCanvas(assets.semantic_png,map.pgm.width,map.pgm.height);s.points=clonePoints(Array.isArray(assets.points)?assets.points:[]);s.selectedPointId="";cancelPointPick(false);s.savedRaster=cloneCanvas(s.raster);s.savedSemantic=cloneCanvas(s.semantic);s.savedPoints=clonePoints(s.points);
      semanticOptions(assets.semantic_legend?.labels||[]);resetToolState();pointList();ui.loading.hidden=true;requestAnimationFrame(()=>{resize();fit();});message("地图已加载；默认使用栅格障碍画笔");sync();
    }catch(error){ui.loading.hidden=false;ui.loading.textContent=`加载失败：${error.message}`;message(`加载失败：${error.message}`,true);}
  }
  async function loadMapChoices(defaultName) {
    ui["map-name"].disabled=true;ui["load-map"].disabled=true;
    const payload=await json("/api/floors",{cache:"no-store"});
    const floors=(Array.isArray(payload.floors)?payload.floors:[]).filter((name)=>name&&!String(name).endsWith("_mapping"));
    ui["map-name"].replaceChildren(...floors.map((name)=>{const option=document.createElement("option");option.value=name;option.textContent=name;return option;}));
    if(!floors.length){const option=document.createElement("option");option.value="";option.textContent="暂无已保存地图";ui["map-name"].appendChild(option);}
    ui["map-name"].value=floors.includes(defaultName)?defaultName:(floors[0]||"");
    ui["map-name"].disabled=!floors.length;ui["load-map"].disabled=!floors.length;
    return ui["map-name"].value;
  }
  async function open(mapName) {
    root.hidden=false;document.body.classList.add("standalone-map-editor-open");
    const defaultName=String(mapName||"").trim();
    ui.loading.hidden=false;ui.loading.textContent="正在读取地图列表…";
    try{const selected=await loadMapChoices(defaultName);if(selected)await loadMap(selected);else message("没有可编辑的已保存地图",true);}catch(error){ui.loading.textContent=`地图列表加载失败：${error.message}`;message(`地图列表加载失败：${error.message}`,true);}
  }
  function close(){if(s.dirty.size&&!confirm("存在未保存修改，确定关闭编辑器吗？"))return;root.hidden=true;document.body.classList.remove("standalone-map-editor-open");s.painting=false;s.panning=false;}
  canvas.addEventListener("pointerdown",(event)=>{if(!s.pgm)return;canvas.setPointerCapture(event.pointerId);if(event.shiftKey||event.button===1){s.panning=true;s.lastX=event.clientX;s.lastY=event.clientY;}else if(event.button===0){s.painting=true;edit(event,true);}});
  canvas.addEventListener("pointermove",(event)=>{if(s.pgm){const p=pixel(event),world=toWorld(p);ui.coordinates.textContent=`X ${world.x.toFixed(2)} · Y ${world.y.toFixed(2)} · 像素 ${Math.floor(p.x)}, ${Math.floor(p.y)}`;if(s.pointPickStep===1){s.pointPickCursor=world;render();}}if(s.panning){s.panX+=event.clientX-s.lastX;s.panY+=event.clientY-s.lastY;s.lastX=event.clientX;s.lastY=event.clientY;render();}else if(s.painting)edit(event,false);});
  const pointerUp=()=>{s.painting=false;s.panning=false;s.movingId="";s.moveSnapshotTaken=false;};canvas.addEventListener("pointerup",pointerUp);canvas.addEventListener("pointercancel",pointerUp);
  canvas.addEventListener("wheel",(event)=>{if(!s.pgm)return;event.preventDefault();const before=pixel(event),rect=canvas.getBoundingClientRect();s.scale=Math.min(30,Math.max(.08,s.scale*(event.deltaY<0?1.12:1/1.12)));s.panX=event.clientX-rect.left-before.x*s.scale;s.panY=event.clientY-rect.top-before.y*s.scale;render();},{passive:false});
  ui["layer-section"].addEventListener("pointerdown",()=>activatePanel("layers"));
  ui["point-section"].addEventListener("pointerdown",()=>activatePanel("points"));
  ui.layer.addEventListener("change",()=>{tools();activatePanel("layers");});
  ui["pick-point"].addEventListener("click",()=>{if(s.pointPickMode)cancelPointPick();else startPointPick("add");});
  ui["semantic-trigger"].addEventListener("click",(event)=>{event.stopPropagation();setSemanticMenu(ui["semantic-menu"].hidden);});
  ui["semantic-menu"].addEventListener("click",(event)=>{const button=event.target.closest("[data-semantic-value]");if(!button)return;ui["semantic-label"].value=button.dataset.semanticValue;semanticSelectionStyle();setSemanticMenu(false);});
  document.addEventListener("click",(event)=>{if(!ui["semantic-picker"].contains(event.target))setSemanticMenu(false);});
  ui["semantic-label"].addEventListener("change",semanticSelectionStyle);[ui["show-grid"],ui["show-meter-grid"],ui["show-semantic"],ui["show-points"]].forEach((node)=>node.addEventListener("change",render));
  ui["load-map"].addEventListener("click",()=>loadMap(ui["map-name"].value));
  ui["map-name"].addEventListener("change",()=>loadMap(ui["map-name"].value));
  ui["point-list"].addEventListener("click",(event)=>{const repick=event.target.closest("[data-repick-point]");if(repick){startPointPick("replace",repick.dataset.repickPoint);return;}const button=event.target.closest("[data-delete-point]");if(button){snapshot("points");s.points=s.points.filter((point)=>point.id!==button.dataset.deletePoint);if(s.selectedPointId===button.dataset.deletePoint)s.selectedPointId="";mark("points");pointList();render();return;}const row=event.target.closest("[data-select-point]");if(row){selectPoint(s.points.find((point)=>point.id===row.dataset.selectPoint));message(`已选中点位：${row.querySelector("strong")?.textContent||row.dataset.selectPoint}；可在地图上按住拖动`);}});
  $("zoom-in").onclick=()=>{s.scale=Math.min(30,s.scale*1.2);render();};$("zoom-out").onclick=()=>{s.scale=Math.max(.08,s.scale/1.2);render();};$("reset-view").onclick=fit;$("close").onclick=close;$("backdrop").onclick=close;
  ui["layer-undo"].onclick=()=>undoScope("layers");ui["layer-discard"].onclick=discardLayers;ui["layer-save"].onclick=saveLayers;
  ui["point-undo"].onclick=()=>undoScope("points");ui["point-discard"].onclick=discardPoints;ui["point-save"].onclick=savePoints;
  document.addEventListener("keydown",(event)=>{if(root.hidden)return;if(event.key==="Escape"&&!ui["semantic-menu"].hidden){setSemanticMenu(false);ui["semantic-trigger"].focus();return;}if(event.key==="Escape"&&s.pointPickMode){cancelPointPick();return;}const typing=/INPUT|TEXTAREA|SELECT/.test(event.target?.tagName||"");if((event.ctrlKey||event.metaKey)&&event.key.toLowerCase()==="z"&&!typing){event.preventDefault();undoScope(s.activePanel==="points"?"points":"layers");}else if(event.key==="Escape")close();});
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(resize).observe($("canvas-wrap"));
  } else {
    window.addEventListener("resize", resize);
  }
  resetToolState();sync();
  window.StandaloneMapEditor=Object.freeze({open,close,load:loadMap,get mapName(){return s.name;},get dirty(){return s.dirty.size>0;}});
}());
