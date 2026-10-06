const {chromium}=require('playwright');
const assert=require('node:assert/strict');
const path=require('node:path');
const {pathToFileURL}=require('node:url');
(async()=>{
const browser=await chromium.launch({executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless:true});
try{
 const page=await browser.newPage({viewport:{width:1500,height:1160},deviceScaleFactor:1});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto(pathToFileURL('D:/RacecarWork/reports/motion-dashboard/index.html').href);
 await page.waitForFunction(()=>document.body.dataset.renderedRun);
 await page.locator('#speed .main-svg').first().waitFor();
 assert.equal(await page.locator('.js-plotly-plot').count(),9);
 assert(await page.evaluate(()=>document.getElementById('path').data.some(t=>t.name==='估计车头方向'&&t.x.length>0)));
 assert(await page.evaluate(()=>document.getElementById('gyro').data.some(t=>t.y.some(Number.isFinite))));
 const hoverIndex=await page.evaluate(()=>{
  const trace=document.getElementById('path').data[1];
  return trace.x.findIndex((x,i)=>Number.isFinite(x)&&trace.customdata[i][2]!=='无数据'&&trace.customdata[i][3]!=='无数据');
 });
 assert(hoverIndex>=0);
 await page.evaluate(i=>Plotly.Fx.hover('path',[{curveNumber:1,pointNumber:i}]),hoverIndex);
 await page.waitForFunction(()=>document.querySelector('#path .hoverlayer')?.textContent.includes('IMU 航向角'));
 const hoverText=await page.locator('#path .hoverlayer').textContent();
 assert(hoverText.includes('车头角')&&hoverText.includes('轨迹移动方向角'));
 await page.locator('#path').screenshot({path:'D:/RacecarWork/reports/motion-dashboard/path-hover-preview.png'});
 assert(!await page.locator('#peak').innerText().then(s=>s.includes('—')));
 const historyCount=await page.locator('#run option').count();
 if(historyCount>1){
  const before=await page.locator('body').getAttribute('data-rendered-run');
  await page.selectOption('#run','1');
  await page.waitForFunction(old=>document.body.dataset.renderedRun!==old,before);
  await page.selectOption('#run','0');
  await page.waitForFunction(old=>document.body.dataset.renderedRun===old,before);
 }
 // Exercise actual box zoom mouse interaction and check linked time axes.
 const box=await page.locator('#speed').boundingBox();
 await page.mouse.move(box.x+box.width*.30,box.y+100);
 await page.mouse.down();await page.mouse.move(box.x+box.width*.60,box.y+210,{steps:12});await page.mouse.up();
 await page.waitForFunction(()=>{const a=document.getElementById('speed').layout.xaxis,b=document.getElementById('motor').layout.xaxis;return a.autorange===false&&Math.abs(a.range[0]-b.range[0])<1e-6});
 const range=await page.evaluate(()=>document.getElementById('speed').layout.xaxis.range);
 await page.locator('#reset').click();
 await page.waitForFunction(()=>document.getElementById('speed').layout.xaxis.autorange===true);
 const [download]=await Promise.all([page.waitForEvent('download'),page.locator('#export').click()]);
 assert(download.suggestedFilename().endsWith('-motion.csv'));
 await download.saveAs('D:/RacecarWork/reports/motion-dashboard/browser-export-check.csv');
 const [imuDownload]=await Promise.all([page.waitForEvent('download'),page.locator('#exportImu').click()]);
 assert(imuDownload.suggestedFilename().endsWith('-imu.csv'));
 await page.screenshot({path:'D:/RacecarWork/reports/motion-dashboard/preview.png',fullPage:true});
 assert.deepEqual(errors,[]);
 console.log(JSON.stringify({charts:9,historyCount,headingArrows:true,perPointAngles:true,hoverText,imuExport:true,boxZoomRange:range,linkedZoom:true,csvExport:download.suggestedFilename(),pageErrors:errors}));
}finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
