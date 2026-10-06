const {chromium}=require('playwright');
const assert=require('node:assert/strict');
const {pathToFileURL}=require('node:url');
const root='D:/RacecarWork/reports/marked-route-dashboard';
(async()=>{
 const browser=await chromium.launch({executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1450,height:1100}});const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto(pathToFileURL(root+'/marked-route-20260930-192905-242091.html').href);
  await page.waitForFunction(()=>document.body.dataset.ready==='true');
  assert.equal(await page.locator('.js-plotly-plot').count(),10);
  assert(await page.evaluate(()=>document.getElementById('speed').data[1].y.some(Number.isFinite)));
  await page.locator('#time').evaluate(el=>{el.value='10';el.dispatchEvent(new Event('input'))});
  await page.waitForFunction(()=>document.getElementById('timeLabel').textContent==='10.00 s'&&!busy);
  assert(await page.evaluate(()=>document.getElementById('map').data.find(t=>t.name==='定位车头方向').x.length>0));
  assert.equal(await page.evaluate(()=>document.getElementById('speed').layout.shapes[0].x0),10);
  await page.evaluate(()=>Plotly.relayout('speed',{'xaxis.range':[5,15]}));
  await page.waitForFunction(()=>document.getElementById('servo').layout.xaxis.range[0]===5);
  await page.locator('#reset').click();
  await page.waitForFunction(()=>document.getElementById('servo').layout.xaxis.autorange===true);
  const download=page.waitForEvent('download');await page.locator('#export').click();assert((await download).suggestedFilename().endsWith('-control.csv'));
  await page.screenshot({path:root+'/preview-driving.png'});
  await page.goto(pathToFileURL(root+'/index.html').href);
  await page.waitForFunction(()=>document.body.dataset.ready==='true');
  const options=await page.locator('#history option').count();assert(options>=3);
  await page.selectOption('#history','marked-route-20260930-203603-464229.html');
  await page.waitForFunction(()=>document.body.dataset.ready==='true');
  assert.equal(await page.locator('#status').innerText(),'启动失败');
  assert.equal(await page.locator('#duration').innerText(),'未发车');
  assert(await page.locator('#time').isDisabled());
  await page.screenshot({path:root+'/preview-startup-failure.png'});
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({charts:10,time_link:true,zoom_link:true,history:true,csv:true,failed_run:true,page_errors:errors}));
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
