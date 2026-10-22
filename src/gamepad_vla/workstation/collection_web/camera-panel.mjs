// Independent of simulation setup, overlays and WebCodecs availability.
export const cameraActions = Object.freeze({
  cameraStart: '/api/v1/camera/start',
  cameraStop: '/api/v1/camera/stop',
  cameraSnapshot: '/api/v1/camera/snapshot',
  cameraRecordStart: '/api/v1/camera/record/start',
  cameraRecordStop: '/api/v1/camera/record/stop',
});

export function installCameraPanel(doc = document, fetcher = fetch) {
  const $ = id => doc.getElementById(id);
  let mode = 'idle', busy = false, imageURL = null, count = 0, begin = performance.now(), signature = '';
  let displayedFPS = 0;
  const metrics = {frames: 0, fps: 0, age_s: null};
  async function api(path, body) {
    const r = await fetcher(path, body === undefined ? {} : {
      method: 'POST', headers: {'Content-Type': 'application/json', 'X-Gamepad-VLA-Client': 'web'},
      body: JSON.stringify(body),
    });
    const value = await r.json();
    if (!r.ok) throw Error(value.message_zh || value.error || r.statusText);
    return value;
  }
  function buttons() {
    $('cameraStart').disabled = busy || mode !== 'idle';
    $('cameraStop').disabled = busy || mode === 'idle';
    $('cameraSnapshot').disabled = busy || mode !== 'preview';
    $('cameraRecordStart').disabled = busy || !['idle', 'preview'].includes(mode);
    $('cameraRecordStop').disabled = busy || mode !== 'recording';
  }
  function report(value) {
    if (value) $('cameraReport').textContent = JSON.stringify(value, null, 2);
  }
  async function photos() {
    const list = await api('/api/v1/camera/photos');
    const next = JSON.stringify(list);
    if (next === signature) return;
    signature = next;
    $('cameraPhotos').replaceChildren();
    for (const photo of list) {
      const figure = doc.createElement('figure'), img = doc.createElement('img'), caption = doc.createElement('figcaption');
      img.src = '/api/v1/camera/photos/' + encodeURIComponent(photo.name + '.png'); img.alt = photo.label || '标定照片'; img.loading = 'lazy';
      caption.textContent = `${photo.label} · ${photo.captured_at_utc}`;
      figure.append(img, caption);
      for (const ext of ['png', 'json']) {
        const a = doc.createElement('a'); a.href = '/api/v1/camera/photos/' + encodeURIComponent(photo.name + '.' + ext);
        a.download = photo.name + '.' + ext; a.textContent = ` 下载 ${ext.toUpperCase()} `; figure.append(a);
      }
      $('cameraPhotos').append(figure);
    }
  }
  async function refresh() {
    const s = await api('/api/v1/camera/status');
    mode = s.mode;
    $('cameraMode').textContent = ({idle:'空闲', preview:'预览中', recording:'录像中', starting:'正在启动预览', starting_recording:'正在切换至录像', stopping_recording:'正在保存录像', failed:'相机服务失败'})[mode] || mode;
    if (s.error) $('cameraMessage').textContent = s.error;
    metrics.age_s = s.age_s;
    $('cameraInfo').textContent = `serial：${s.serial || '未配置'} · ${s.width} × ${s.height}\nK：${s.K ? JSON.stringify(s.K) : '等待 camera_info'}`;
    $('cameraOutput').textContent = s.output_dir ? `照片目录：${s.output_dir}` : '启动相机后创建标定会话。';
    $('cameraMetrics').textContent = mode === 'preview'
      ? `浏览器预览 ${displayedFPS.toFixed(1)} fps · 相机 ${Number(s.fps || 0).toFixed(1)} fps · 画面年龄 ${s.age_s == null ? '未知' : s.age_s.toFixed(3) + ' s'}`
      : '预览已暂停';
    report(s.recording_report);
    buttons();
  }
  for (const [id, path] of Object.entries(cameraActions)) {
    $(id).onclick = async () => {
      busy = true; buttons(); $('cameraMessage').textContent = '正在处理…';
      try {
        const value = await api(path, id === 'cameraSnapshot' ? {label: $('cameraLabel').value} : {});
        if (id === 'cameraSnapshot') {
          $('cameraLastPhoto').src = '/api/v1/camera/photos/' + encodeURIComponent(value.name + '.png');
          $('cameraLastPhoto').hidden = false;
        }
        report(value.recording_report);
        $('cameraMessage').textContent = id === 'cameraSnapshot' ? `已保存：${value.name}` : '操作完成';
        await refresh(); await photos();
      } catch (e) { $('cameraMessage').textContent = e.message; }
      finally { busy = false; buttons(); }
    };
  }
  $('cameraGrid').onchange = () => $('cameraGuide').classList.toggle('grid', $('cameraGrid').checked);
  $('cameraCross').onchange = () => $('cameraGuide').classList.toggle('cross', $('cameraCross').checked);
  async function preview() {
    const start = performance.now();
    if (mode === 'preview' && $('cameraPanel').open) {
      try {
        const r = await fetcher('/api/v1/camera/preview.jpg?t=' + Date.now());
        if (!r.ok) throw Error('等待新鲜相机画面');
        const next = URL.createObjectURL(await r.blob());
        $('cameraPreview').src = next;
        if (imageURL) URL.revokeObjectURL(imageURL);
        imageURL = next; count++; metrics.frames++;
        const elapsed = (performance.now() - begin) / 1000;
        if (elapsed >= 1) {
          displayedFPS = count / elapsed; metrics.fps = displayedFPS;
          count = 0; begin = performance.now();
        }
      } catch (e) { $('cameraMessage').textContent = e.message; }
    } else { count = 0; begin = performance.now(); }
    setTimeout(preview, Math.max(0, 66 - (performance.now() - start)));
  }
  async function poll() {
    try { await refresh(); await photos(); }
    catch (e) { $('cameraMessage').textContent = e.message; }
    setTimeout(poll, 500);
  }
  buttons(); preview(); poll();
  return metrics;
}
if (typeof document !== 'undefined') window.cameraPanelMetrics = installCameraPanel();
