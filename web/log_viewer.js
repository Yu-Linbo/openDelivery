/* ROS log viewer adapted from tools/ros_log_viewer; embedded in bag playback, using the shared playback clock. */
(() => {
  const levels = ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL', 'RAW'];
  const state = { rows: [], levels: new Set(levels), page: 0, request: 0, time: 0, signature: '', filtered: [], queryError: false, loading: true, emptyMessage: '' };
  const $ = id => document.getElementById(`lv-${id}`);
  const pageSize = 250;
  function parseLine(raw, index) {
    const text = raw.replace(/\x1b\[[0-9;]*m/g, '').replace(/^\[[^\]]+-\d+\]\s+(?=\[(?:DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\])/, '');
    const legacy = text.match(/^\[([^\]]+)\] rosout level=(\d+) name=(.*?) file=(.*?) line=(\d+) msg=(.*)$/);
    const standard = text.match(/^\s*\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*\[([\d.]+)\]\s*\[([^\]]+)\]:\s*(.*)$/i);
    const extended = text.match(/^\s*([\d.]+)\s+\[\s*(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\s*\]\s+\[([^\]]*)\](?:\[([^\]]*)\])?(?:\[(\d+)\])?:?\s*(.*)$/i);
    const launch = text.match(/^\s*\[([\d.]+)\]\s*\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*\[([^\]]+)\]:?\s*(.*)$/i);
    const recorder = text.match(/^\[([^\]]+)\] (.*)$/);
    const short = text.match(/^\s*\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*\[([^\]]+)\]:?\s*(.*)$/i);
    let level = 'RAW', epoch = null, node = '', message = text;
    if (legacy) { level = ({10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'})[legacy[2]] || 'RAW'; epoch = Date.parse(legacy[1]) / 1000; node = legacy[3]; message = legacy[6] + (legacy[4] ? ` [source=${legacy[4]}:${legacy[5]}]` : ''); }
    else if (standard) [, level, epoch, node, message] = standard;
    else if (extended) { [, epoch, level, node] = extended; message = [extended[4], extended[5] && `L${extended[5]}`, extended[6]].filter(Boolean).join(' · '); }
    else if (launch) [, epoch, level, node, message] = launch;
    else if (short) [, level, node, message] = short;
    else if (recorder && Number.isFinite(Date.parse(recorder[1]))) { epoch = Date.parse(recorder[1]) / 1000; node = 'robot_log_recorder'; message = recorder[2]; }
    return { index: index + 1, raw, level: level.toUpperCase().replace('WARNING', 'WARN'), epoch: epoch === null ? null : Number(epoch), node, message };
  }
  // Exposed for parser verification and embedding, without DOM dependencies.
  window.ROSLogParser = { parseLine };
  function timeText(time) {
    if (time === null || !Number.isFinite(time)) return '原文';
    const minutes = Math.floor(time / 60);
    return String(minutes).padStart(2, '0') + ':' + (time % 60).toFixed(3).padStart(6, '0');
  }
  function markText(element, value, regex) {
    if (!regex) { element.textContent = value; return; }
    let start = 0, match; regex.lastIndex = 0;
    while ((match = regex.exec(value))) {
      element.append(document.createTextNode(value.slice(start, match.index)));
      const mark = document.createElement('mark'); mark.textContent = match[0]; element.append(mark);
      start = match.index + match[0].length;
      if (!match[0].length) regex.lastIndex++;
    }
    element.append(document.createTextNode(value.slice(start)));
  }
  function updateNodes() {
    const previous = $('node').value;
    const names = [...new Set(state.rows.map(row => row.node).filter(Boolean))].sort();
    $('node').replaceChildren(new Option('全部节点', ''), ...names.map(node => {
      const option = new Option(node, node); option.setAttribute('data-i18n-ignore', ''); return option;
    }));
    if (names.includes(previous)) $('node').value = previous;
  }
  function refilter() {
    state.page = 0; state.signature = ''; state.queryError = false;
    const query = $('search').value.trim();
    try { state.regex = query ? new RegExp($('regex').checked ? query : query.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'), 'ig') : null; }
    catch (_) { state.queryError = true; state.filtered = []; render(); return; }
    state.filtered = state.rows.filter(row => {
      if (!state.levels.has(row.level) || ($('node').value && row.node !== $('node').value)) return false;
      if (!state.regex) return true;
      state.regex.lastIndex = 0; return state.regex.test(row.raw);
    });
    render();
  }
  function render() {
    const rows = state.filtered;
    let active = -1;
    // Use the sorted synchronized prefix. Rows without a usable clock remain at the end.
    let low = 0, high = rows.length - 1;
    while (low <= high) {
      const middle = (low + high) >> 1;
      if (rows[middle].t !== null && rows[middle].t <= state.time) { active = middle; low = middle + 1; }
      else high = middle - 1;
    }
    const lastPage = Math.max(0, Math.ceil(rows.length / pageSize) - 1);
    if ($('follow').checked && active >= 0) state.page = Math.floor(active / pageSize);
    state.page = Math.min(state.page, lastPage);
    const signature = `${state.page}:${active}:${rows.length}:${state.queryError}:${state.loading}:${state.emptyMessage}`;
    if (state.signature === signature) return;
    state.signature = signature;
    const fragment = document.createDocumentFragment();
    rows.slice(state.page * pageSize, (state.page + 1) * pageSize).forEach((row, offset) => {
      const tr = document.createElement('tr'); tr.dataset.level = row.level;
      if (state.page * pageSize + offset === active) { tr.classList.add('lv-current'); tr.setAttribute('aria-current', 'true'); }
      const timeCell = document.createElement('td');
      if (row.t !== null) {
        const button = document.createElement('button'); button.type = 'button'; button.textContent = timeText(row.t);
        button.title = '跳转到此日志时间'; button.onclick = () => seekBagReplayProgress(row.t);
        const progress = document.createElement('div'); progress.append('播放进度 ', button); timeCell.append(progress);
      } else timeCell.textContent = '无播放进度';
      const rosTime = document.createElement('div'); rosTime.className = 'lv-ros-time';
      rosTime.textContent = Number.isFinite(row.epoch) ? `ROS ${row.epoch.toFixed(9)} s` : '无 ROS 时间';
      timeCell.prepend(rosTime);
      tr.append(timeCell);
      [row.level === 'RAW' ? '原文' : row.level, row.node || '—', row.message].forEach((value, index) => {
        const td = document.createElement('td');
        if (index > 0) td.setAttribute('data-i18n-ignore', '');
        markText(td, value, index > 0 ? state.regex : null);
        if (index === 2) td.title = row.raw;
        tr.append(td);
      }); fragment.append(tr);
    });
    $('rows').replaceChildren(fragment);
    $('empty').hidden = rows.length > 0;
    $('empty').textContent = state.queryError ? '正则表达式无效，请修改搜索条件' : state.rows.length ? '没有匹配日志，请调整筛选条件' : state.loading ? '随 bag 加载对应日志…' : state.emptyMessage;
    $('matched').textContent = `${rows.length.toLocaleString()} / ${state.rows.length.toLocaleString()} 行`;
    $('errors').textContent = `${rows.filter(row => ['ERROR', 'FATAL'].includes(row.level)).length} 条错误 / 致命`;
    $('prev').disabled = state.page === 0; $('next').disabled = state.page === lastPage;
    $('status').textContent = state.queryError ? '正则表达式无效，请修改搜索条件' : `第 ${state.page + 1} / ${lastPage + 1} 页 · 每页 ${pageSize} 行`;
    if ($('follow').checked) {
      const current = $('rows').querySelector('.lv-current');
      if (current) { const wrapper = current.closest('.lv-table-wrap'); wrapper.scrollTop += current.getBoundingClientRect().top - wrapper.getBoundingClientRect().top - wrapper.clientHeight / 2; }
    }
  }
  function reset() {
    document.getElementById('log-viewer').open = false;
    state.request++; state.rows = []; state.filtered = []; state.page = 0; state.signature = ''; state.time = 0; state.loading = true; state.emptyMessage = ''; state.queryError = false;
    $('note').textContent = '随 bag 加载对应日志…'; updateNodes(); render();
  }
  async function loadReplay(data, entries) {
    reset();
    const request = state.request;
    state.rows = (data.timeline?.logs || []).map((row, index) => ({ ...row, index: index + 1, raw: `[${row.level}] [${row.epoch}] [${row.node}]: ${row.message}` }));
    const issues = [], cache = new Map();
    let associatedFiles = 0, loadedFiles = 0, loadError = "";
    // Bag rosout uses receipt timestamps; old recordings use associated terminal logs.
    for (const segment of data.segments || []) {
      if (state.rows.some(row => row.segment_index === segment.index)) continue;
      const entry = entries.find(entry => entry.bag === segment.bag);
      const paths = [...new Set(entry?.txt || [])];
      associatedFiles += paths.length;
      for (const path of paths) {
        try {
          if (!cache.has(path)) cache.set(path, (async () => {
            const response = await fetch(`${API_BASE_URL}/api/log_bag/text?path=${encodeURIComponent(path)}`, { cache: 'no-store' });
            const result = await response.json();
            if (!response.ok) { const error = new Error(result.error || `HTTP ${response.status}`); error.status = response.status; throw error; }
            return result;
          })());
          const result = await cache.get(path);
          if (request !== state.request) return;
          loadedFiles++;
          if (result.truncated) issues.push('关联日志较大，仅加载末尾 2 MiB');
          const origin = Number(segment.source_start_time_ns) / 1e9;
          let previousTime = null;
          result.text.split(/\r?\n/).forEach((raw, index) => {
            if (!raw) return;
            const row = parseLine(raw, index);
            if (row.epoch !== null && Number.isFinite(row.epoch)) {
              const relative = row.epoch - origin;
              if (relative < -0.05 || relative > segment.duration + 0.05) { previousTime = undefined; return; }
              previousTime = segment.start + Math.max(0, Math.min(segment.duration, relative));
            }
            if (previousTime === undefined) return;
            state.rows.push({ ...row, t: previousTime, segment_index: segment.index, bag: segment.bag });
          });
        } catch (error) {
          if (request !== state.request) return;
          loadError = error.status === 404 ? '日志文件或后端日志接口不存在，请检查后端是否已更新' : '关联日志读取失败，bag 回放仍可继续';
          issues.push(loadError);
        }
      }
    }
    if (request !== state.request) return;
    state.rows.sort((a, b) => (a.t === null ? Infinity : a.t) - (b.t === null ? Infinity : b.t));
    if (state.rows.some(row => row.t === null)) issues.push('无时间信息的原文保留在列表末尾');
    state.loading = false;
    state.emptyMessage = loadError || (loadedFiles ? '关联日志存在，但当前 bag 时间范围内没有日志记录' : associatedFiles ? '关联日志尚未加载' : '此 bag 暂无关联日志');
    $('note').textContent = [...new Set(issues)].join(' · ') || (state.rows.length ? '日志随播放进度高亮，点击日志时间可跳转' : state.emptyMessage);
    updateNodes(); refilter();
  }
  window.openDeliveryLogViewer = { reset, loadReplay, sync(time) { state.time = time; render(); } };
  $('levels').replaceChildren(...levels.map(level => {
    const label = document.createElement('label'), input = document.createElement('input'); input.type = 'checkbox'; input.checked = true;
    input.onchange = () => { input.checked ? state.levels.add(level) : state.levels.delete(level); refilter(); };
    label.dataset.level = level; label.append(input, level === 'RAW' ? '原文' : level); return label;
  }));
  ['search', 'node', 'regex'].forEach(id => $(id).addEventListener(id === 'search' ? 'input' : 'change', refilter));
  $('follow').onchange = () => { state.signature = ''; render(); };
  $('prev').onclick = () => { $('follow').checked = false; state.page--; state.signature = ''; render(); };
  $('next').onclick = () => { $('follow').checked = false; state.page++; state.signature = ''; render(); };
  document.addEventListener('keydown', event => { if (event.key === '/' && !document.getElementById('bag-replay-dialog').hidden && !event.target.closest('input,textarea,select,[contenteditable]')) { event.preventDefault(); document.getElementById('log-viewer').open = true; $('search').focus(); } });
  document.getElementById('log-viewer').addEventListener('toggle', () => {
    if (document.getElementById('log-viewer').open) { state.signature = ''; render(); }
    window.dispatchEvent(new Event('resize'));
  });
  reset();
})();
