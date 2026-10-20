"use strict";

// Uses the real DOM and styles, but never connects to a CAN service.
async function runCanDashboardTests(html, source, css) {
  const iframe = document.createElement("iframe");
  iframe.srcdoc = html.replace(/<script\b[^>]*>[\s\S]*?<\/script>/gi, "").replace(/<link\b[^>]*>/gi, "");
  iframe.style.cssText = "width:1440px;height:1000px;border:0";
  const loaded = new Promise(resolve => iframe.addEventListener("load", resolve, {once: true}));
  document.body.append(iframe);
  await loaded;
  const win = iframe.contentWindow, doc = win.document;
  const style = doc.createElement("style");
  style.textContent = css;
  doc.head.append(style);
  Object.defineProperty(doc, "hidden", {configurable: true, value: false});
  const el = id => doc.getElementById(id);
  const errors = [], requests = [], intervals = [], downloads = [], blobs = [];
  let nextPoll, rejectClaim = false, rejectSend = false, rejectHeartbeat = false, failState = false;
  let holdClaim = false, finishClaim, holdFrames = false, finishFrames;
  let sessionNumber = 0;
  const frames = Array.from({length: 1000}, (_, index) => ({
    sequence: index + 1, direction: ["rx", "written", "submitted"][index % 3],
    can_id: index % 2 ? 0x601 : 0x600, channel: 0, extended: false,
    rtr: false, fd: false, brs: false, error: index === 999, data: [1, index % 256],
  }));
  const snapshot = {
    bus_namespace: "/test/can/main", connected: true, status_age: 0.02, release_pending: false,
    notice: "Monitor only. Debugging can activate or damage arbitrary devices.", debug: null, tx_observed_available: true,
    capabilities: {known: true, can_fd: true, channel: 0},
    status: {backend: "sim", ready: true, endpoint_ready: true, epoch: "18446744073709551614",
      port: "sim", can_bitrate: 1000000, rx_count: 12000, tx_count: 8000, rejected: 2,
      clients: 1, queue_depth: 0, all_quiet: true, observer_dropped: 0, observer_queue_depth: 0,
      maintenance_owner: "", error: "", client_errors: [], group_faults: [],
      client_details: [{name: "<img src=x onerror=alert(1)>", heartbeat_sequence: 42, rx_ready: true, fault: false}]},
    monitor: {rates: {rx: 1200, written: 800, submitted: 0}, error_frames: 1,
      ids: frames.slice(0, 6).map(last => ({last, count: 2000, rate: 500}))},
  };
  win.addEventListener("error", event => errors.push(event.message));
  win.addEventListener("unhandledrejection", event => errors.push(String(event.reason)));
  win.setTimeout = callback => { nextPoll = callback; return 1; };
  win.setInterval = (callback, delay) => { intervals.push({callback, delay}); return intervals.length; };
  win.URL.createObjectURL = blob => { blobs.push(blob); return "blob:can-test"; };
  win.URL.revokeObjectURL = () => {};
  win.HTMLAnchorElement.prototype.click = function () { downloads.push(this.download); };
  win.fetch = async (url, options = {}) => {
    const path = url.split("?")[0], body = options.body && JSON.parse(options.body);
    requests.push({path, body, options});
    let result = {}, status = 200;
    if (path === "/api/state") {
      if (failState) throw new Error("SIM network unavailable");
      result = JSON.parse(JSON.stringify(snapshot));
    } else if (path === "/api/frames") result = {frames, latest: 1000, oldest: 1};
    else if (path === "/api/session") {
      if (rejectClaim) { status = 409; result = {error: "Route overlaps existing client"}; }
      else {
        result = {session: `private-session-${++sessionNumber}`, route: body.route,
          readonly: body.readonly, duration: body.duration, epoch: snapshot.status.epoch};
        snapshot.debug = {owner: body.owner, route: body.route, readonly: body.readonly, remaining: body.duration};
      }
    } else if (path === "/api/send") {
      if (rejectSend) { status = 400; result = {error: "Unsupported frame length"}; }
      else result = {submitted: true, sequence: 8, written: false, note: "Not a broker acknowledgment"};
    } else if (path === "/api/release") {
      snapshot.debug = null;
      snapshot.release_pending = true;
      result = {ok: true, immediate: false, lease_expiry_seconds: 0.3};
    } else if (path === "/api/heartbeat") {
      if (rejectHeartbeat) { status = 409; result = {error: "Lease expired"}; }
      else result = {ok: true};
    } else throw new Error(`Unexpected mocked endpoint ${path}`);
    const response = {ok: status === 200, status, json: async () => result};
    if (path === "/api/session" && holdClaim) return await new Promise(resolve => { finishClaim = () => resolve(response); });
    if (path === "/api/frames" && holdFrames) return await new Promise(resolve => { finishFrames = () => resolve(response); });
    return response;
  };
  let checks = 0;
  const assert = (condition, message) => {
    if (!condition) throw new Error(message);
    checks += 1;
  };
  const settle = async () => {
    for (let i = 0; i < 4; i += 1) await new Promise(resolve => {
      const channel = new MessageChannel();
      channel.port1.onmessage = () => { channel.port1.close(); channel.port2.close(); resolve(); };
      channel.port2.postMessage(null);
    });
  };
  const poll = async () => {
    const callback = nextPoll; nextPoll = null;
    assert(Boolean(callback), "Monitoring schedules the next bounded poll");
    await callback(); await settle();
  };
  const singleRow = label => {
    const bar = el("status-dock").getBoundingClientRect();
    const identity = doc.querySelector(".identity").getBoundingClientRect();
    const readings = el("status-panel").getBoundingClientRect();
    const actions = doc.querySelector(".header-actions").getBoundingClientRect();
    const middle = bar.top + bar.height / 2;
    assert(identity.right <= readings.left && readings.right <= actions.left,
      `${label}: bus identity, counters and release occupy one horizontal row`);
    assert([identity, readings, actions].every(bounds => bounds.top <= middle && bounds.bottom >= middle),
      `${label}: the counters are not stacked beneath a separate title bar`);
  };
  const edit = (id, value) => {
    el(id).value = value;
    el(id).dispatchEvent(new win.Event(id === "direction" ? "change" : "input", {bubbles: true}));
  };
  const posted = path => requests.filter(request => request.path === "/api/" + path && request.body);
  const reclaim = async () => {
    snapshot.release_pending = false;
    await poll();
    el("claim").click();
    await settle();
  };
  try {
    win.eval(source);
    await settle();
    assert(requests.every(request => !request.body), "Opening CAN monitoring never registers or sends");
    assert(!el("confirm") && !el("debug").disabled && !el("claim").disabled,
      "A ready dashboard can submit a route directly without an authorization checkbox");
    assert(el("send").disabled, "Single-frame sends still require an actual claimed route");
    assert(el("rx-count").textContent === "12,000" && el("tx-count").textContent === "8,000", "Authoritative counters are prominent, not retained-history totals");
    assert(el("rx-rate").textContent.includes("1200.0"), "Rates above 1000 Hz are not capped");
    assert(el("id-count").textContent.includes("6"), "Every ID/direction combination is represented");
    assert(el("ids").children.length === 6, "ID table contains all observed keys");
    const dock = el("status-dock");
    const dockHeight = dock.getBoundingClientRect().height;
    let maxDockHeight = dockHeight;
    const readoutFont = parseFloat(win.getComputedStyle(el("rx-count")).fontSize);
    assert(dockHeight >= 80 && dockHeight <= 85, `CAN matches the 84px motor status bar (actual ${dockHeight})`);
    assert(readoutFont >= 26, "Desktop bus counters use large readable figures");
    singleRow("Desktop CAN header");
    assert(el("status-panel").tabIndex === 0, "The telemetry strip is keyboard focusable");
    assert(el("backend-label").textContent === "仿真" && el("backend-label").title.includes("SIMULATION"),
      "The compact environment badge retains the full backend identity");
    assert(el("tx-rate").title.includes("不等于 CAN ACK"), "Compact TX information never implies device execution");
    for (const id of ["bus-endpoint", "queue-status", "observer-status", "status-age"]) {
      assert(el("diagnostics-panel").contains(el(id)) && !dock.contains(el(id)),
        `${id} remains available in diagnostics without expanding the fixed bar`);
    }
    const primaryTop = el("overview-panel").getBoundingClientRect().top;
    assert(primaryTop < 500, `Primary traffic table starts within 500px of the viewport top (actual ${primaryTop})`);
    const ids = [...doc.querySelectorAll("[id]")].map(element => element.id);
    assert(new Set(ids).size === ids.length, "All HTML IDs are unique");
    for (const label of doc.querySelectorAll("label[for]")) assert(Boolean(el(label.htmlFor)), `Label ${label.htmlFor} targets an input`);
    el("tab-frames").click();
    iframe.style.height = "640px";
    await settle();
    win.scrollTo(0, 600);
    await settle();
    assert(win.scrollY > 0 && Math.abs(dock.getBoundingClientRect().top) < 1,
      "CAN status stays fixed during actual document scrolling");
    const initialRx = snapshot.status.rx_count, initialNamespace = snapshot.bus_namespace;
    snapshot.status.rx_count += 1;
    await poll();
    assert(el("rx-count").textContent === "12,001" && Math.abs(dock.getBoundingClientRect().top) < 1,
      "Bus counters continue updating while the frame log is scrolled");
    snapshot.status.rx_count = 123456789012345;
    snapshot.bus_namespace = "/very/long/robot/namespace/can/main";
    iframe.style.width = "390px";
    await poll();
    assert(dock.getBoundingClientRect().height <= 85
      && el("rx-count").title === snapshot.status.rx_count.toLocaleString("zh-CN")
      && el("bus-namespace").title === snapshot.bus_namespace,
      "Large counts and long namespaces retain full tooltips without making the bar taller");
    snapshot.status.rx_count = initialRx;
    snapshot.bus_namespace = initialNamespace;
    iframe.style.width = "1440px";
    iframe.style.height = "1000px";
    await poll();
    win.scrollTo(0, 0);
    assert(el("frames").children.length === 200, "Recent frame table is capped at 200 rows");
    assert(el("frames").firstChild.firstChild.textContent === "1000", "Newest frames appear first");
    assert(el("frames").querySelector(".frame-error") !== null, "Error frames are explicitly marked");
    edit("filter", "0x600");
    edit("direction", "rx");
    assert([...el("frames").children].every(row => row.children[2].textContent === "0x600" && row.children[1].textContent.includes("RX")), "ID and direction filters apply together");
    el("tab-overview").click();
    assert(el("ids").children.length === 1, "The same filters apply to the aggregate view");
    edit("filter", "1536");
    assert(el("ids").children.length === 1, "Decimal and hexadecimal IDs are equivalent");
    edit("filter", "0xzz");
    assert(!el("filter-error").hidden && el("filter").getAttribute("aria-invalid") === "true", "Invalid filters show a field error");
    await poll();
    assert(el("connection").textContent.includes("已连接"), "Invalid filters do not masquerade as disconnections");
    assert(el("export-frames").disabled, "Invalid filters cannot export stale results");
    edit("filter", "0x20000000");
    assert(el("filter-error").textContent.includes("1FFFFFFF"), "Extended CAN ID bound is enforced");
    el("reset-filter").click();
    assert(el("filter").value === "" && el("direction").value === "" && el("filter-error").hidden, "Clearing filters restores both fields");
    el("tab-frames").click();
    el("export-frames").click();
    const csv = await blobs.at(-1).text();
    assert(downloads.includes("can-frames.csv") && csv.trim().split("\n").length === 201, "CSV contains exactly the 200 visible frames and its header");
    assert(csv.split("\n")[1].startsWith("1000,"), "CSV order matches the visible table");
    el("pause-frames").click();
    const beforePause = el("frames").textContent;
    const frameGets = requests.filter(request => request.path === "/api/frames").length;
    snapshot.status.rx_count += 10;
    await poll();
    assert(el("pause-frames").getAttribute("aria-pressed") === "true" && el("frames").textContent === beforePause, "Pause freezes the frame list");
    assert(el("rx-count").textContent === "12,010", "Status counters remain live while paused");
    assert(requests.filter(request => request.path === "/api/frames").length === frameGets, "Paused lists do not waste frame requests");
    el("pause-frames").click();
    await poll();
    assert(el("pause-frames").getAttribute("aria-pressed") === "false", "Resume restores live observation");
    edit("filter", "0x7FF");
    assert(el("frames").textContent.includes("没有匹配") && el("export-frames").disabled, "Empty matches have an explicit empty state");
    el("reset-filter").click();
    el("tab-diagnostics").click();
    assert(el("traffic-tools").hidden, "Traffic controls do not clutter diagnostics");
    assert(el("status").textContent.includes("18446744073709551614"), "Exact uint64 epoch string is retained");
    assert(el("clients").textContent.includes("<img") && !el("clients").querySelector("img"), "Client names are rendered as text");
    el("export-state").click();
    const exported = JSON.parse(await blobs.at(-1).text());
    assert(exported.status.epoch === snapshot.status.epoch && exported.exported_at, "State export preserves exact epoch and export time");
    snapshot.status.observer_dropped = 12;
    await poll();
    assert(el("health-banner").textContent.includes("丢弃 12") && el("health-banner").classList.contains("warning"), "Observation gaps are prominent");
    snapshot.status.observer_dropped = 0;
    snapshot.status.error = "SIM transport fault";
    await poll();
    assert(el("health-banner").classList.contains("error") && el("health-banner").textContent.includes("SIM transport fault"), "Transport faults are visible");
    snapshot.status.error = "";
    snapshot.tx_observed_available = false;
    await poll();
    assert(el("health-banner").textContent.includes("尚无驱动写出观察源"), "Missing write observation is not reported as successful execution");
    snapshot.tx_observed_available = true;
    snapshot.status.endpoint_ready = false;
    await poll();
    assert(el("connection").textContent === "未就绪" && el("connection").classList.contains("error"),
      "An unready bus is visible in the pinned status rather than only in page content");
    el("tab-debug").click();
    assert(el("claim").disabled && el("debug-guidance").textContent.includes("未就绪"), "Unready transport cannot accept a route");
    snapshot.status.endpoint_ready = true;
    await poll();
    edit("stops", "{not JSON}");
    el("claim").click(); await settle();
    assert(!posted("session").length && el("result").textContent.includes("JSON 数组"), "Malformed stop JSON is rejected before HTTP");
    edit("stops", "{}");
    el("claim").click(); await settle();
    assert(!posted("session").length && el("result").textContent.includes("JSON 数组"), "Stop frames require an array");
    edit("stops", '[{"can_id":1536,"channel":0,"data":[0,0,0,0,0,0,0,0]}]');
    edit("canid", "-1");
    el("claim").click(); await settle();
    assert(!posted("session").length, "Invalid route ID never reaches the service");
    edit("canid", "0x600");
    rejectClaim = true;
    el("claim").click(); await settle();
    assert(el("result").textContent.includes("overlaps") && el("send").disabled, "Route conflicts are explicit and never grant send access");
    rejectClaim = false;
    edit("prefix", "01");
    el("claim").click(); await settle();
    assert(posted("session").at(-1).body.route.prefix[0] === 1 && posted("session").at(-1).body.confirm === true,
      "The explicit claim click retains the exact prefix and compatible HTTP contract");
    assert(el("claim-fields").disabled && !el("send").disabled && !el("release").disabled, "Claim locks route editing but exposes sending and release");
    assert(!posted("send").length, "Claim does not automatically send a frame");
    assert(el("claimed-route").textContent.includes("0x600"), "Claimed route is visible beside the send form");
    edit("canid", "0x601");
    edit("data", "01 02 aa ff");
    el("send").click(); await settle();
    const send = posted("send").at(-1).body;
    assert(send.frame.can_id === 0x600 && send.frame.data.join() === "1,2,170,255", "Sending uses the immutable claimed route and exact hex bytes");
    assert(posted("send").length === 1 && el("result").textContent.includes("未确认"), "One click sends exactly one frame without claiming execution");
    edit("data", "1 GG");
    el("send").click(); await settle();
    assert(posted("send").length === 1 && el("result").classList.contains("error"), "Invalid byte input is explicitly rejected locally");
    edit("data", "01 02 03 04");
    rejectSend = true;
    el("send").click(); await settle();
    assert(el("result").textContent.includes("Unsupported frame length") && !el("release").disabled, "Rejected send retains a usable release control");
    rejectSend = false;
    el("fd").click();
    assert(!el("brs").disabled && el("rtr").disabled, "FD exposes BRS and excludes RTR");
    el("brs").click();
    el("send").click(); await settle();
    assert(posted("send").at(-1).body.frame.fd && posted("send").at(-1).body.frame.brs, "FD/BRS flags reach the API");
    el("fd").click();
    assert(el("brs").disabled && !el("brs").checked, "Disabling FD clears its dependent BRS draft");
    el("rtr").click();
    assert(el("fd").disabled, "RTR excludes FD");
    el("send").click(); await settle();
    assert(posted("send").at(-1).body.frame.rtr, "RTR flag reaches the API");
    el("rtr").click();
    const heartbeat = intervals.find(interval => interval.delay === 250);
    assert(Boolean(heartbeat), "Lease heartbeat has its own independent 250 ms timer");
    holdFrames = true;
    const pendingPoll = nextPoll(); nextPoll = null;
    await settle();
    const heartbeats = posted("heartbeat").length;
    await heartbeat.callback();
    assert(posted("heartbeat").length === heartbeats + 1, "Slow frame requests do not block lease heartbeats");
    holdFrames = false; finishFrames(); await pendingPoll; await settle();
    el("tab-frames").click();
    el("pause-frames").click();
    await heartbeat.callback();
    assert(posted("heartbeat").length === heartbeats + 2, "Pausing lists does not stop the active lease heartbeat");
    el("pause-frames").click();
    el("tab-diagnostics").click();
    el("export-state").click();
    assert(!(await blobs.at(-1).text()).includes("private-session"), "Status exports never contain browser session credentials");
    el("release").click(); await settle();
    el("tab-debug").click();
    assert(el("send").disabled && el("release").disabled && el("result").textContent.includes("非即时"), "Release ends local sends and clearly explains broker expiry");
    await poll();
    assert(el("claim").disabled, "Release-pending state prevents immediate reclaim");
    edit("canid", "0x600");
    el("readonly").checked = true;
    edit("stops", "[]");
    await reclaim();
    assert(posted("session").at(-1).body.readonly && !el("send").disabled, "Explicit non-active claims retain the documented send capability");
    rejectHeartbeat = true;
    await heartbeat.callback(); await settle();
    assert(el("send").disabled && el("result").textContent.includes("心跳中断"), "Heartbeat failure releases the session without automatic restoration");
    rejectHeartbeat = false;
    await reclaim();
    el("release").click(); await settle();
    assert(el("send").disabled && posted("release").length >= 3, "Explicit release ends sending without a separate authorization state");
    snapshot.release_pending = false; await poll();
    holdClaim = true;
    el("claim").click(); await settle();
    assert(!el("release").disabled && el("release").textContent.includes("取消"), "Pending route requests can be cancelled directly");
    el("release").click(); await settle();
    assert(el("release").disabled && el("send").disabled, "Cancellation prevents sending before the delayed response arrives");
    holdClaim = false; finishClaim(); await settle();
    assert(el("send").disabled && el("result").textContent.includes("迟到"), "A late claim cannot restore a cancelled session");
    await reclaim();
    const beforeHidden = posted("heartbeat").length;
    Object.defineProperty(doc, "hidden", {configurable: true, value: true});
    doc.dispatchEvent(new win.Event("visibilitychange")); await settle();
    await heartbeat.callback();
    assert(el("send").disabled && posted("heartbeat").length === beforeHidden, "Hidden pages release sessions and stop renewals");
    Object.defineProperty(doc, "hidden", {configurable: true, value: false});
    doc.dispatchEvent(new win.Event("visibilitychange")); await settle();
    assert(el("send").disabled, "Returning to the page never resumes a released session");
    await reclaim();
    win.dispatchEvent(new win.Event("pagehide")); await settle();
    assert(el("send").disabled && posted("release").at(-1).options.keepalive, "Leaving the page uses a keepalive release");
    await reclaim();
    failState = true;
    await poll();
    assert(el("connection").textContent.includes("未连接") && el("send").disabled, "Network failure clearly marks stale data and stops sending");
    assert(el("result").textContent.includes("SIM network unavailable"), "Network error details remain visible");
    failState = false;
    snapshot.release_pending = false; await poll();
    assert(el("send").disabled, "Monitoring recovery does not reacquire a debug session");
    el("fd").checked = true; el("brs").checked = true; el("extended").checked = true;
    snapshot.status.backend = "serial"; snapshot.capabilities.can_fd = false;
    await poll();
    assert(el("fd").disabled && el("extended").disabled && el("rtr").disabled, "Legacy serial capabilities gate unsupported flags");
    assert(!el("fd").checked && !el("brs").checked && !el("extended").checked
      && el("result").textContent.includes("后端能力已改变"),
      "A backend change clears unsupported drafts with an explicit explanation");
    assert(el("frame-help").textContent.includes("4 / 8"), "Serial frame-length and padding constraints are visible");
    el("tab-overview").focus();
    el("tab-overview").dispatchEvent(new win.KeyboardEvent("keydown", {key: "End", bubbles: true}));
    assert(doc.activeElement === el("tab-debug") && !el("debug-panel").hidden, "End navigates and focuses the last tab");
    el("tab-debug").dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowRight", bubbles: true}));
    assert(doc.activeElement === el("tab-overview"), "Arrow navigation wraps to the first tab");
    for (const width of [1440, 1366, 1024, 900, 768, 640, 550, 390, 320]) {
      iframe.style.width = `${width}px`;
      await settle();
      singleRow(`${width}px CAN header`);
      const barHeight = dock.getBoundingClientRect().height;
      maxDockHeight = Math.max(maxDockHeight, barHeight);
      assert(barHeight >= 80 && barHeight <= 85, `The ${width}px CAN header remains 84px high (actual ${barHeight})`);
      const readings = el("status-panel");
      const area = readings.getBoundingClientRect();
      const identityLeft = doc.querySelector(".identity").getBoundingClientRect().left;
      const releaseRight = el("release").getBoundingClientRect().right;
      readings.scrollLeft = 0;
      if (readings.scrollWidth > readings.clientWidth + 1) {
        assert(win.getComputedStyle(readings).overflowX === "auto", "Only bus readouts scroll on narrow screens");
        assert(el("rx-count").getBoundingClientRect().left >= area.left
          && el("rx-count").getBoundingClientRect().right <= area.right + 1, "RX is fully reachable at the strip start");
        readings.scrollLeft = readings.scrollWidth;
        assert(readings.scrollLeft > 0 && el("clients-count").getBoundingClientRect().left >= area.left - 1
          && el("clients-count").getBoundingClientRect().right <= area.right + 1, "Client count is fully reachable at the strip end");
        assert(doc.querySelector(".identity").getBoundingClientRect().left === identityLeft
          && el("release").getBoundingClientRect().right === releaseRight, "Identity and release never scroll with the counters");
        readings.scrollLeft = 0;
      }
      for (const id of ["rx-count", "tx-count", "rejected-count", "clients-count", "connection", "backend-label", "release"]) {
        const bounds = el(id).getBoundingClientRect();
        assert(bounds.height > 0 && bounds.top >= 0 && bounds.bottom <= dock.getBoundingClientRect().bottom,
          `${id} remains in the fixed ${width}px row`);
      }
      for (const tab of doc.querySelectorAll("[data-panel]")) {
        tab.click();
        assert(doc.querySelectorAll(".view-panel:not([hidden])").length === 1, `Only one panel is visible at ${width}px`);
        assert(doc.documentElement.scrollWidth <= win.innerWidth + 1, `${tab.id} has no page-level overflow at ${width}px`);
        win.scrollTo(0, 600);
        await settle();
        assert(Math.abs(dock.getBoundingClientRect().top) < 1, `${tab.id} keeps bus status pinned`);
      }
      win.scrollTo(0, 0);
    }
    for (const [width, height] of [[1440, 640], [844, 390], [390, 400], [320, 300]]) {
      iframe.style.width = `${width}px`; iframe.style.height = `${height}px`;
      el("tab-frames").click();
      await settle();
      win.scrollTo(0, 600);
      await settle();
      const bar = dock.getBoundingClientRect();
      maxDockHeight = Math.max(maxDockHeight, bar.height);
      assert(win.scrollY > 0 && Math.abs(bar.top) < 1 && bar.height >= 80 && bar.height <= 85,
        `CAN remains one fixed 84px row at ${width}x${height}`);
      singleRow(`${width}x${height} CAN header`);
      assert(doc.documentElement.scrollWidth <= win.innerWidth + 1, "Short CAN layouts have no page-level horizontal overflow");
      win.scrollTo(0, 0);
    }
    assert(errors.length === 0, `No browser errors: ${errors.join("; ")}`);
    return {checks, requests: requests.length, primaryTop, dockHeight, maxDockHeight, readoutFont, errors};
  } finally {
    iframe.remove();
  }
}
