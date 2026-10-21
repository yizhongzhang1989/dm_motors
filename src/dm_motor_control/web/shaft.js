"use strict";
(() => {
  const el = id => document.getElementById(id), disc = el("shaft-disc");
  const source = "shaft_" + [...crypto.getRandomValues(new Uint32Array(4))].map(n => n.toString(16)).join("");
  let snapshot = null, actual = null, fresh = false, received = 0;
  let drag = null, timer = null, sending = false, generation = 0, sequence = 0, issue = "";
  let anchorBusy = false;
  const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
  function allowed() {
    return fresh && !document.hidden && performance.now() - received < 700 &&
      snapshot?.state?.simulated && snapshot.state.absolute_position_valid !== false &&
      Math.abs(snapshot.state.direction) === 1 && snapshot.simulation?.enabled &&
      snapshot.simulation.connected && snapshot.simulation.state?.virtual_bus_confirmed &&
      Number.isFinite(actual?.absolute_position);
  }
  function physicalPosition() {
    const model = snapshot.simulation.state;
    return actual.absolute_position + model.position - model.encoder_position;
  }
  function anchorView() {
    const model = snapshot?.simulation?.state;
    if (!model?.anchor_active || !Number.isFinite(actual?.position)) return null;
    const direction = snapshot.state.direction, physical = physicalPosition();
    const bearing = direction * model.anchor_bearing + actual.position - direction * physical;
    return {bearing, contact: bearing + direction * (physical - model.anchor_position),
      torque: direction * model.anchor_torque};
  }
  function paint() {
    const position = actual?.position;
    const ready = fresh && Number.isFinite(position);
    const enabled = Boolean(allowed());
    disc.classList.toggle("stale", !ready);
    disc.setAttribute("aria-disabled", String(!enabled));
    disc.setAttribute("tabindex", enabled ? "0" : "-1");
    disc.setAttribute("role", enabled ? "spinbutton" : "img");
    if (ready) {
      const degrees = position * 180 / Math.PI;
      el("shaft-rotor").setAttribute("transform", `rotate(${degrees % 360} 140 140)`);
      el("shaft-position").textContent = `${position.toFixed(4)} rad`;
      disc.setAttribute("aria-label", `电机轴实测 ${position.toFixed(4)} 弧度。${enabled ? "拖动或左右键施力；双击或回车设置锚点" : "只读反馈"}`);
    } else {
      el("shaft-position").textContent = "-- rad";
    }
    if (enabled) disc.setAttribute("aria-valuenow", String(drag?.offset || 0));
    else disc.removeAttribute("aria-valuenow");
    el("shaft-hand").setAttribute("display", drag?.active && ready ? "inline" : "none");
    let status = !ready ? "等待新鲜轴反馈" : enabled ? "拖动施力；双击添加锚点" :
      snapshot?.simulation?.enabled ? "等待虚拟设备确认" : "只读：随实测反馈旋转";
    const anchor = anchorView();
    el("shaft-anchor").setAttribute("display", anchor && ready ? "inline" : "none");
    if (anchor && ready) {
      const x = 140 + 112 * Math.sin(anchor.bearing), y = 140 - 112 * Math.cos(anchor.bearing);
      const cx = 140 + 100 * Math.sin(anchor.contact), cy = 140 - 100 * Math.cos(anchor.contact);
      el("shaft-anchor-point").setAttribute("cx", x); el("shaft-anchor-point").setAttribute("cy", y);
      el("shaft-anchor-contact").setAttribute("cx", cx); el("shaft-anchor-contact").setAttribute("cy", cy);
      for (const [key, value] of [["x1", x], ["y1", y], ["x2", cx], ["y2", cy]]) {
        el("shaft-anchor-spring").setAttribute(key, value);
      }
      status = `锚点 ${anchor.torque.toFixed(3)} Nm · 双击紫点取消`;
    }
    if (drag?.active && ready) {
      const contact = position + drag.anchor, goal = drag.bearing + drag.offset;
      const x = 140 + 100 * Math.sin(contact), y = 140 - 100 * Math.cos(contact);
      el("shaft-contact").setAttribute("cx", x); el("shaft-contact").setAttribute("cy", y);
      el("shaft-goal").setAttribute("cx", 140 + 100 * Math.sin(goal));
      el("shaft-goal").setAttribute("cy", 140 - 100 * Math.cos(goal));
      const torque = snapshot.simulation.state.transient_torque * drag.direction;
      const length = clamp(torque * 120, -60, 60), arrow = el("shaft-force");
      for (const [key, value] of [["x1", x], ["y1", y], ["x2", x + length * Math.cos(contact)],
        ["y2", y + length * Math.sin(contact)]]) arrow.setAttribute(key, value);
      arrow.setAttribute("visibility", Math.abs(torque) > 1e-6 ? "visible" : "hidden");
      status = `手力 ${torque.toFixed(3)} Nm${anchor ? ` / 锚点 ${anchor.torque.toFixed(3)} Nm` : ""} · 松手释放手力`;
    }
    if (anchorBusy) status = "锚点操作确认中，以反馈为准";
    el("shaft-status").textContent = issue || status;
    el("shaft-status").classList.toggle("error", Boolean(issue));
  }
  async function request(path, body, keepalive = false) {
    const response = await fetch("/api/" + path, {
      method: "POST", cache: "no-store", keepalive,
      headers: {"Content-Type": "application/json", "X-External-Source": source,
        "X-External-Sequence": String(++sequence)},
      body: JSON.stringify(body), signal: AbortSignal.timeout(2000),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  }
  function finish() {
    if (!drag) return;
    const previous = drag, version = ++generation;
    drag = null;
    clearInterval(timer); timer = null;
    disc.classList.remove("dragging");
    if (previous.pointer !== null && disc.hasPointerCapture(previous.pointer)) disc.releasePointerCapture(previous.pointer);
    if (previous.active) {
      request("external-torque", {torque: 0, duration: 0.1}, true).catch(error => {
        if (version === generation) {
          issue = `撤力未确认：${error.message}；拖拽外力最多 300 ms 到期。`;
          paint();
        }
      });
    }
    paint();
  }
  async function send() {
    if (!drag?.active) return;
    if (!allowed()) {finish(); return;}
    if (sending) return;
    const version = generation;
    sending = true;
    try {
      await request("shaft-drag", {
        position: drag.origin + drag.direction * drag.offset,
        simulation_time: snapshot.simulation.state.simulation_time,
      });
    } catch (error) {
      if (version === generation && drag) {
        issue = `拖拽已停止：${error.message}`;
        finish();
      }
    } finally {sending = false;}
  }
  function angle(event) {
    const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(disc.getScreenCTM().inverse());
    const x = point.x - 140, y = point.y - 140;
    return Math.hypot(x, y) < 12 ? null : Math.atan2(x, -y);
  }
  function activate() {
    if (!drag || drag.active) return;
    drag.active = true;
    timer = setInterval(send, 50);
    send();
  }
  async function toggleAnchor(event = null) {
    if (!allowed()) return;
    if (anchorBusy) {issue = "锚点操作尚未确认，请稍后再试。"; paint(); return;}
    const model = snapshot.simulation.state, existing = anchorView();
    if (!Number.isSafeInteger(model.anchor_revision) || model.anchor_revision < 0) {
      issue = "锚点反馈尚未就绪，请确认仿真节点版本。"; paint(); return;
    }
    if (existing && event) {
      const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(disc.getScreenCTM().inverse());
      if (Math.hypot(point.x - 140 - 112 * Math.sin(existing.bearing),
        point.y - 140 + 112 * Math.cos(existing.bearing)) > 18) {
        issue = "请双击现有紫色锚点取消，再创建新锚点。"; paint(); return;
      }
    }
    finish();
    anchorBusy = true; issue = ""; paint();
    const bearing = event ? angle(event) ?? actual.position : actual.position;
    try {
      await request("shaft-anchor", {
        active: !existing, bearing: existing ? model.anchor_bearing :
          physicalPosition() + snapshot.state.direction * (bearing - actual.position),
        revision: model.anchor_revision, simulation_time: model.simulation_time,
      });
    } catch (error) {
      issue = `锚点操作未确认：${error.message}；以反馈为准，不自动重试。`;
    } finally {anchorBusy = false; paint();}
  }
  function begin(bearing, pointer) {
    if (!allowed() || drag) return false;
    const model = snapshot.simulation.state;
    if (![model.position, model.encoder_position].every(Number.isFinite)) {
      issue = "无法确认物理轴坐标，拒绝施力。"; paint(); return false;
    }
    generation++;
    issue = "";
    drag = {pointer, active: false, bearing, angle: bearing, offset: 0, anchor: bearing - actual.position,
      origin: physicalPosition(),
      direction: snapshot.state.direction};
    disc.classList.add("dragging");
    paint();
    return true;
  }
  disc.addEventListener("pointerdown", event => {
    if (event.button !== 0 || !allowed()) return;
    const bearing = angle(event);
    if (bearing !== null && begin(bearing, event.pointerId)) {
      event.preventDefault(); disc.setPointerCapture(event.pointerId);
    }
  });
  disc.addEventListener("pointermove", event => {
    if (!drag || event.pointerId !== drag.pointer) return;
    const bearing = angle(event);
    if (bearing === null) return;
    event.preventDefault();
    const delta = Math.atan2(Math.sin(bearing - drag.angle), Math.cos(bearing - drag.angle));
    drag.offset += delta;
    drag.angle = bearing;
    if (Math.abs(drag.offset) > 0.02) activate();
    paint();
  });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) {
    disc.addEventListener(name, event => {if (drag && drag.pointer === event.pointerId) finish();});
  }
  disc.addEventListener("keydown", event => {
    if (event.key === "Enter" && !event.repeat && allowed()) {
      event.preventDefault(); toggleAnchor(); return;
    }
    if (!["ArrowLeft", "ArrowRight"].includes(event.key) || !allowed()) return;
    if (!drag) begin(actual.position, null);
    if (drag?.pointer !== null) return;
    event.preventDefault();
    drag.offset += event.key === "ArrowRight" ? 0.1 : -0.1;
    activate();
    paint();
  });
  disc.addEventListener("keyup", event => {
    if (["ArrowLeft", "ArrowRight"].includes(event.key) && drag?.pointer === null) finish();
  });
  disc.addEventListener("dblclick", event => {
    if (event.button === 0 && allowed()) {event.preventDefault(); toggleAnchor(event);}
  });
  el("stop").addEventListener("click", finish);
  document.addEventListener("keydown", event => {if (event.key === "Escape") finish();});
  document.addEventListener("visibilitychange", () => {if (document.hidden) finish();});
  for (const name of ["pagehide", "blur"]) window.addEventListener(name, finish);
  window.motorShaft = {
    update(data, connected) {
      snapshot = data; fresh = connected; received = performance.now();
      if (data?.state) actual = data.state;
      if (drag && !allowed()) finish();
      paint();
    },
    sample(value) {actual = value; paint();},
  };
  paint();
})();
