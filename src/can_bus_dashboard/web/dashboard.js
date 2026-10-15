"use strict";
const el = id => document.getElementById(id);
let session = null;
let busy = false;
function bytes(value) {
  const text = value.trim();
  if (!text) return [];
  return text.split(/\s+/).map(word => {
    if (!/^[0-9a-fA-F]{2}$/.test(word)) throw new Error("Use two hex digits per byte, separated by spaces");
    return parseInt(word, 16);
  });
}
function number(value) {
  if (!/^(0x[0-9a-fA-F]+|[0-9]+)$/.test(value.trim())) throw new Error("Invalid integer");
  return Number(value);
}
function flags(frame) {
  return ["extended", "rtr", "fd", "brs", "error"].filter(key => frame[key]).map(key => key.toUpperCase()).join(" ") || "STD";
}
function hex(data) {return data.map(byte => byte.toString(16).padStart(2, "0")).join(" ");}
function row(values) {
  const tr = document.createElement("tr");
  for (const value of values) {
    const td = document.createElement("td"); td.textContent = String(value); tr.append(td);
  }
  return tr;
}
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {cache:"no-store"} :
    {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}
function controls() {
  el("debug").disabled = !el("confirm").checked;
  el("claim").disabled = Boolean(session) || busy;
  el("release").disabled = !session || busy;
  el("send").disabled = !session || busy;
}
async function action(fn) {
  busy = true; controls();
  try {el("result").textContent = JSON.stringify(await fn());}
  catch (error) {el("result").textContent = error.message;}
  finally {busy = false; controls();}
}
el("confirm").addEventListener("change", () => {
  if (!el("confirm").checked && session) {
    const previous = session; session = null;
    api("/api/release", {session:previous}).catch(error => {el("result").textContent = error.message;});
  }
  controls();
});
el("claim").addEventListener("click", () => action(async () => {
  const result = await api("/api/session", {
    confirm:el("confirm").checked, owner:el("owner").value, duration:Number(el("duration").value),
    readonly:el("readonly").checked,
    route:{can_id:number(el("canid").value),channel:number(el("channel").value),
      extended:el("extended").checked,prefix:bytes(el("prefix").value)},
    stop_frames:JSON.parse(el("stops").value)
  });
  session = result.session;
  return {...result,session:"browser session established"};
}));
el("release").addEventListener("click", () => action(async () => {
  const result = await api("/api/release", {session}); session = null; return result;
}));
el("send").addEventListener("click", () => action(() => api("/api/send", {
  session,confirm:el("confirm").checked,
  frame:{can_id:number(el("canid").value),channel:number(el("channel").value),
    extended:el("extended").checked,rtr:el("rtr").checked,fd:el("fd").checked,
    brs:el("brs").checked,data:bytes(el("data").value)}
})));
let nextHeartbeat = 0;
async function poll() {
  try {
    if (session && performance.now() >= nextHeartbeat) {
      nextHeartbeat = performance.now() + 250;
      try {await api("/api/heartbeat", {session});}
      catch (error) {session = null;el("result").textContent = error.message;controls();}
    }
    const state = await api("/api/state");
    el("connection").textContent = `${state.bus_namespace}: ${state.connected ? "connected" : "STALE / disconnected"}`;
    el("notice").textContent = state.notice;
    el("session").textContent = state.debug ? `${state.debug.owner}: ${state.debug.remaining.toFixed(1)}s remaining, ${state.debug.readonly ? "read-only" : "ACTIVE sends"}` : "No debug session";
    const status = {...(state.status || {}),capabilities:state.capabilities,
      rates:state.monitor.rates,observed_error_frames:state.monitor.error_frames,
      tx_observed_available:state.tx_observed_available,release_pending:state.release_pending};
    el("status").replaceChildren(...Object.entries(status).map(([key,value]) => {
      const div = document.createElement("div");
      div.textContent = `${key}: ${typeof value === "object" ? JSON.stringify(value) : value}`;return div;
    }));
    el("ids").replaceChildren(...state.monitor.ids.map(item => {
      const f = item.last;return row([f.direction,`0x${f.can_id.toString(16)}`,f.channel,flags(f),item.count,item.rate,hex(f.data)]);
    }));
    const query = new URLSearchParams({limit:"1000",direction:el("direction").value});
    if (el("filter").value.trim()) query.set("can_id",el("filter").value.trim());
    const frames = await api(`/api/frames?${query}`);
    el("frames").replaceChildren(...frames.frames.slice(-200).reverse().map(f =>
      row([f.sequence,f.direction,`0x${f.can_id.toString(16)}`,f.channel,flags(f),hex(f.data)])));
  } catch (error) {
    el("connection").textContent = error.message;
  } finally {
    setTimeout(poll,100);
  }
}
controls();poll();
