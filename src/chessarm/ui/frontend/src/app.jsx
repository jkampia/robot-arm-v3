import React, { useEffect, useMemo, useRef, useState } from "react";

function newReqId() {
  return Math.random().toString(16).slice(2) + Date.now().toString(16);
}

export default function App() {

  const [wsStatus, setWsStatus] = useState("disconnected");
  const [state, setState] = useState(null);
  const [lastAck, setLastAck] = useState(null);
  const wsRef = useRef(null);

  const [frame1, setFrame1] = useState(null);
  const [frame2, setFrame2] = useState(null);
  const [frame3, setFrame3] = useState(null);
  const [frame4, setFrame4] = useState(null);

  const wsUrl = useMemo(() => {
    // In dev, Vite proxy handles /ws; in prod we can use absolute
    // Using relative won't work for WS, so build it:
    const host = window.location.host;
    return `ws://${host}/ws`;
  }, []);

  useEffect(() => {

    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;

    ws.onopen = () => setWsStatus("connected");
    ws.onclose = () => setWsStatus("disconnected");
    ws.onerror = () => setWsStatus("error");

    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        
        if (msg.type === "state") setState(msg);
        
        if (msg.type === "ack") setLastAck(msg);

        if (msg.type === "frames" && msg.displayTiles) {
          const toUrl = (b64) => (b64 ? `data:image/jpeg;base64,${b64}` : "");
          setFrame1(toUrl(msg.displayTiles["0"]));
          setFrame2(toUrl(msg.displayTiles["1"]));
          setFrame3(toUrl(msg.displayTiles["2"]));
          setFrame4(toUrl(msg.displayTiles["3"]));
        }
      } 
      catch {
        // ignore parse errors
      }
    };

    return () => {
      ws.close();
    };
  }, [wsUrl]);

  function sendCmd(cmd, args = {}) {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;

    const req_id = newReqId();
    ws.send(JSON.stringify({ type: "cmd", req_id, cmd, args }));
  }

  return (

    <div style={{ fontFamily: "sans-serif", padding: 16 }}>
      {/* Row layout: left controls + right images */}
      <div style={{ display: "flex", gap: 24, alignItems: "flex-start" }}>
        
        {/* LEFT COLUMN */}
        <div style={{ width: 320 }}>
          <h2>Robot UI</h2>

          <div
            style={{
              display: "flex",
              flexDirection: "column",
              gap: 20,
              alignItems: "flex-start",
              marginBottom: 12,
            }}
          >
            <span>
              WS: <b>{wsStatus}</b>
            </span>

            <button onClick={() => sendCmd("updateall")} style={{ padding: "8px 12px" }}>
              update all
            </button>

            <button onClick={() => sendCmd("enable")} style={{ padding: "8px 12px" }}>
              Enable
            </button>

            <button
              onClick={() => sendCmd("move_named_pose", { name: "home" })}
              style={{ padding: "8px 12px" }}
              disabled={!state || state.busy}
            >
              Home
            </button>

            <button
              onClick={() => sendCmd("run_action", { name: "pick" })}
              style={{ padding: "8px 12px" }}
              disabled={!state || state.busy}
            >
              Run “pick”
            </button>
          </div>

          <h3>State</h3>
          {!state ? (
            <p>(waiting for state…)</p>
          ) : (
            <ul>
              <li><b>enabled:</b> {String(state.enabled)}</li>
              <li><b>estop:</b> {String(state.estop)}</li>
              <li><b>busy:</b> {String(state.busy)}</li>
              <li><b>mode:</b> {state.mode}</li>
              <li><b>speed:</b> {state.speed}</li>
              <li><b>last_error:</b> {state.last_error || "(none)"}</li>
            </ul>
          )}

          <h3>Last ACK</h3>
          {!lastAck ? (
            <p>(none)</p>
          ) : (
            <p>
              req_id={lastAck.req_id} ok={String(lastAck.ok)}{" "}
              {lastAck.reason ? `reason=${lastAck.reason}` : ""}
            </p>
          )}

          <hr />

          <button
            onClick={async () => {
              const r = await fetch("/api/ping");
              alert(JSON.stringify(await r.json()));
            }}
          >
            Test REST /api/ping
          </button>
        </div>

        {/* RIGHT SIDE: 2x2 IMAGES */}
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "1fr 1fr",
            gap: 48,
            marginTop: 48,
            marginLeft: 48,
            flex: 1,
            maxWidth: 1000,
          }}
        >
          <img src={frame1 ?? ""} style={{ width: "100%", height: "auto" }} />
          <img src={frame2 ?? ""} style={{ width: "100%", height: "auto" }} />
          <img src={frame3 ?? ""} style={{ width: "100%", height: "auto" }} />
          <img src={frame4 ?? ""} style={{ width: "100%", height: "auto" }} />
        </div>

      </div>
    </div>
  );
}