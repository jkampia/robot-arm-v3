import React, { useEffect, useMemo, useRef, useState } from "react";

function newReqId() {
  return Math.random().toString(16).slice(2) + Date.now().toString(16);
}

export default function App() {

  const [wsStatus, setWsStatus] = useState("disconnected");
  const [lastAck, setLastAck] = useState(null);

  const wsRef = useRef(null);

  const [frame1, setFrame1] = useState(null);
  const [frame2, setFrame2] = useState(null);
  const [frame3, setFrame3] = useState(null);
  const [frame4, setFrame4] = useState(null);

  const [robotStatus, setRobotStatus] = useState("ready");

  const [jog0, setJog0] = useState("0");
  const [jog1, setJog1] = useState("0");
  const [jog2, setJog2] = useState("0");
  const [jog3, setJog3] = useState("0");
  const [jog4, setJog4] = useState("0");

  const [pose0, setPose0] = useState("0");
  const [pose1, setPose1] = useState("0");
  const [pose2, setPose2] = useState("0");
  const [pose3, setPose3] = useState("0");
  const [pose4, setPose4] = useState("0");

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
      
      try 
      {

        const msg = JSON.parse(ev.data);

        // backend acknowledgement
        if (msg.type === "ack") setLastAck(msg);

        // update image frames
        else if (msg.type === "frames" && msg.displayTiles) 
        {
          const toUrl = (b64) => (b64 ? `data:image/jpeg;base64,${b64}` : "");
          setFrame1(toUrl(msg.displayTiles["0"]));
          setFrame2(toUrl(msg.displayTiles["1"]));
          setFrame3(toUrl(msg.displayTiles["2"]));
          setFrame4(toUrl(msg.displayTiles["3"]));
        }
        
        // update robot state
        else if (msg.type === "robot_state")
        {
          setRobotStatus(msg.status);
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
          <h2>Chessbot UI</h2>

          <div
            style={{
              display: "flex",
              flexDirection: "column",
              gap: 20,
              alignItems: "flex-start",
              marginBottom: 12,
            }}
          >
            <span style={{ fontSize: 18, display: "block", marginBottom: 0 }}>
              Interface: <b>{wsStatus}</b>
            </span>

            <span style={{ fontSize: 18, display: "block", marginBottom: 8 }}>
              Robot status: <b>{robotStatus}</b>
            </span>

            <button onClick={() => sendCmd("update_all")} 
              style={{width: 200, height: 60, fontSize: 16 }}
              disabled={robotStatus !== "ready"}
              >
              update all
            </button>

            <button
              onClick={() => sendCmd("move_named_pose", { name: "home" })}
              style={{ padding: "8px 12px" }}
              disabled={robotStatus !== "ready"}
            >
              Home
            </button>

            <button
              onClick={() => sendCmd("run_action", { name: "pick" })}
              style={{ padding: "8px 12px" }}
              disabled={robotStatus !== "ready"}
            >
              Run “pick”
            </button>
          </div>

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
            marginLeft: 24,
            flex: 1,
            maxWidth: 1000,
          }}
        >
          <img src={frame1 ?? ""} style={{ width: "100%", height: "auto" }} />
          <img src={frame2 ?? ""} style={{ width: "100%", height: "auto" }} />
          <img src={frame3 ?? ""} style={{ width: "100%", height: "auto" }} />
          <img src={frame4 ?? ""} style={{ width: "100%", height: "auto" }} />
        </div>

        {/* number inputs + 1 button (drop-in JSX) */}
        <div 
          style={{ 
            marginTop: 16,
            marginLeft: 24,
          }}>
          
          <h3>Jog θ (deg)</h3>

          <div style={{ display: "flex", flexDirection: "column", gap: 10, maxWidth: 120 }}>
            
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="J0">J0:</label>
              <input
                type="number"
                step="0.1"
                value={jog0}
                onChange={(e) => setJog0(e.target.value)}
                placeholder="J0 (deg)"
                style={{ padding: "8px 10px", width: 40 }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="J1">J1:</label>
              <input
                type="number"
                step="0.1"
                value={jog1}
                onChange={(e) => setJog1(e.target.value)}
                placeholder="J1 (deg)"
                style={{ padding: "8px 10px", width: 40 }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="J2">J2:</label>
              <input
                type="number"
                step="0.1"
                value={jog2}
                onChange={(e) => setJog2(e.target.value)}
                placeholder="J2 (deg)"
                style={{ padding: "8px 10px", width: 40 }}
              />
            </div>
              
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="J3">J3:</label>
              <input
                type="number"
                step="0.1"
                value={jog3}
                onChange={(e) => setJog3(e.target.value)}
                placeholder="J3 (deg)"
                style={{ padding: "8px 10px", width: 40 }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="J4">J4:</label>
              <input
                type="number"
                step="0.1"
                value={jog4}
                onChange={(e) => setJog4(e.target.value)}
                placeholder="J4 (deg)"
                style={{ padding: "8px 10px", width: 40 }}
              />
            </div>

            <button
              onClick={() =>
                sendCmd("jog_joints", {
                  joint_angles: [
                    Number(jog0 || 0),
                    Number(jog1 || 0),
                    Number(jog2 || 0),
                    Number(jog3 || 0),
                    Number(jog4 || 0),
                  ],
                })
              }
              style={{ padding: "10px 12px", marginTop: 8}}
              disabled={robotStatus !== "ready"}
            >
              Jog
            </button>
          </div>
        </div>

        {/* number inputs + 1 button (drop-in JSX) */}
        <div 
          style={{ 
            marginTop: 16,
            marginLeft: 24
          }}>
          
          <h3>Jog pose (mm)</h3>

          <div style={{ display: "flex", flexDirection: "column", gap: 10, maxWidth: 120 }}>
            
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="X">X:</label>
              <input
                type="number"
                step="0.1"
                value={pose0}
                onChange={(e) => setPose0(e.target.value)}
                style={{ padding: "8px 10px", width: 60 }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="Y">Y:</label>
              <input
                type="number"
                step="0.1"
                value={pose1}
                onChange={(e) => setPose1(e.target.value)}
                style={{ padding: "8px 10px", width: 60 }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="Z">Z:</label>
              <input
                type="number"
                step="0.1"
                value={pose2}
                onChange={(e) => setPose2(e.target.value)}
                style={{ padding: "8px 10px", width: 60 }}
              />
            </div>
              
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="R">R:</label>
              <input
                type="number"
                step="0.1"
                value={pose3}
                onChange={(e) => setPose3(e.target.value)}
                style={{ padding: "8px 10px", width: 60 }}
              />
            </div>

            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <label htmlFor="P">P:</label>
              <input
                type="number"
                step="0.1"
                value={pose4}
                onChange={(e) => setPose4(e.target.value)}
                style={{ padding: "8px 10px", width: 60 }}
              />
            </div>

            <button
              onClick={() =>
                sendCmd("jog_pose", {
                  pose: [
                    Number(pose0 || 0),
                    Number(pose1 || 0),
                    Number(pose2 || 0),
                    Number(pose3 || 0),
                    Number(pose4 || 0),
                  ],
                })
              }
              style={{ padding: "10px 12px", marginTop: 8}}
              disabled={robotStatus !== "ready"}
            >
              Jog
            </button>
          </div>
        </div>

      </div>
    </div>
  );
}