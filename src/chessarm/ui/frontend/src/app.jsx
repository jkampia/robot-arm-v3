import React, { useEffect, useMemo, useRef, useState } from "react";

function newReqId() {
  return Math.random().toString(16).slice(2) + Date.now().toString(16);
}

function formatFieldValue(value) {
  if (!Number.isFinite(value)) return "0";
  return String(Math.round(value * 10) / 10);
}

export default function App() {

  const [wsStatus, setWsStatus] = useState("disconnected");

  const wsRef = useRef(null);

  const [frame1, setFrame1] = useState(null);
  const [frame2, setFrame2] = useState(null);
  const [frame3, setFrame3] = useState(null);
  const [frame4, setFrame4] = useState(null);
  const [linkPlotXY, setLinkPlotXY] = useState(null);
  const [linkPlotXZ, setLinkPlotXZ] = useState(null);

  const [robotStatus, setRobotStatus] = useState("ready");

  const [jog0, setJog0] = useState("0");
  const [jog1, setJog1] = useState("0");
  const [jog2, setJog2] = useState("0");
  const [jog3, setJog3] = useState("0");
  const [jog4, setJog4] = useState("0");

  const [robotJoint0, setRobotJoint0] = useState("0");
  const [robotJoint1, setRobotJoint1] = useState("0");
  const [robotJoint2, setRobotJoint2] = useState("0");
  const [robotJoint3, setRobotJoint3] = useState("0");
  const [robotJoint4, setRobotJoint4] = useState("0");

  const [pose0, setPose0] = useState("0");
  const [pose1, setPose1] = useState("0");
  const [pose2, setPose2] = useState("0");
  const [pose3, setPose3] = useState("0");
  const [pose4, setPose4] = useState("0");

  const [activeTab, setActiveTab] = useState("controls");
  const [slidersEnabled, setSlidersEnabled] = useState(true);

  const colors = {
    bg: "#1f2328",
    panel: "#2b3036",
    panelSoft: "#252a30",
    border: "#3d444d",
    text: "#f0f3f6",
    muted: "#a9b1bb",
    accent: "#7dd3fc",
    button: "#343b44",
    buttonDisabled: "#282d33",
    input: "#161a1f",
  };

  const tabStyle = (selected) => ({
    padding: "10px 14px",
    border: "none",
    borderBottom: selected ? `3px solid ${colors.accent}` : "3px solid transparent",
    background: "transparent",
    color: selected ? colors.text : colors.muted,
    fontWeight: selected ? 700 : 400,
    cursor: "pointer",
  });

  const buttonStyle = (extra = {}) => ({
    background: colors.button,
    color: colors.text,
    border: `1px solid ${colors.border}`,
    borderRadius: 6,
    cursor: "pointer",
    ...extra,
  });

  const inputStyle = (extra = {}) => ({
    background: colors.input,
    color: colors.text,
    border: `1px solid ${colors.border}`,
    borderRadius: 6,
    ...extra,
  });

  const jointControls = [
    { label: "J0", value: robotJoint0 },
    { label: "J1", value: robotJoint1 },
    { label: "J2", value: robotJoint2 },
    { label: "J3", value: robotJoint3 },
    { label: "J4", value: robotJoint4 },
  ];

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

        // update image frames
        if (msg.type === "frames" && msg.displayTiles) 
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
          if (Array.isArray(msg.joint_angles_deg) && msg.joint_angles_deg.length >= 5) {
            const nextJoints = msg.joint_angles_deg.map((value) => formatFieldValue(Number(value)));
            setRobotJoint0(nextJoints[0]);
            setRobotJoint1(nextJoints[1]);
            setRobotJoint2(nextJoints[2]);
            setRobotJoint3(nextJoints[3]);
            setRobotJoint4(nextJoints[4]);
            setJog0(nextJoints[0]);
            setJog1(nextJoints[1]);
            setJog2(nextJoints[2]);
            setJog3(nextJoints[3]);
            setJog4(nextJoints[4]);
          }

          if (Array.isArray(msg.fk_pose_mm_deg) && msg.fk_pose_mm_deg.length >= 5) {
            setPose0(formatFieldValue(Number(msg.fk_pose_mm_deg[0])));
            setPose1(formatFieldValue(Number(msg.fk_pose_mm_deg[1])));
            setPose2(formatFieldValue(Number(msg.fk_pose_mm_deg[2])));
            setPose3(formatFieldValue(Number(msg.fk_pose_mm_deg[3])));
            setPose4(formatFieldValue(Number(msg.fk_pose_mm_deg[4])));
          }

          if (msg.link_plot_svgs) {
            setLinkPlotXY(msg.link_plot_svgs.xy ? `data:image/svg+xml;base64,${msg.link_plot_svgs.xy}` : null);
            setLinkPlotXZ(msg.link_plot_svgs.xz ? `data:image/svg+xml;base64,${msg.link_plot_svgs.xz}` : null);
          }
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

  function updateJointValue(index, value) {
    const nextValues = [robotJoint0, robotJoint1, robotJoint2, robotJoint3, robotJoint4];
    nextValues[index] = value;

    const setters = [setRobotJoint0, setRobotJoint1, setRobotJoint2, setRobotJoint3, setRobotJoint4];
    setters[index](value);

    sendCmd("set_joint_angles", {
      joint_angles: nextValues.map((jointValue) => Number(jointValue || 0)),
    });
  }

  return (

    <div style={{ fontFamily: "sans-serif", padding: 16, minHeight: "100vh", background: colors.bg, color: colors.text }}>
      <h2>Chessbot UI</h2>

      <div
        role="tablist"
        aria-label="Chessbot views"
        style={{
          display: "flex",
          gap: 8,
          marginBottom: 20,
          borderBottom: `1px solid ${colors.border}`,
        }}
      >
        <button
          role="tab"
          aria-selected={activeTab === "controls"}
          onClick={() => setActiveTab("controls")}
          style={tabStyle(activeTab === "controls")}
        >
          Controls
        </button>

        <button
          role="tab"
          aria-selected={activeTab === "image-pipeline"}
          onClick={() => setActiveTab("image-pipeline")}
          style={tabStyle(activeTab === "image-pipeline")}
        >
          Image pipeline
        </button>
      </div>

      {activeTab === "controls" && (
      <div style={{ display: "flex", flexDirection: "column", gap: 24, alignItems: "flex-start" }}>
        
        <div style={{ width: 320, background: colors.panel, border: `1px solid ${colors.border}`, borderRadius: 8, padding: 16 }}>

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
          </div>
        </div>

        <div style={{ display: "flex", gap: 16, alignItems: "stretch", flexWrap: "wrap", justifyContent: "flex-start", width: "100%" }}>

        <div 
          style={{ 
            background: colors.panel,
            border: `1px solid ${colors.border}`,
            borderRadius: 8,
            padding: 16,
            order: 1,
            width: 180,
          }}>
          
          <h3 style={{ textAlign: "center", marginTop: 0 }}>Jog θ (deg)</h3>

          <div style={{ display: "flex", flexDirection: "column", gap: 10, width: "100%" }}>
            
            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="J0">J0:</label>
              <input
                type="number"
                step="0.1"
                value={jog0}
                onChange={(e) => setJog0(e.target.value)}
                placeholder="J0 (deg)"
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>

            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="J1">J1:</label>
              <input
                type="number"
                step="0.1"
                value={jog1}
                onChange={(e) => setJog1(e.target.value)}
                placeholder="J1 (deg)"
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>

            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="J2">J2:</label>
              <input
                type="number"
                step="0.1"
                value={jog2}
                onChange={(e) => setJog2(e.target.value)}
                placeholder="J2 (deg)"
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>
              
            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="J3">J3:</label>
              <input
                type="number"
                step="0.1"
                value={jog3}
                onChange={(e) => setJog3(e.target.value)}
                placeholder="J3 (deg)"
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>

            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="J4">J4:</label>
              <input
                type="number"
                step="0.1"
                value={jog4}
                onChange={(e) => setJog4(e.target.value)}
                placeholder="J4 (deg)"
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
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
              style={buttonStyle({ padding: "10px 12px", marginTop: 8, width: "100%" })}
              disabled={robotStatus !== "ready"}
            >
              Jog
            </button>
          </div>
        </div>

        <div
          style={{
            background: colors.panel,
            border: `1px solid ${colors.border}`,
            borderRadius: 8,
            padding: 16,
            width: 320,
            order: 3,
            display: "flex",
            flexDirection: "column",
          }}
        >
          <h3 style={{ textAlign: "center", marginTop: 0 }}>Joint sliders (deg)</h3>

          <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
            {jointControls.map((joint, index) => (
              <div key={joint.label} style={{ display: "grid", gridTemplateColumns: "32px 1fr 42px", gap: 10, alignItems: "center", minHeight: 35 }}>
                <label htmlFor={`${joint.label}-slider`}>{joint.label}</label>
                <input
                  id={`${joint.label}-slider`}
                  type="range"
                  min="0"
                  max="360"
                  step="1"
                  value={Number(joint.value || 0)}
                  onChange={(e) => {
                    if (slidersEnabled) updateJointValue(index, e.target.value);
                  }}
                  style={{
                    width: "100%",
                    opacity: slidersEnabled ? 1 : 0.55,
                    cursor: slidersEnabled ? "pointer" : "not-allowed",
                  }}
                />
                <span style={{ color: colors.muted, textAlign: "right" }}>
                  {Number(joint.value || 0)}
                </span>
              </div>
            ))}
          </div>

          <button
            type="button"
            onClick={() => setSlidersEnabled((enabled) => !enabled)}
            style={{
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
              gap: 12,
              marginTop: 16,
              padding: 0,
              width: "100%",
              background: "transparent",
              color: colors.text,
              border: "none",
              cursor: "pointer",
            }}
            aria-pressed={slidersEnabled}
            aria-label="Toggle slider joint control"
          >
            <span style={{ fontSize: 13 }}>Sliders control joints</span>
            <span
              aria-hidden="true"
              style={{
                position: "relative",
                display: "inline-flex",
                alignItems: "center",
                width: 54,
                height: 26,
                padding: 3,
                borderRadius: 999,
                background: slidersEnabled ? "#22c55e" : "#47505a",
                transition: "background 120ms ease",
              }}
            >
              <span
                style={{
                  width: 20,
                  height: 20,
                  borderRadius: "50%",
                  background: "#f0f3f6",
                  transform: slidersEnabled ? "translateX(28px)" : "translateX(0)",
                  transition: "transform 120ms ease",
                }}
              />
            </span>
          </button>
        </div>

        <div 
          style={{ 
            background: colors.panel,
            border: `1px solid ${colors.border}`,
            borderRadius: 8,
            padding: 16,
            order: 2,
            width: 180,
          }}>
          
          <h3 style={{ textAlign: "center", marginTop: 0 }}>Jog pose (mm)</h3>

          <div style={{ display: "flex", flexDirection: "column", gap: 10, width: "100%" }}>
            
            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="X">X:</label>
              <input
                type="number"
                step="0.1"
                value={pose0}
                onChange={(e) => setPose0(e.target.value)}
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>

            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="Y">Y:</label>
              <input
                type="number"
                step="0.1"
                value={pose1}
                onChange={(e) => setPose1(e.target.value)}
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>

            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="Z">Z:</label>
              <input
                type="number"
                step="0.1"
                value={pose2}
                onChange={(e) => setPose2(e.target.value)}
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>
              
            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="R">R:</label>
              <input
                type="number"
                step="0.1"
                value={pose3}
                onChange={(e) => setPose3(e.target.value)}
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
              />
            </div>

            <div style={{ display: "grid", gridTemplateColumns: "34px 1fr", alignItems: "center", gap: 8 }}>
              <label htmlFor="P">P:</label>
              <input
                type="number"
                step="0.1"
                value={pose4}
                onChange={(e) => setPose4(e.target.value)}
                style={inputStyle({ padding: "8px 10px", width: "100%", boxSizing: "border-box" })}
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
              style={buttonStyle({ padding: "10px 12px", marginTop: 8, width: "100%" })}
              disabled={robotStatus !== "ready"}
            >
              Jog
            </button>
          </div>
        </div>

        </div>

        <div
          style={{
            display: "grid",
            gridTemplateColumns: "360px 640px",
            gap: 16,
            alignItems: "start",
          }}
        >
          <img
            src={linkPlotXY ?? ""}
            alt="Robot arm joints plotted on x y axes"
            style={{ width: "100%", height: "auto", border: `1px solid ${colors.border}`, borderRadius: 8, background: colors.panelSoft }}
          />
          <img
            src={linkPlotXZ ?? ""}
            alt="Robot arm joints plotted on x z axes"
            style={{ width: "100%", height: "auto", border: `1px solid ${colors.border}`, borderRadius: 8, background: colors.panelSoft }}
          />
        </div>
      </div>
      )}

      {activeTab === "image-pipeline" && (
        <div>
          <div style={{ marginBottom: 16 }}>
            <button
              onClick={() => sendCmd("update_all")}
              style={buttonStyle({ width: 200, height: 44, fontSize: 16 })}
              disabled={robotStatus !== "ready"}
            >
              update all
            </button>
          </div>

          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(2, minmax(260px, 1fr))",
              gap: 24,
              maxWidth: 1100,
            }}
          >
            <img src={frame1 ?? ""} alt="Pipeline frame 1" style={{ width: "100%", height: "auto", background: colors.panelSoft, border: `1px solid ${colors.border}`, borderRadius: 8 }} />
            <img src={frame2 ?? ""} alt="Pipeline frame 2" style={{ width: "100%", height: "auto", background: colors.panelSoft, border: `1px solid ${colors.border}`, borderRadius: 8 }} />
            <img src={frame3 ?? ""} alt="Pipeline frame 3" style={{ width: "100%", height: "auto", background: colors.panelSoft, border: `1px solid ${colors.border}`, borderRadius: 8 }} />
            <img src={frame4 ?? ""} alt="Pipeline frame 4" style={{ width: "100%", height: "auto", background: colors.panelSoft, border: `1px solid ${colors.border}`, borderRadius: 8 }} />
          </div>
        </div>
      )}
    </div>
  );
}
