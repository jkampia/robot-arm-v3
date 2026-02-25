import asyncio
import json
from dataclasses import dataclass, asdict
from typing import Any, Dict, Set

import sys
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

import cv2
import base64

srcDir = Path(__file__).resolve().parents[2]
sys.path.append(str(srcDir))

app = FastAPI()


# --- Robot-ish state (replace with your real driver) ---
@dataclass
class RobotState:
    enabled: bool = False
    estop: bool = False
    busy: bool = False
    mode: str = "manual"
    speed: float = 0.2
    last_error: str = ""


state = RobotState()
clients: Set[WebSocket] = set()
lock = asyncio.Lock()



from display import ChessarmDisplay
chessarmDisplay = ChessarmDisplay()



@app.get("/api/ping")
async def ping():
    return JSONResponse({"ok": True, "msg": "pong"})



async def broadcast(msg: Dict[str, Any]) -> None:
    payload = json.dumps(msg)
    dead = []
    for ws in list(clients):
        try:
            await ws.send_text(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)



def ack(req_id: str, ok: bool, reason: str = "") -> Dict[str, Any]:
    out = {"type": "ack", "req_id": req_id, "ok": ok}
    if reason:
        out["reason"] = reason
    return out



async def do_work_simulated(cmd: str, args: Dict[str, Any]) -> None:
    # Replace with your robot SDK calls
    await asyncio.sleep(0.25)



# --- WebSocket MUST be defined before StaticFiles mount at "/" ---
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):

    await ws.accept()
    clients.add(ws)

    # send initial state
    await ws.send_text(json.dumps({"type": "state", **asdict(state)}))

    # send initial frames
    await ws.send_json({
                        "type": "frames",
                        "displayTiles": {
                            str(i): tile.toBase64()
                            for i, tile in enumerate(chessarmDisplay.displayTiles)
                        }
                    })

    try:
        
        while True:

            raw = await ws.receive_text()
            msg = json.loads(raw)

            if msg.get("type") != "cmd":
                continue

            req_id = str(msg.get("req_id", ""))
            cmd = str(msg.get("cmd", ""))
            args = msg.get("args") or {}

            # Gatekeeping + fast state changes
            async with lock:

                if state.estop:
                    await ws.send_text(json.dumps(ack(req_id, False, "E-STOP active")))
                    continue

                if cmd == "updateall":
                    
                    await ws.send_text(json.dumps(ack(req_id, True)))

                    chessarmDisplay.updateAllDisplayTiles()

                    await ws.send_json({
                        "type": "frames",
                        "displayTiles": {
                            str(i): tile.toBase64()
                            for i, tile in enumerate(chessarmDisplay.displayTiles)
                        }
                    })
                    continue

                if cmd == "stop":
                    # TODO: send stop to robot
                    state.busy = False
                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await broadcast({"type": "state", **asdict(state)})
                    continue

                if not state.enabled:
                    await ws.send_text(json.dumps(ack(req_id, False, "Robot not enabled")))
                    continue

                if state.busy:
                    await ws.send_text(json.dumps(ack(req_id, False, "Robot busy")))
                    continue

                # commands that take time
                if cmd in ("move_named_pose", "run_action"):
                    state.busy = True
                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await broadcast({"type": "state", **asdict(state)})
                else:
                    await ws.send_text(json.dumps(ack(req_id, False, f"Unknown cmd: {cmd}")))
                    continue

            # Execute outside lock
            try:
                await do_work_simulated(cmd, args)
            except Exception as e:
                async with lock:
                    state.last_error = str(e)
            finally:
                async with lock:
                    state.busy = False
                    await broadcast({"type": "state", **asdict(state)})

    except WebSocketDisconnect:
        clients.discard(ws)

# --- Static frontend (React build) mount LAST ---
BASE_DIR = Path(__file__).resolve().parent
DIST_DIR = BASE_DIR / "dist"
app.mount("/", StaticFiles(directory=str(DIST_DIR), html=True), name="static")