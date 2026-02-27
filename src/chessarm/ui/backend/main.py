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


# robot
from kinematic import ARM_5DOF
robot = ARM_5DOF()

# display library
from display import ChessarmDisplay
chessarmDisplay = ChessarmDisplay()


clients: Set[WebSocket] = set()
lock = asyncio.Lock()





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




# --- WebSocket MUST be defined before StaticFiles mount at "/" ---
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):

    await ws.accept()
    clients.add(ws)

    # send initial frames
    await ws.send_json(
    {
        "type": "frames",
        "displayTiles": 
        {
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

                if cmd == "update_all":
                    
                    await ws.send_text(json.dumps(ack(req_id, True)))

                    robot.status = "busy"
                    await ws.send_text(json.dumps(robot.stateToDict()))

                    await asyncio.to_thread(chessarmDisplay.updateAllDisplayTiles)

                    await ws.send_json({
                        "type": "frames",
                        "displayTiles": {
                            str(i): tile.toBase64()
                            for i, tile in enumerate(chessarmDisplay.displayTiles)
                        }
                    })

                    robot.status = "ready"
                    await ws.send_text(json.dumps(robot.stateToDict()))


                # do not continue on commands that take time
                elif cmd == "jog_joints":

                    await ws.send_text(json.dumps(ack(req_id, True)))

                    robot.status = "busy"
                    await ws.send_text(json.dumps(robot.stateToDict()))

                    print(f"jogging joint angles: {args['joint_angles']}")
                    await asyncio.sleep(0.25) # pretend task takes time

                    robot.status = "ready"
                    await ws.send_text(json.dumps(robot.stateToDict()))


                elif cmd == "jog_pose":

                    await ws.send_text(json.dumps(ack(req_id, True)))

                    robot.status = "busy"
                    await ws.send_text(json.dumps(robot.stateToDict()))

                    print(f"jogging pose: {args['pose']}")
                    await asyncio.sleep(0.25) # pretend task takes time

                    robot.status = "ready"
                    await ws.send_text(json.dumps(robot.stateToDict()))
            
                else:

                    await ws.send_text(json.dumps(ack(req_id, False, f"Unknown cmd: {cmd}")))
    


    except WebSocketDisconnect:
        clients.discard(ws)

# --- Static frontend (React build) mount LAST ---
BASE_DIR = Path(__file__).resolve().parent
DIST_DIR = BASE_DIR / "dist"
app.mount("/", StaticFiles(directory=str(DIST_DIR), html=True), name="static")