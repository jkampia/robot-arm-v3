import asyncio
import json
import math
import os
import re
import threading
import time
from dataclasses import dataclass, asdict
from typing import Any, Dict, Set

import sys
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

moduleDir = Path(__file__).resolve().parents[2]
srcDir = moduleDir.parent
sys.path.append(str(srcDir))
sys.path.append(str(moduleDir))

app = FastAPI()


SERIAL_UTILS_HEADER = srcDir / "teensy" / "serial_utils.hpp"


def parseCppEnum(headerPath: Path, enumName: str) -> Dict[str, int]:
    text = headerPath.read_text()
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//.*", "", text)

    match = re.search(
        rf"enum\s+class\s+{re.escape(enumName)}(?:\s*:\s*\w+)?\s*\{{(?P<body>.*?)\}};",
        text,
        flags=re.DOTALL,
    )
    if not match:
        raise ValueError(f"Could not find enum class {enumName} in {headerPath}")

    enumValues: Dict[str, int] = {}
    nextValue = 0
    for rawEntry in match.group("body").split(","):
        entry = rawEntry.strip()
        if not entry:
            continue

        if "=" in entry:
            name, valueText = [part.strip() for part in entry.split("=", 1)]
            value = int(valueText, 0)
        else:
            name = entry
            value = nextValue

        enumValues[name] = value
        nextValue = value + 1

    return enumValues


serialCommands = parseCppEnum(SERIAL_UTILS_HEADER, "serialCommand")


def buildTeensyCommand(commandName: str, values) -> list:
    if commandName not in serialCommands:
        raise ValueError(f"Unknown serial command: {commandName}")
    return [serialCommands[commandName], *values]


class SerialConnectionManager:

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 921600):
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.status = "stopped"
        self.error = ""
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    def start(self):
        if self._thread and self._thread.is_alive():
            return

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._connect,
            name="chessarm-serial",
            daemon=True,
        )
        self._thread.start()

    def close(self):
        self._stop.set()
        if self.connection is not None:
            self.connection.close()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def writeCommand(self, commandValues) -> None:
        payload = "<" + ":".join(str(value) for value in commandValues) + ">"
        with self._lock:
            if self.connection is None or not self.connection.is_open:
                raise RuntimeError("Serial connection is not open")
            self.connection.write(payload.encode("ascii"))

    def stateToDict(self):
        with self._lock:
            return {
                "port": self.port,
                "baudrate": self.baudrate,
                "status": self.status,
                "error": self.error,
            }

    def _setStatus(self, status, error=""):
        with self._lock:
            self.status = status
            self.error = error

    def _connect(self):
        self._setStatus("connecting")
        try:
            import serial

            self.connection = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1,
                write_timeout=0.1,
            )
            self._setStatus("connected")
            print(f"serial connected: {self.port} @ {self.baudrate}")

            while not self._stop.wait(0.25):
                if self.connection is None or not self.connection.is_open:
                    self._setStatus("stopped")
                    break
                self._setStatus("connected")

        except Exception as exc:
            self._setStatus("error", str(exc))
            print(f"serial connection failed: {self.port} @ {self.baudrate}: {exc}")

        finally:
            if self.connection is not None:
                try:
                    self.connection.close()
                finally:
                    self.connection = None
            if self.status != "error":
                self._setStatus("stopped")


def envInt(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except ValueError:
        print(f"invalid {name}; using {default}")
        return default


def envFloat(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        print(f"invalid {name}; using {default}")
        return default


serialConnection = SerialConnectionManager(
    port=os.getenv("CHESSARM_SERIAL_PORT", "/dev/ttyACM0"),
    baudrate=envInt("CHESSARM_SERIAL_BAUD", 921600),
)


# robot
from chessarm.kinematics import ARM_5DOF
robot = ARM_5DOF()
robotLock = threading.RLock()


class SensorPollingManager:
    
    def __init__(self, pollInterval: float = 0.1):
        self.pollInterval = pollInterval
        self.status = "stopped"
        self.error = ""
        self.anglesDeg = [None, None, None, None, None]
        self.steps = [None, None, None, None, None]
        self.readTimeSeconds = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._loop = None

    def start(self, loop: asyncio.AbstractEventLoop):
        if self._thread and self._thread.is_alive():
            return

        self._loop = loop
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._poll,
            name="chessarm-sensors",
            daemon=True,
        )
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def stateToDict(self):
        with self._lock:
            return {
                "status": self.status,
                "error": self.error,
                "angles_deg": self.anglesDeg,
                "steps": self.steps,
                "read_time_seconds": self.readTimeSeconds,
            }

    def _setState(
        self,
        status: str,
        error: str = "",
        anglesDeg=None,
        steps=None,
        readTimeSeconds=None,
    ):
        with self._lock:
            self.status = status
            self.error = error
            if anglesDeg is not None:
                self.anglesDeg = anglesDeg
            if steps is not None:
                self.steps = steps
            if readTimeSeconds is not None:
                self.readTimeSeconds = readTimeSeconds

    def _broadcastRobotState(self):
        if self._loop is None:
            return

        with robotLock:
            message = robot.stateToDict()

        asyncio.run_coroutine_threadsafe(broadcast(message), self._loop)

    def _poll(self):
        self._setState("connecting")
        try:
            from chessarm.sensors import AS5600, TCA9548A, read_encoder_angles_on_bus
            from chessarm.sensors import DEFAULT_ENCODER_CHANNELS
            from chessarm.sensors import SMBus

            with SMBus(1) as bus:
                mux = TCA9548A(bus)
                encoder = AS5600(bus)
                self._setState("connected")

                while not self._stop.is_set():
                    loopStart = time.perf_counter()
                    readings = read_encoder_angles_on_bus(
                        mux,
                        encoder,
                        DEFAULT_ENCODER_CHANNELS,
                        mux_settle_seconds=0.001,
                        retries=0,
                    )
                    anglesDeg = readings.angles_degrees
                    currentAnglesDeg = None

                    with robotLock:
                        robot.setRawEncoderStepCounts(readings.step_counts)
                        jointAnglesDeg = robot.encoderAnglesToJointAnglesDeg(anglesDeg)
                        currentAnglesDeg = [
                            math.degrees(angle)
                            for angle in robot.jointAngles
                        ]
                        nextAnglesDeg = [
                            current if angle is None else float(angle)
                            for current, angle in zip(currentAnglesDeg, jointAnglesDeg)
                        ]
                        if nextAnglesDeg != currentAnglesDeg:
                            robot.setJointAnglesDeg(nextAnglesDeg)

                    self._setState(
                        "connected",
                        "",
                        anglesDeg=anglesDeg,
                        steps=readings.step_counts,
                        readTimeSeconds=readings.read_time_seconds,
                    )
                    self._broadcastRobotState()
                    elapsed = time.perf_counter() - loopStart
                    self._stop.wait(max(0.0, self.pollInterval - elapsed))

        except Exception as exc:
            self._setState("error", str(exc))
            print(f"sensor polling failed: {exc}")

        finally:
            if self.status != "error":
                self._setState("stopped")


sensorPolling = SensorPollingManager(
    pollInterval=envFloat("CHESSARM_SENSOR_POLL_INTERVAL", 1.0 / 60.0),
)

# display library
from display import ChessarmDisplay
from chessarm.board_renderer import imageToBase64, renderChessboardImage
from chessarm.chessboard import ChessBoard
from chessarm.game_logger import GameLogger
chessarmDisplay = ChessarmDisplay()
chessBoard = ChessBoard(playing_as="white", use_stockfish=False)
chessBoardLock = threading.RLock()
LOGS_DIR = moduleDir / "logs"


clients: Set[WebSocket] = set()
lock = asyncio.Lock()





@app.get("/api/ping")
async def ping():
    return JSONResponse({"ok": True, "msg": "pong"})


@app.get("/api/serial")
async def serialStatus():
    return JSONResponse(serialConnection.stateToDict())


@app.get("/api/serial_commands")
async def serialCommandStatus():
    return JSONResponse({
        "header": str(SERIAL_UTILS_HEADER),
        "commands": serialCommands,
    })


@app.get("/api/sensors")
async def sensorStatus():
    return JSONResponse(sensorPolling.stateToDict())


def serialStateMessage() -> Dict[str, Any]:
    return {
        "type": "serial_state",
        **serialConnection.stateToDict(),
    }


def serialCommandsMessage() -> Dict[str, Any]:
    return {
        "type": "serial_commands",
        "header": str(SERIAL_UTILS_HEADER),
        "commands": serialCommands,
    }


async def broadcastSerialStatusLoop() -> None:
    try:
        while True:
            await broadcast(serialStateMessage())
            await asyncio.sleep(2.0)
    except asyncio.CancelledError:
        pass


def robotStateMessage() -> Dict[str, Any]:
    with robotLock:
        return robot.stateToDict()


def boardStateMessage() -> Dict[str, Any]:
    with chessBoardLock:
        fen = chessBoard.toFEN(chessBoard.current_state)
        playerColor = chessBoard.playing_as
        canUndo = len(chessBoard.move_history) > 0
        canEnterMove = chessBoard.awaiting_stockfish_response
        stockfishElo = chessBoard.stockfish_elo

    boardImage = renderChessboardImage(fen)

    return {
        "type": "board_state",
        "fen": fen,
        "player_color": playerColor,
        "can_undo": canUndo,
        "can_enter_move": canEnterMove,
        "stockfish_elo": stockfishElo,
        "image": imageToBase64(boardImage),
    }


def currentBoardFen() -> str:
    with chessBoardLock:
        return chessBoard.toFEN(chessBoard.current_state)


def createGameLoggerWithInitialState() -> GameLogger:
    gameLogger = GameLogger(LOGS_DIR)
    gameLogger.logBoardState(currentBoardFen())
    return gameLogger


def resetGameState() -> GameLogger:
    global chessBoard

    with chessBoardLock:
        stockfishElo = chessBoard.stockfish_elo
        chessBoard = ChessBoard(playing_as="white", use_stockfish=False, stockfish_elo=stockfishElo)

    return createGameLoggerWithInitialState()


@app.on_event("startup")
async def startBackgroundWorkers():
    app.state.gameLogger = createGameLoggerWithInitialState()
    serialConnection.start()
    sensorPolling.start(asyncio.get_running_loop())
    app.state.serialStatusTask = asyncio.create_task(broadcastSerialStatusLoop())


@app.on_event("shutdown")
async def stopBackgroundWorkers():
    task = getattr(app.state, "serialStatusTask", None)
    if task is not None:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    sensorPolling.close()
    serialConnection.close()



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
    await ws.send_text(json.dumps(robotStateMessage()))
    await ws.send_text(json.dumps(boardStateMessage()))
    await ws.send_text(json.dumps(serialStateMessage()))
    await ws.send_text(json.dumps(serialCommandsMessage()))

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

                    with robotLock:
                        robot.setStatus("busy")
                    await ws.send_text(json.dumps(robotStateMessage()))

                    await asyncio.to_thread(chessarmDisplay.updateAllDisplayTiles)

                    await ws.send_json({
                        "type": "frames",
                        "displayTiles": {
                            str(i): tile.toBase64()
                            for i, tile in enumerate(chessarmDisplay.displayTiles)
                        }
                    })

                    with robotLock:
                        robot.setStatus("ready")
                    await ws.send_text(json.dumps(robotStateMessage()))


                elif cmd == "update_board":

                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await ws.send_text(json.dumps(boardStateMessage()))

                elif cmd == "new_game":

                    app.state.gameLogger = resetGameState()
                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await ws.send_text(json.dumps(boardStateMessage()))

                elif cmd == "update_stockfish_elo":

                    try:
                        stockfishElo = int(args.get("elo"))
                        if stockfishElo < 1320 or stockfishElo > 3190:
                            raise ValueError("Stockfish ELO must be between 1320 and 3190.")

                        with chessBoardLock:
                            chessBoard.setStockfishElo(stockfishElo)

                    except Exception as exc:
                        await ws.send_text(json.dumps(ack(req_id, False, str(exc))))
                        continue

                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await ws.send_text(json.dumps(boardStateMessage()))

                elif cmd == "move_piece":

                    fromSquare = str(args.get("from", ""))
                    toSquare = str(args.get("to", ""))
                    with chessBoardLock:
                        ok, reason, move = chessBoard.movePieceIfLegal(fromSquare, toSquare)
                        fen = chessBoard.toFEN(chessBoard.current_state) if ok else ""

                    await ws.send_text(json.dumps(ack(req_id, ok, reason)))
                    if ok:
                        app.state.gameLogger.logMove(move, fen)
                        await ws.send_text(json.dumps(boardStateMessage()))

                elif cmd == "enter_move":

                    with chessBoardLock:
                        if not chessBoard.awaiting_stockfish_response:
                            ok = False
                            reason = "No entered move is waiting for a Stockfish response."
                            stockfishMove = ""
                            stockfishFen = ""
                        else:
                            ok, reason, stockfishMove = chessBoard.makeStockfishMove()
                            stockfishFen = chessBoard.toFEN(chessBoard.current_state) if ok else ""

                    await ws.send_text(json.dumps(ack(req_id, ok, reason)))
                    if ok:
                        if stockfishMove:
                            app.state.gameLogger.logMove(stockfishMove, stockfishFen)
                        await ws.send_text(json.dumps(boardStateMessage()))

                elif cmd == "undo_move":

                    with chessBoardLock:
                        ok, reason, move = chessBoard.undoLastMove()
                        fen = chessBoard.toFEN(chessBoard.current_state) if ok else ""

                    await ws.send_text(json.dumps(ack(req_id, ok, reason)))
                    if ok:
                        app.state.gameLogger.logUndoMove(move, fen)
                        await ws.send_text(json.dumps(boardStateMessage()))


                # do not continue on commands that take time
                elif cmd == "set_joint_angles":

                    jointAnglesDeg = [float(value) for value in args["joint_angles"]]
                    try:
                        with robotLock:
                            robot.setJointAnglesDeg(jointAnglesDeg)
                    except Exception as exc:
                        await ws.send_text(json.dumps(ack(req_id, False, f"Invalid joint angles: {exc}")))
                        continue

                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await ws.send_text(json.dumps(robotStateMessage()))


                elif cmd == "jog_joints":

                    jointAnglesDeg = [float(value) for value in args["joint_angles"]]
                    
                    try:
                        with robotLock:
                            robot.setJoggedJointAnglesDeg(jointAnglesDeg)
                        jointAnglesRad = [math.radians(angle) for angle in jointAnglesDeg]
                        command = buildTeensyCommand(
                            "TGT_ANGLES_JOINT_SPACE_ACCEL",
                            [f"{angle:.6f}" for angle in jointAnglesRad],
                        )
                        serialConnection.writeCommand(command)
                        print(f"Wrote command: {command} to teensy")

                    except Exception as exc:
                        await ws.send_text(json.dumps(ack(req_id, False, f"Failed to jog joints: {exc}")))
                        continue

                    await ws.send_text(json.dumps(ack(req_id, True)))

                    with robotLock:
                        robot.setStatus("busy")
                    await ws.send_text(json.dumps(robotStateMessage()))

                    with robotLock:
                        robot.setStatus("ready")
                    await ws.send_text(json.dumps(robotStateMessage()))


                elif cmd == "jog_pose":

                    pose = [float(value) for value in args["pose"]]
                    try:
                        with robotLock:
                            robot.setJoggedJointAnglesRad(robot.solveIK(pose))
                    except Exception as exc:
                        await ws.send_text(json.dumps(ack(req_id, False, f"Invalid pose: {exc}")))
                        continue

                    await ws.send_text(json.dumps(ack(req_id, True)))
                    await ws.send_text(json.dumps(robotStateMessage()))
            
                else:

                    await ws.send_text(json.dumps(ack(req_id, False, f"Unknown cmd: {cmd}")))
    


    except WebSocketDisconnect:
        clients.discard(ws)

# --- Static frontend (React build) mount LAST ---
BASE_DIR = Path(__file__).resolve().parent
DIST_DIR = BASE_DIR.parent / "frontend" / "dist"
if DIST_DIR.exists():
    app.mount("/", StaticFiles(directory=str(DIST_DIR), html=True), name="static")
