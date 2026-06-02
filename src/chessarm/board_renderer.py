import base64

import cairosvg
import chess
import chess.svg
import cv2
import numpy as np


def renderChessboardImage(fen: str, boardSize=720, flipped=False):
    board = chess.Board(fen)
    svg = chess.svg.board(board=board, size=boardSize, flipped=flipped, coordinates=False)
    pngBytes = cairosvg.svg2png(bytestring=svg.encode("utf-8"))
    imgBgr = cv2.imdecode(np.frombuffer(pngBytes, np.uint8), cv2.IMREAD_COLOR)
    if imgBgr is None:
        raise RuntimeError("Failed to decode board image. Check cairosvg install.")
    if imgBgr.shape[0] != boardSize or imgBgr.shape[1] != boardSize:
        imgBgr = cv2.resize(imgBgr, (boardSize, boardSize), interpolation=cv2.INTER_AREA)

    return imgBgr


def imageToBase64(image) -> str:
    success, buffer = cv2.imencode(".jpg", image)
    if not success:
        raise RuntimeError("cv2.imencode failed")
    return base64.b64encode(buffer.tobytes()).decode("utf-8")
