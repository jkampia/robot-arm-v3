import chess
import chess.svg
import cairosvg
import cv2
import numpy as np

from chessarm_colors import COLOR_RGB
from frame_server.request_frame import FrameRequester
from vision import ImageProcessor, CalibrationHelper


class ChessarmDisplay:

    def __init__(self):

        self.windowName = "debug display"
        
        cv2.namedWindow(self.windowName, cv2.WINDOW_NORMAL)  # resizable
        
        # Optional: set initial window size (grid is 1280x960)
        cv2.resizeWindow(self.windowName, 1280, 960)

        self.frameCount = 0

        self.displayTiles = [None, None, None, None]
        self.displayTitles = [None, None, None, None]

        self.frameRequester = FrameRequester()
        self.imageProcessor = ImageProcessor()
        self.calibrationHelper = CalibrationHelper()



    def frameEmptyChessboard(self, boardSize=480):

        board = chess.Board(None)  # None = empty board (no pieces)
        svg = chess.svg.board(board=board, size=boardSize)  # size in pixels
        png_bytes = cairosvg.svg2png(bytestring=svg.encode("utf-8"))
        img_array = np.frombuffer(png_bytes, dtype=np.uint8)
        img_bgr = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        canvas = np.zeros((boardSize, 640, 3), dtype=np.uint8)
        x_offset = (640 - boardSize) // 2 # center board square inside (preserve aspect ratio)
        canvas[:, x_offset:x_offset + boardSize] = img_bgr
        return canvas


    
    def frameBlobChessboard(
    self,
    fen: str,
    size: int = 480,
    flipped: bool = False,
    circleRadiusScale: float = 0.33,
    whiteFillBgr=(255, 255, 255),
    blackFillBgr=(0, 0, 0),
    outlineBgr=(0, 0, 0),
    outlineThickness: int = 2,
    ):
        # --- Parse FEN, but use ONLY the placement field (before first space) ---
        placement = fen.split()[0]

        # --- Render EMPTY board background ---
        emptyBoard = chess.Board(None)  # empty
        svg = chess.svg.board(board=emptyBoard, size=size, flipped=flipped, coordinates=False)
        pngBytes = cairosvg.svg2png(bytestring=svg.encode("utf-8"))
        imgBgr = cv2.imdecode(np.frombuffer(pngBytes, np.uint8), cv2.IMREAD_COLOR)
        if imgBgr is None:
            raise RuntimeError("Failed to decode board image. Check cairosvg install.")
        if imgBgr.shape[0] != size or imgBgr.shape[1] != size:
            imgBgr = cv2.resize(imgBgr, (size, size), interpolation=cv2.INTER_AREA)

        squareSize = size // 8
        radius = max(1, int(circleRadiusScale * squareSize))

        def squareCenter(fileIdx: int, rankIdx: int):
            # fileIdx: 0..7 for a..h
            # rankIdx: 0..7 for 8..1 (rankIdx 0 is rank 8)
            if not flipped:
                x = fileIdx
                y = rankIdx
            else:
                x = 7 - fileIdx
                y = 7 - rankIdx

            cx = x * squareSize + squareSize // 2
            cy = y * squareSize + squareSize // 2
            return cx, cy

        # --- Walk the placement ranks (8 to 1) ---
        ranks = placement.split("/")
        if len(ranks) != 8:
            raise ValueError("Invalid FEN placement field (expected 8 ranks).")

        for rankIdx, rankStr in enumerate(ranks):  # rankIdx 0 => rank 8 (top)
            fileIdx = 0
            for ch in rankStr:
                if ch.isdigit():
                    fileIdx += int(ch)
                    continue

                if fileIdx > 7:
                    break

                # ch is a piece letter: uppercase = white, lowercase = black
                cx, cy = squareCenter(fileIdx, rankIdx)

                if ch.isupper():
                    cv2.circle(imgBgr, (cx, cy), radius, whiteFillBgr, -1, lineType=cv2.LINE_AA)
                    if outlineThickness > 0:
                        cv2.circle(imgBgr, (cx, cy), radius, outlineBgr, outlineThickness, lineType=cv2.LINE_AA)
                else:
                    cv2.circle(imgBgr, (cx, cy), radius, blackFillBgr, -1, lineType=cv2.LINE_AA)
                    if outlineThickness > 0:
                        # outline black circles in a light outline so they show on dark squares
                        cv2.circle(imgBgr, (cx, cy), radius, (255, 255, 255), outlineThickness, lineType=cv2.LINE_AA)

                fileIdx += 1

        # embed in tile
        canvas = np.zeros((size, 640, 3), dtype=np.uint8)
        x_offset = (640 - size) // 2 # center board square inside (preserve aspect ratio)
        canvas[:, x_offset:x_offset + size] = imgBgr

        return canvas


    

    def embedInTile(self, frame, size=480):

        canvas = np.zeros((size, 640, 3), dtype=np.uint8)
        x_offset = (640 - size) // 2 # center board square inside (preserve aspect ratio)
        canvas[:, x_offset:x_offset + size] = frame

        return canvas

    


    def createDisplayOutput(
        self,
        gap=60,
        margin=60,
        bg="cream",
        titles=("raw frame capture", "detected colors", "template", "board state"),
        titleYOffset=-40,
        titleXPadding=0,
    ):
        tileH, tileW = 480, 640

        def rgbToBgr(rgbTuple):
            return rgbTuple[::-1]

        tileBgr = rgbToBgr(COLOR_RGB["green"])
        bgBgr = rgbToBgr(COLOR_RGB[bg])

        def blank_tile():
            return np.full((tileH, tileW, 3), tileBgr, dtype=np.uint8)

        def normalize_tile(tile):
            # None -> blank
            if tile is None:
                return blank_tile()

            tile = np.asarray(tile)

            # empty or 1D -> blank (this is the exact error you hit)
            if tile.size == 0 or tile.ndim == 1:
                return blank_tile()

            # grayscale -> BGR
            if tile.ndim == 2:
                tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)

            # BGRA -> BGR
            if tile.ndim == 3 and tile.shape[2] == 4:
                tile = cv2.cvtColor(tile, cv2.COLOR_BGRA2BGR)

            # ensure uint8
            if tile.dtype != np.uint8:
                tile = np.clip(tile, 0, 255).astype(np.uint8)

            # resize to tile size if needed
            if tile.shape[0] != tileH or tile.shape[1] != tileW:
                tile = cv2.resize(tile, (tileW, tileH), interpolation=cv2.INTER_NEAREST)

            # final sanity: must be HxWx3
            if tile.ndim != 3 or tile.shape[2] != 3:
                return blank_tile()

            return tile

        # IMPORTANT: write back into the list
        for i in range(len(self.displayTiles)):
            self.displayTiles[i] = normalize_tile(self.displayTiles[i])

        # build grid with gaps
        verticalGap = np.full((tileH, gap, 3), bgBgr, dtype=np.uint8)
        horizontalGap = np.full((gap, 2 * tileW + gap, 3), bgBgr, dtype=np.uint8)

        topRow = np.hstack((self.displayTiles[0], verticalGap, self.displayTiles[1]))
        bottomRow = np.hstack((self.displayTiles[2], verticalGap, self.displayTiles[3]))
        grid = np.vstack((topRow, horizontalGap, bottomRow))

        # add outer margin
        gridH, gridW = grid.shape[:2]
        if margin > 0:
            canvas = np.full((gridH + 2 * margin, gridW + 2 * margin, 3), bgBgr, dtype=np.uint8)
            canvas[margin:margin + gridH, margin:margin + gridW] = grid
        else:
            canvas = grid

        # titles
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1.2
        thickness = 2
        textColor = (0, 0, 0)

        x0, y0 = margin, margin
        x1, y1 = x0 + tileW + gap, y0
        x2, y2 = x0, y0 + tileH + gap
        x3, y3 = x1, y2
        tileOrigins = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

        for title, (tileX, tileY) in zip(titles, tileOrigins):
            if title is None:
                continue
            textSize = cv2.getTextSize(title, font, fontScale, thickness)[0]
            textX = tileX + (tileW - textSize[0]) // 2 + titleXPadding
            textY = tileY + titleYOffset + textSize[1]
            cv2.putText(canvas, title, (textX, textY), font, fontScale, textColor, thickness, cv2.LINE_AA)

        self.frameCount += 1
        canvasH, _ = canvas.shape[:2]
        cv2.putText(canvas, f"board update: {self.frameCount}", (20, canvasH - 20),
                    font, fontScale, textColor, thickness, cv2.LINE_AA)

        return canvas



    def runStepLoop(self):

        displayFrame = self.createDisplayOutput()
        cv2.imshow(self.windowName, displayFrame)

        while True:

            key = cv2.waitKey(0) & 0xFF  # wait for key input

            if key == ord(' ') or key == 32 :  # spacebar

                originalFrame = self.frameRequester.requestFrameToMat()
                self.displayTiles[0] = originalFrame.copy()

                boardCorners = self.calibrationHelper.readCalibrationFromFile("board_corners.txt")
                rectifiedFrame, predictions = self.imageProcessor.occupancyDetectionPipeline(originalFrame, boardCorners)
                rectifiedFrame, scale = self.imageProcessor.downscaleForDisplay(rectifiedFrame, max_width=480, max_height=480)
                self.displayTiles[1] = self.embedInTile(rectifiedFrame)

                displayFrame = self.createDisplayOutput()
                cv2.imshow(self.windowName, displayFrame)

            elif key == ord('q') or key == 27:  # q or ESC -> quit
                break

        cv2.destroyWindow(self.windowName)




    def close(self):
        cv2.destroyWindow(self.windowName)




def main():

    chessarmDisplay = ChessarmDisplay()
    chessarmDisplay.runStepLoop()




if __name__ == "__main__":
    main()
    

    