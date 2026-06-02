from datetime import datetime
from pathlib import Path
import re


class GameLogger:

    LOG_PATTERN = re.compile(r"^game(\d+)\.txt$")

    def __init__(self, logsDir):
        self.logsDir = Path(logsDir)
        self.logsDir.mkdir(parents=True, exist_ok=True)
        self.createdAt = datetime.now()
        self.path = self._createNextLogFile()

    def _nextGameNumber(self):
        highest = 0
        for logPath in self.logsDir.iterdir():
            match = self.LOG_PATTERN.match(logPath.name)
            if match:
                highest = max(highest, int(match.group(1)))
        return highest + 1

    def _createNextLogFile(self):
        while True:
            logPath = self.logsDir / f"game{self._nextGameNumber()}.txt"
            try:
                with logPath.open("x", encoding="utf-8") as logFile:
                    logFile.write(f"{self.createdAt.isoformat(timespec='seconds')}\n")
                return logPath
            except FileExistsError:
                continue

    def logBoardState(self, fen):
        with self.path.open("a", encoding="utf-8") as logFile:
            logFile.write(f"{fen}\n")

    def logMove(self, move, fen):
        with self.path.open("a", encoding="utf-8") as logFile:
            logFile.write("\n")
            logFile.write(f"MOVE: {move}\n")
            logFile.write(f"FEN: {fen}\n")

    def logUndoMove(self, move, fen):
        with self.path.open("a", encoding="utf-8") as logFile:
            logFile.write("\n")
            logFile.write(f"UNDO MOVE: {move}\n")
            logFile.write(f"FEN: {fen}\n")
