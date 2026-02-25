from vision import CalibrationHelper
from frame_server.request_frame import FrameRequester

def main():

    calibrationHelper = CalibrationHelper()
    frameRequester = FrameRequester()

    cmd = input("Would you like to relocate the chessboard corners? (y/n): ")
    if cmd == "y" or cmd == "Y":
        frame = frameRequester.requestFrameToMat()
        corners = calibrationHelper.collectManualChessboardCorners(frame)
        calibrationHelper.writeCalibrationToFile(corners, filename="board_corners.txt")



if __name__ == "__main__":
    main()
