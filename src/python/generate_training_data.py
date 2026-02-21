from occupancy_detector import ImageProcessor

import cv2


def main():

    processor = ImageProcessor()

    frames = []

    frame1 = cv2.imread("frame_server/testframe.jpg")
    frames.append(frame1)

    n = 0
    
    for frame in frames:
        cellRois = processor.rectifiedRoiPipeline(frame)
        _, numSaves = processor.saveRoisToDirectoryPNG(cellRois, "../../training_data/original_data", prefix = f"frame{n}")
        print(f"Saved {numSaves} tiles from frame {n}")
        n += 1



if __name__ == "__main__":
    main()