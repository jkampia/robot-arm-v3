from vision import ImageProcessor

import cv2


def main():

    import argparse

    parser = argparse.ArgumentParser(description="Generate training tiles from board images.")
    parser.add_argument("images", nargs="+", help="Paths to board images")
    args = parser.parse_args()

    processor = ImageProcessor()
    frames = []
    for image_path in args.images:
        frame = cv2.imread(image_path)
        if frame is None:
            parser.error(f"Could not read image: {image_path}")
        frames.append(frame)

    n = 0
    
    for frame in frames:
        cellRois = processor.rectifiedRoiPipeline(frame)
        _, numSaves = processor.saveRoisToDirectoryPNG(cellRois, "../../training_data/original_data", prefix = f"frame{n}")
        print(f"Saved {numSaves} tiles from frame {n}")
        n += 1



if __name__ == "__main__":
    main()