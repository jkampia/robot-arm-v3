import torch
import torch.nn as nn
from torchvision import transforms
from PIL import Image
import os
import cv2
import numpy as np

import time


class TinyCNN(nn.Module):

    def __init__(self, num_classes=3, img_size=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Flatten(),
            nn.Linear(64 * (img_size // 8) * (img_size // 8), 128), nn.ReLU(),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        return self.net(x)


class OccupancyDetector:

    def __init__(self, model_path, class_names=['black', 'empty', 'white'], img_size=64):
        
        self.class_names = class_names
        self.img_size = img_size
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load model
        self.model = TinyCNN(num_classes=len(class_names), img_size=img_size).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        # Image transform
        self.transform = transforms.Compose([
            transforms.Resize((img_size, img_size)),
            transforms.ToTensor(),
            transforms.Normalize([0.5] * 3, [0.5] * 3)
        ])

    def predict(self, image_path):
        img = Image.open(image_path).convert("RGB")
        x = self.transform(img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            logits = self.model(x)
            pred_idx = torch.argmax(logits, dim=1).item()
            return self.class_names[pred_idx]
        
    def predict_cv(self, cv_img):

        # Convert BGR (OpenCV) → RGB (PIL)
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_img)

        # Apply same preprocessing
        x = self.transform(pil_img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            logits = self.model(x)
            pred_idx = torch.argmax(logits, dim=1).item()
            return self.class_names[pred_idx]



def orderCorners(pts):
    """Order 4 points as: top-left, top-right, bottom-right, bottom-left."""
    pts = np.array(pts, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    return np.array([tl, tr, br, bl], dtype=np.float32)



def downscaleForDisplay(img, max_width=1280, max_height=720):
    """
    Downscale image to fit inside max_width x max_height
    while preserving aspect ratio.

    Returns:
        resized_img
        scale_factor (float)
    """
    h, w = img.shape[:2]

    scale_w = max_width / w
    scale_h = max_height / h
    scale = min(scale_w, scale_h)

    # If image already smaller than target, don't upscale
    if scale >= 1.0:
        return img.copy(), 1.0

    new_w = int(w * scale)
    new_h = int(h * scale)

    # INTER_AREA is best for shrinking
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return resized, scale


import numpy as np

def transformCornersToDisplay(boardCornersFull, scaleFactor):
    """
    Convert full-res corner coordinates to display-res coordinates.

    Args:
        boardCornersFull: (4,2) array in full-res pixel coords (TL,TR,BR,BL)
        scaleFactor: float returned by downscaleForDisplay()

    Returns:
        boardCornersDisplay: (4,2) float32 array in display pixel coords
    """
    corners = np.array(boardCornersFull, dtype=np.float32).reshape(4, 2)
    return corners * float(scaleFactor)




def computeCornerAnglesDegrees(corners):
    """
    corners: (4,2) ordered TL,TR,BR,BL (or any consistent order)
    returns: 4 angles in degrees
    """
    pts = np.array(corners, dtype=np.float32).reshape(4, 2)
    angles = []
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i - 1) % 4]
        p2 = pts[(i + 1) % 4]

        v1 = p1 - p0
        v2 = p2 - p0

        # angle between v1 and v2
        denom = (np.linalg.norm(v1) * np.linalg.norm(v2)) + 1e-9
        cosAng = float(np.dot(v1, v2) / denom)
        cosAng = max(-1.0, min(1.0, cosAng))
        ang = np.degrees(np.arccos(cosAng))
        angles.append(ang)
    return np.array(angles, dtype=np.float32)



def findLargestQuadrilateral(bgr, minAreaFraction=0.05, angleToleranceDeg=25):
    """
    Finds the best quadrilateral candidate for a chessboard:
    - big enough (relative to image area)
    - convex
    - roughly rectangular (angles near 90°)
    Returns:
        (orderedCorners, edges) or (None, edges)
    """
    height, width = bgr.shape[:2]
    imageArea = float(height * width)
    minArea = imageArea * float(minAreaFraction)  # e.g., 5% of image

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(gray, 50, 150)

    # Close gaps in edges so the board outline forms one contour
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find contours (RETR_EXTERNAL is OK, RETR_LIST can sometimes help if outline isn't outermost)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, edges

    bestCorners = None
    bestScore = -1.0

    # Look at more than 10; sorting is cheap
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:50]

    for cnt in cnts:
        area = float(cv2.contourArea(cnt))
        if area < minArea:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) != 4:
            continue

        if not cv2.isContourConvex(approx):
            continue

        quad = approx.reshape(4, 2)
        corners = orderCorners(quad)  # your existing TL,TR,BR,BL ordering

        # Rectangle-ness check: angles near 90
        angles = computeCornerAnglesDegrees(corners)
        if np.any(np.abs(angles - 90.0) > float(angleToleranceDeg)):
            continue

        # Score: prefer large area, and also prefer more “rectangular”
        # (penalize angle error)
        angleError = float(np.mean(np.abs(angles - 90.0)))
        score = area - (angleError * 1000.0)  # tune weight if needed

        if score > bestScore:
            bestScore = score
            bestCorners = corners

    return bestCorners, edges



def drawBoardEdges(image, boardCorners, lineThickness=4, drawLabels=True):
    """
    Draw a quadrilateral (board edges) onto the image.

    Args:
        image: BGR image (numpy array)
        boardCorners: array-like shape (4,2) in order [tl, tr, br, bl]
        lineThickness: thickness of the outline
        drawLabels: whether to label corners

    Returns:
        overlayImage: copy of image with edges drawn
    """
    overlayImage = image.copy()

    corners = np.array(boardCorners, dtype=np.float32).reshape(4, 2)
    cornersInt = np.round(corners).astype(np.int32)

    # Draw polygon outline
    cv2.polylines(
        overlayImage,
        [cornersInt.reshape(-1, 1, 2)],
        isClosed=True,
        color=(0, 255, 0),
        thickness=lineThickness,
        lineType=cv2.LINE_AA
    )

    # Draw corner points
    for i, (x, y) in enumerate(cornersInt):
        cv2.circle(overlayImage, (int(x), int(y)), 8, (0, 0, 255), -1, lineType=cv2.LINE_AA)

    # Optional labels
    if drawLabels:
        labels = ["TL", "TR", "BR", "BL"]
        for (x, y), label in zip(cornersInt, labels):
            cv2.putText(
                overlayImage,
                label,
                (int(x) + 10, int(y) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 0),
                2,
                cv2.LINE_AA
            )

    return overlayImage




def rectifyBoard(image, boardCorners, outputWidth=800, outputHeight=800):
    """
    Rectify (perspective-warp) the board region to a top-down view.

    Args:
        image: BGR image (numpy array)
        boardCorners: (4,2) corners in TL,TR,BR,BL order (full-res coords)
        outputWidth: output image width in pixels
        outputHeight: output image height in pixels

    Returns:
        rectifiedImage: warped board image (BGR)
        homography: 3x3 perspective transform matrix mapping input->output
    """
    corners = np.array(boardCorners, dtype=np.float32).reshape(4, 2)

    dstCorners = np.array([
        [0, 0],
        [outputWidth - 1, 0],
        [outputWidth - 1, outputHeight - 1],
        [0, outputHeight - 1]
    ], dtype=np.float32)

    homography = cv2.getPerspectiveTransform(corners, dstCorners)
    rectifiedImage = cv2.warpPerspective(image, homography, (outputWidth, outputHeight))

    return rectifiedImage, homography




def sliceRectifiedBoardIntoCells(rectifiedImage, drawGrid=True, lineThickness=2):
    """
    Slice a rectified board image into an 8x8 grid.

    Args:
        rectifiedImage: BGR or grayscale image (H,W[,C])
        drawGrid: whether to draw the 7 internal grid lines
        lineThickness: thickness of drawn lines

    Returns:
        gridOverlayImage: image with grid drawn (copy of rectifiedImage)
        cellRois: list of 64 ROI images (row-major: r0c0 ... r7c7)
        cellRects: list of 64 rects (x, y, w, h) matching cellRois
    """
    height, width = rectifiedImage.shape[:2]

    # Compute cell boundaries (use integer rounding; last cell absorbs remainder)
    xBounds = [round(i * width / 8.0) for i in range(9)]
    yBounds = [round(i * height / 8.0) for i in range(9)]

    # Clamp + force last boundary to exact size
    xBounds[0] = 0
    yBounds[0] = 0
    xBounds[-1] = width
    yBounds[-1] = height

    gridOverlayImage = rectifiedImage.copy()

    if drawGrid:
        # Choose line color based on channels
        lineColor = (0, 255, 0) if rectifiedImage.ndim == 3 else 255

        # Draw 7 internal vertical lines (exclude 0 and width)
        for i in range(1, 8):
            x = int(xBounds[i])
            cv2.line(gridOverlayImage, (x, 0), (x, height - 1), lineColor, lineThickness, cv2.LINE_AA)

        # Draw 7 internal horizontal lines (exclude 0 and height)
        for i in range(1, 8):
            y = int(yBounds[i])
            cv2.line(gridOverlayImage, (0, y), (width - 1, y), lineColor, lineThickness, cv2.LINE_AA)

    cellRois = []
    cellRects = []

    # Row-major order: row 0..7 (top to bottom), col 0..7 (left to right)
    for row in range(8):
        y0 = int(yBounds[row])
        y1 = int(yBounds[row + 1])
        for col in range(8):
            x0 = int(xBounds[col])
            x1 = int(xBounds[col + 1])

            roi = rectifiedImage[y0:y1, x0:x1]
            cellRois.append(roi)
            cellRects.append((x0, y0, x1 - x0, y1 - y0))

    return gridOverlayImage, cellRois, cellRects




def visualizeAverageColors(cellRois, outputSize=800):
    """
    Create an 8x8 visualization image where each square is filled
    with the average color of the corresponding ROI.

    Args:
        cellRois: list of 64 ROI images (row-major order)
        outputSize: size of output square image (default 800x800)

    Returns:
        colorImage: 800x800 BGR image
        avgColors: list of 64 average BGR colors
    """
    if len(cellRois) != 64:
        raise ValueError(f"Expected 64 ROIs, got {len(cellRois)}")

    cellSize = outputSize // 8
    colorImage = np.zeros((outputSize, outputSize, 3), dtype=np.uint8)

    avgColors = []

    for idx, roi in enumerate(cellRois):
        if roi.size == 0:
            avgColor = np.array([0, 0, 0], dtype=np.uint8)
        else:
            # Compute mean per channel (BGR)
            meanBgr = np.mean(roi.reshape(-1, roi.shape[-1]), axis=0)
            avgColor = np.clip(meanBgr, 0, 255).astype(np.uint8)

        avgColors.append(avgColor)

        row = idx // 8
        col = idx % 8

        y0 = row * cellSize
        y1 = (row + 1) * cellSize
        x0 = col * cellSize
        x1 = (col + 1) * cellSize

        colorImage[y0:y1, x0:x1] = avgColor

    return colorImage, avgColors



def main():

    startTime = time.time()

    # ingest image
    frame = cv2.imread("frame_server/testframe.jpg")

    # find most likely chessboard edges
    boardCorners, edges = findLargestQuadrilateral(frame)

    # order
    boardCorners = orderCorners(boardCorners)

    # draw them onto new frame
    #edgedFrame = drawBoardEdges(frame, boardCorners)

    #downscale frame
    #displayImage, scale = downscaleForDisplay(edgedFrame)

    # display that frame
    #cv2.imshow("large", displayImage)

    #rectify frame
    rectifiedFrame, homography = rectifyBoard(frame, boardCorners)

    # draw cells
    rectifiedFrame, cellRois, cellRects = sliceRectifiedBoardIntoCells(rectifiedFrame)

    # display rectified frame
    cv2.imshow("rectified", rectifiedFrame)


    stopTime = time.time()
    print(f"Time elapsed: {stopTime-startTime} s")

    key = cv2.waitKey(0) & 0xFF  # wait for key input
    if key == ord('q') or key == 27:  # q or ESC -> quit
        cv2.destroyAllWindows()
        return




if __name__ == "__main__":
    main()



"""
EXAMPLE USAGE:

detector = OccupancyDetector(
    model_path="chess_square_classifier.pt",
    class_names=["black", "empty", "white"]
)

test_dir = "empty"  # <-- Change to your directory of test images
valid_exts = (".png", ".jpg", ".jpeg")

# -------- RUN PREDICTIONS --------
for fname in sorted(os.listdir(test_dir)):
    if not fname.lower().endswith(valid_exts):
        continue

    img_path = os.path.join(test_dir, fname)
    label = detector.predict(img_path)
    print(f"{fname:<30} → {label}")

"""