import cv2

import numpy as np

import os

import uuid

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .occupancy_detector import OccupancyDetector



class VisionHelper(Node):

    def __init__(self):

        super().__init__('vision_helper')

        self.bridge = CvBridge()
        self.rgb_frame_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgbImageCallback, 10)
        self.main_timer = self.create_timer(0.1, self.main_timer_cb)
        self.first_pass = True
        self.cv_image = None
        self.detector = OccupancyDetector(
            model_path="training_data/chess_square_classifier.pt",
            class_names=["black", "empty", "white"]
        )


    
    def rgbImageCallback(self, msg):

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.cv_image_disp = self.cv_image.copy()
        #contours = self.findChessboardSquares(cv_image, True)


    def get_initial_user_input(self):
        
        userin = input("Enter 'c' to calibrate the camera or 's' to skip calibration: ").strip().lower()
        
        if userin == 'c':
            while self.cv_image is None:
                self.get_logger().info("Waiting for camera image...")
                rclpy.spin_once(self, timeout_sec=0.1)
            self.pts = self.collect_four_points(self.cv_image)
            self.write_calibration_to_file(self.pts)
            print(f'Got corners: {self.pts}')
            self.first_pass = False

        elif userin == 's':
            self.get_logger().info("Skipping camera calibration.")
            self.pts = self.read_calibration_from_file()
            self.first_pass = False

        elif userin == 'd':
            while self.cv_image is None:
                self.get_logger().info("Waiting for camera image...")
                rclpy.spin_once(self, timeout_sec=0.1)
            self.pts = self.read_calibration_from_file()
            grid, quads = self.grid_from_corners(self.cv_image, self.cv_image_disp, self.pts, cells=8, assume_ordered=False, draw=True)
            self.save_64_squares(self.cv_image, quads)
            self.first_pass = False

                
        
    def main_timer_cb(self):

        if self.first_pass:
            self.get_initial_user_input()
    
        else:
            poly = np.array(self.pts, dtype=np.int32)        # shape (4,2)
            cv2.polylines(self.cv_image_disp, [poly], isClosed=True, color=(0,255,0), thickness=2)
            grid, quads = self.grid_from_corners(self.cv_image, self.cv_image_disp, self.pts, cells=8, assume_ordered=False, draw=True)
            labels = self.analyze_board_and_overlay(self.cv_image, self.cv_image_disp, quads, cells=8, cell_size=64, alpha=0.35)
            self.displayImage(self.cv_image_disp)


    
    def write_calibration_to_file(self, pts, filename='calibration.txt'):
        if pts is not None:
            with open(filename, 'w') as f:
                for pt in self.pts:
                    f.write(f"{pt[0]},{pt[1]}\n")



    def read_calibration_from_file(self, filename='calibration.txt'):
        try:
            with open(filename, 'r') as f:
                pts = [tuple(map(int, line.strip().split(','))) for line in f.readlines()]
                return pts
        except FileNotFoundError:
            self.get_logger().error(f"Calibration file '{filename}' not found or is empty. Please calibrate the camera first.")
            return None
        

    
    def displayImage(self, img):

        cv2.imshow("Detected Chessboard", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Pressed 'q'. Shutting down.")
            rclpy.shutdown()
            cv2.destroyAllWindows()

    
    
    def collect_four_points(self, frame, window_name="Select 4 points", timeout_ms=None):
        """
        Displays `frame`, waits for 4 left-clicks, returns Nx2 int array of (x,y).
        Right-click = undo last point. Esc = cancel -> returns None.
        If timeout_ms is set, cancels after that many milliseconds.
        """
        img = frame.copy()
        pts = []

        def on_click(event, x, y, flags, userdata):
            nonlocal img, pts
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(pts) < 4:
                    pts.append((x, y))
            elif event == cv2.EVENT_RBUTTONDOWN:
                if pts:
                    pts.pop()

            # redraw overlay
            img = frame.copy()
            for i, (px, py) in enumerate(pts):
                cv2.circle(img, (px, py), 5, (0, 0, 255), -1)
                cv2.putText(img, str(i+1), (px+6, py-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if len(pts) >= 2:
                for a in range(len(pts)-1):
                    cv2.line(img, pts[a], pts[a+1], (0, 255, 0), 2)

            cv2.imshow(window_name, img)

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, frame.shape[1], frame.shape[0])
        cv2.setMouseCallback(window_name, on_click)
        cv2.imshow(window_name, img)

        start = cv2.getTickCount()
        while True:
            key = cv2.waitKey(20) & 0xFF
            if len(pts) == 4:
                break
            if key == 27:  # Esc
                pts = None
                break
            if timeout_ms is not None:
                elapsed_ms = (cv2.getTickCount() - start) / cv2.getTickFrequency() * 1000.0
                if elapsed_ms > timeout_ms:
                    pts = None
                    break

        cv2.destroyWindow(window_name)
        return None if pts is None else np.array(pts, dtype=int)
    


    def analyze_board_and_overlay(self, frame_bgr, img_disp, quads, cells=8, cell_size=64, alpha=0.35):
        """
        frame_bgr : original image
        quads     : list/array of 64 quads (each (4,2) TL,TR,BR,BL) in source image coords
        cells     : 8 for chess
        cell_size : warp size for classification (e.g., 64x64)
        alpha     : overlay opacity for filled squares

        returns:
        labels_8x8 : np.ndarray (cells, cells) of {"empty","white","black"}
        overlay    : frame with semi-transparent fills on occupied quads
        """
        H, W = frame_bgr.shape[:2]
        labels_8x8 = np.empty((cells, cells), dtype=object)

        dst_sq = np.float32([[0,0],[cell_size-1,0],[cell_size-1,cell_size-1],[0,cell_size-1]])

        # safety: ensure we can index quads as (8,8,4,2)
        quads_arr = np.asarray(quads, dtype=np.float32).reshape(cells, cells, 4, 2)

        for r in range(cells):
            for c in range(cells):
                q = quads_arr[r, c].astype(np.float32)

                # Warp to normalized square
                Hcell = cv2.getPerspectiveTransform(q, dst_sq)
                roi = cv2.warpPerspective(frame_bgr, Hcell, (cell_size, cell_size))

                
                # Classify
                label = self.detector.predict_cv(roi)  # "empty" | "white" | "black"
                
                print(f"Square ({r},{c}) classified as: {label}")
                labels_8x8[r, c] = label

                if label != "empty":
                    fill_color = (255, 255, 255) if label == "white" else (0, 0, 0)
                    poly = q.astype(np.int32).reshape(-1, 1, 2)
                    cv2.polylines(self.cv_image_disp, [poly], isClosed=True, color=fill_color, thickness=2)


        return labels_8x8



    def grid_from_corners(self, img, img_disp, corners_4x2, cells=8, assume_ordered=False, draw=True):
        """
        corners_4x2: np.ndarray, shape (4,2), dtype float/int
        If assume_ordered=True, order is TL, TR, BR, BL.
        Otherwise we auto-order them.
        returns:
        grid_src: (cells+1, cells+1, 2) float32  # 9x9 corners in source image
        quads:    list of (4,2) float32          # 64 TL,TR,BR,BL quads in source image
        overlay:  visualization (or None if draw=False)
        """
        S = 800  # working square size for the rectified board

        c = np.asarray(corners_4x2, dtype=np.float32).reshape(4,2)
        if not assume_ordered:
            # order corners as TL, TR, BR, BL
            s = c.sum(axis=1); d = np.diff(c, axis=1).ravel()
            tl = c[np.argmin(s)]; br = c[np.argmax(s)]
            tr = c[np.argmin(d)]; bl = c[np.argmax(d)]
            src = np.array([tl,tr,br,bl], dtype=np.float32)
        else:
            src = c.astype(np.float32)

        dst = np.float32([[0,0],[S,0],[S,S],[0,S]])
        Hm   = cv2.getPerspectiveTransform(src, dst)
        Hinv = np.linalg.inv(Hm)

        ticks = np.linspace(0, S, cells+1, dtype=np.float32)
        gx, gy = np.meshgrid(ticks, ticks)                  # 9x9
        grid_dst = np.stack([gx, gy], axis=-1).reshape(-1,1,2)
        grid_src = cv2.perspectiveTransform(grid_dst, Hinv).reshape(cells+1, cells+1, 2).astype(np.float32)

        # Collect 64 cell quads (TL,TR,BR,BL) in source image coords
        quads = []
        for r in range(cells):
            for c_ in range(cells):
                tl = grid_src[r,   c_  ]
                tr = grid_src[r,   c_+1]
                br = grid_src[r+1, c_+1]
                bl = grid_src[r+1, c_  ]
                quads.append(np.array([tl,tr,br,bl], dtype=np.float32))

        if draw:
            # draw grid lines
            for r in range(cells+1):
                pts = grid_src[r].astype(int)
                for i in range(cells):
                    cv2.line(img_disp, tuple(pts[i]), tuple(pts[i+1]), (0,255,0), 1)
            for c_ in range(cells+1):
                pts = grid_src[:,c_].astype(int)
                for i in range(cells):
                    cv2.line(img_disp, tuple(pts[i]), tuple(pts[i+1]), (0,255,0), 1)
            # outer boundary
            cv2.polylines(img_disp, [src.astype(int)], True, (0,0,255), 2)

        return grid_src, quads
    


    def save_64_squares(self, frame_bgr, quads, directory="training_data/original/"):
        """
        Save 64 rectangular crops 
        Each crop is the bounding rectangle enclosing the quad
        """
        os.makedirs(directory, exist_ok=True)
        H, W = frame_bgr.shape[:2]

        for idx, q in enumerate(quads):
            q = q.astype(int)

            x0, y0 = q[:,0].min(), q[:,1].min()
            x1, y1 = q[:,0].max(), q[:,1].max()

            # clamp to image bounds
            x0 = max(0, min(W-1, x0))
            x1 = max(0, min(W,   x1))
            y0 = max(0, min(H-1, y0))
            y1 = max(0, min(H,   y1))

            # crop rectangular ROI
            crop = frame_bgr[y0:y1, x0:x1]

            # save
            rand_name = f"{uuid.uuid4().hex}.png"
            filename = os.path.join(directory, rand_name)
            cv2.imwrite(filename, crop)
            print(f'Saved file {filename} to {directory}')


    


    """ 
    EVERYTHING THAT FOLLOWS IS FOR DYNAMIC CHESSBOARD DETECTION
    - NOT USED YET, BUT MAY BE USEFUL IN THE FUTURE
    - IT IS A WORK IN PROGRESS
    """

    def line_slope(self, x1, y1, x2, y2):
        if x2 - x1 == 0:
            return float('inf')
        return (y2 - y1) / (x2 - x1)
    

    def cartesian_to_normal(self, x1,y1,x2,y2):
        vx, vy = x2-x1, y2-y1
        theta = (np.arctan2(vy, vx) + np.pi/2) % np.pi
        rho = x1*np.cos(theta) + y1*np.sin(theta)
        return rho, theta   
    


    def fit_and_extend(self, group, h, w):
        pts = []
        for x1,y1,x2,y2 in group:
            pts += [(x1,y1),(x2,y2)]
        pts = np.array(pts, dtype=np.float32)
        [vx,vy,x0,y0] = cv2.fitLine(pts, cv2.DIST_L2,0,0.01,0.01)  # robust fit
        # parametric form; intersect with image rectangle
        # compute two far points along the direction, then clip:
        p1 = (x0 - 2000*vx, y0 - 2000*vy); p2 = (x0 + 2000*vx, y0 + 2000*vy)
        return np.array(p1).ravel(), np.array(p2).ravel()
        
    

    def warpImage(self, image_bgr, outer_corners, debug=False):
        # Order outer corners: top-left, top-right, bottom-right, bottom-left
        outer = np.array(outer_corners, dtype=np.float32)
        rect = np.zeros((4, 2), dtype="float32")
        s = outer.sum(axis=1)
        rect[0] = outer[np.argmin(s)]      # top-left
        rect[2] = outer[np.argmax(s)]      # bottom-right
        diff = np.diff(outer, axis=1)
        rect[1] = outer[np.argmin(diff)]   # top-right
        rect[3] = outer[np.argmax(diff)]   # bottom-left

        side = 800
        dst = np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype="float32")
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image_bgr, M, (side, side))

        return warped
    

    def divideImageIntoSquares(self, img):
        
        squares = []
        contours = []

        height, width = img.shape[:2]
        side_h = height // 8
        side_w = width // 8

        for i in range(8):
            for j in range(8):
                x = j * side_w
                y = i * side_h
                square = img[y:y + side_h, x:x + side_w]
                squares.append(square)

                # Define square contour (clockwise)
                contour = np.array([
                    [[x, y]],
                    [[x + side_w - 1, y]],
                    [[x + side_w - 1, y + side_h - 1]],
                    [[x, y + side_h - 1]]
                ], dtype=np.int32)
                contours.append(contour)

        return squares, contours

    
    def decideSquareOccupancy(self, square_img, contour, idx):
        
        gray = cv2.cvtColor(square_img, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255,
                                    cv2.ADAPTIVE_THRESH_MEAN_C,
                                    cv2.THRESH_BINARY_INV, 11, 3)

        white_pixels = cv2.countNonZero(thresh)
        total_pixels = thresh.shape[0] * thresh.shape[1]
        fill_ratio = white_pixels / total_pixels
        print(f"Fill ratio at square {idx}: {fill_ratio:.2f}")

        occupation_threshold = 0.05  # tweak this threshold empirically
        if fill_ratio < occupation_threshold:
            return # No piece detected
        
        # If we reach here, a piece is decidedly present
        avg_intensity = np.mean(gray)
        piece_color = 'white' if avg_intensity > 127 else 'black'
        self.markSquareStatus(square_img, contour, piece_color)




    def markSquareStatus(self, image, contour, piece_color=None):
       
        # Calculate center of the contour (approx as mean of corners)
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return  # avoid division by zero
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Choose color
        if piece_color == 'white':
            color = (255, 0, 0)
        elif piece_color == 'black':
            color = (0, 255, 0)
        else:
            return  # no piece, do not mark

        # Draw circle at center
        cv2.circle(image, (cx, cy), radius=10, color=color, thickness=-1)


    def findChessboardSquares(self, img, debug=False):
        
        # preprocess image
        # pretty standard opencv stuff
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        # blur to reduce noise
        gaussian_blur = cv2.GaussianBlur(gray, (5, 5), 0) 
        # otsu binary, adaptively black and white the image
        ret, otsu_binary = cv2.threshold(gaussian_blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # canny edge detection
        # some of the edges are gonna be grainy / not solid but thats fine
        canny = cv2.Canny(otsu_binary, 20, 255)
        # dilate image to thicken the edges & make contours more 'visible'
        kernel = np.ones((7, 7), np.uint8) 
        img_dilation = cv2.dilate(canny, kernel, iterations=1) 
        # add hough lines to image
        lines = cv2.HoughLinesP(img_dilation, 1, np.pi/180, threshold=200, minLineLength=100, maxLineGap=50)

        img_color = cv2.cvtColor(img_dilation, cv2.COLOR_GRAY2BGR)

        if lines is not None:
            for i, line in enumerate(lines):
                x1, y1, x2, y2 = line[0]
                cv2.line(img_color, (x1, y1), (x2, y2), (255, 255, 255), 2)


        self.pull_relevant_lines(lines, img_color)

        
        self.displayImage(img_color)



    def pull_relevant_lines(self, lines, img, min_len=400, ang_tol_deg=12, dst_size=800):
        """
        - Keep only segments longer than min_len (pixels)
        - Classify by angle: grpA = horizontal, grpB = vertical (within ang_tol_deg)
        - Draw A red, B green
        - Fit long lines for each group and return warp info
        """

        if lines is None or len(lines) == 0:
            return [], [], (None,None,None,None), None, None

        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        S = lines.reshape(-1,4).astype(np.float32)                # (x1,y1,x2,y2)
        dx, dy = S[:,2]-S[:,0], S[:,3]-S[:,1]
        lens = np.hypot(dx, dy)
        ang  = np.abs(np.arctan2(dy, dx))                         # [0, π]

        # 1) Length filter
        keep_len = lens >= float(min_len)
        S, ang = S[keep_len], ang[keep_len]
        if len(S) < 4:
            return [], [], (None,None,None,None), None, None

        # 2) Angle classification (tolerance)
        tol = np.deg2rad(float(ang_tol_deg))
        horiz_mask = (ang < tol) | (np.abs(ang - np.pi) < tol)    # ~0 or ~π
        vert_mask  = (np.abs(ang - np.pi/2) < tol)                # ~π/2

        grpA = S[horiz_mask]  # horizontal
        grpB = S[vert_mask]   # vertical

        # If tolerance leaves gaps, relax a bit or choose closest family:
        if len(grpA) == 0 and len(S) > 0:
            # take lines closer to horizontal than vertical
            closer_to_h = (np.abs(ang - 0) < np.abs(ang - np.pi/2)) | (np.abs(ang - np.pi) < np.abs(ang - np.pi/2))
            grpA = S[closer_to_h]
        if len(grpB) == 0 and len(S) > 0:
            grpB = S[~horiz_mask]  # whatever isn't horizontal becomes vertical-ish fallback

        # 3) Draw overlays
        for x1,y1,x2,y2 in grpA.astype(int): cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)   # red = horizontal
        for x1,y1,x2,y2 in grpB.astype(int): cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)   # green = vertical






def main(args=None):
    rclpy.init(args=args)
    node = VisionHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()