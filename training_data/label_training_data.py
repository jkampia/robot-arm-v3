import os
import cv2
import shutil

src_dir = "original_data"
dst_dirs = {
    'e': "sorted_data/empty",
    'w': "sorted_data/white",
    'b': "sorted_data/black"
}

for d in dst_dirs.values():
    os.makedirs(d, exist_ok=True)

files = sorted([f for f in os.listdir(src_dir) if f.endswith(".png")])
undo_stack = []

for f in files:
    path = os.path.join(src_dir, f)
    img = cv2.imread(path)
    if img is None:
        continue

    while True:
        cv2.imshow("Label (e = empty, w = white, b = black, u = undo, q = quit)", img)
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q'):
            exit()
        elif key == ord('u') and undo_stack:
            last_src, last_dst = undo_stack.pop()
            if os.path.exists(last_dst):
                shutil.move(last_dst, last_src)
                print(f"Undid move: {os.path.basename(last_dst)} moved back to original")
            else:
                print(f"Could not find {last_dst} to undo.")
        elif chr(key) in dst_dirs:
            dst_folder = dst_dirs[chr(key)]
            dst_path = os.path.join(dst_folder, f)
            shutil.move(path, dst_path)
            undo_stack.append((path, dst_path))
            print(f"Moved '{f}' to '{dst_folder}'")
            break
        else:
            print("Invalid key. Press e (empty), w (white), b (black), u (undo), or q (quit).")

cv2.destroyAllWindows()


