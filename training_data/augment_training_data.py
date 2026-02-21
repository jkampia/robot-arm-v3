import cv2
import numpy as np
import os
import random
import shutil

def random_brightness_contrast(img, brightness_range=(-40, 40), contrast_range=(0.8, 1.2)):
    brightness = random.randint(*brightness_range)
    contrast = random.uniform(*contrast_range)
    img = img.astype(np.float32) * contrast + brightness
    return np.clip(img, 0, 255).astype(np.uint8)

def random_blur(img, max_ksize=3):
    if random.random() < 0.5:
        k = random.choice([1, 3])  # odd kernel size
        return cv2.GaussianBlur(img, (k, k), 0)
    return img

def random_rotation(img, max_angle=8):
    if random.random() < 0.5:
        angle = random.uniform(-max_angle, max_angle)
        h, w = img.shape[:2]
        M = cv2.getRotationMatrix2D((w/2, h/2), angle, 1.0)
        return cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REFLECT)
    return img

def augment_image(img):
    img = random_brightness_contrast(img)
    img = random_blur(img)
    img = random_rotation(img)
    return img

def augment_dataset(src_class_dir, dst_aug_class_dir, num_augments=5, exts=(".png", ".jpg", ".jpeg")):
    
    os.makedirs(dst_aug_class_dir, exist_ok=True)

    images = [f for f in os.listdir(src_class_dir) if f.lower().endswith(exts)]

    for fname in images:
        src_path = os.path.join(src_class_dir, fname)
        dst_path = os.path.join(dst_aug_class_dir, fname)

        # Copy original
        shutil.copy(src_path, dst_path)

        img = cv2.imread(src_path)
        if img is None:
            continue

        base_name = os.path.splitext(fname)[0]

        for i in range(num_augments):
            aug = augment_image(img)
            new_name = f"{base_name}_aug{i}.png"
            cv2.imwrite(os.path.join(dst_aug_class_dir, new_name), aug)

    print(f"{len(images)} images copied and augmented ({num_augments}x) into '{dst_aug_class_dir}'")



if __name__ == "__main__":
    """
    Augment datasets after they have been labeled
    so that you don't have to relabel them when you already know
    what the image contains!
    """

    augment_dataset("sorted_data/empty", "aug_data/empty", num_augments=5)
    augment_dataset("sorted_data/white", "aug_data/white", num_augments=5)
    augment_dataset("sorted_data/black", "aug_data/black", num_augments=5)
