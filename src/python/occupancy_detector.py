import torch
import torch.nn as nn
from torchvision import transforms
from PIL import Image
import os
import cv2


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