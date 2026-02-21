import os
import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader, random_split
from pathlib import Path

# -------- CONFIG --------
DATA_DIR = Path("aug_data/")  # where empty_aug/, white_aug/, black_aug/ are located
BATCH_SIZE = 32
EPOCHS = 10
LR = 1e-3
IMG_SIZE = 64  # assuming square crops
MODEL_NAME = "square_classifier.pt"


# -------- TRANSFORMS --------
transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),  # converts to [0,1] tensor
    transforms.Normalize([0.5]*3, [0.5]*3)  # normalize to [-1,1]
])

# -------- DATASET --------
dataset = datasets.ImageFolder(root=DATA_DIR, transform=transform)
class_names = dataset.classes
print("Classes:", class_names)

# Split into train/val (90/10)
train_size = int(0.9 * len(dataset))
val_size = len(dataset) - train_size
train_ds, val_ds = random_split(dataset, [train_size, val_size])

train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True, num_workers=2)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=2)

# -------- MODEL --------
class TinyCNN(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Flatten(),
            nn.Linear(64 * (IMG_SIZE//8) * (IMG_SIZE//8), 128), nn.ReLU(),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        return self.net(x)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = TinyCNN(num_classes=len(class_names)).to(device)

# -------- TRAINING SETUP --------
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=LR)

# -------- TRAIN LOOP --------
for epoch in range(EPOCHS):
    model.train()
    total_loss = 0
    correct = 0
    for images, labels in train_loader:
        images, labels = images.to(device), labels.to(device)

        optimizer.zero_grad()
        logits = model(images)
        loss = criterion(logits, labels)
        loss.backward()
        optimizer.step()

        total_loss += loss.item() * images.size(0)
        preds = torch.argmax(logits, dim=1)
        correct += (preds == labels).sum().item()

    acc = correct / train_size
    print(f"Epoch {epoch+1}/{EPOCHS} - Train loss: {total_loss/train_size:.4f} | Train acc: {acc:.4f}")

    # --- Validation ---
    model.eval()
    val_correct = 0
    with torch.no_grad():
        for images, labels in val_loader:
            images, labels = images.to(device), labels.to(device)
            logits = model(images)
            preds = torch.argmax(logits, dim=1)
            val_correct += (preds == labels).sum().item()
    val_acc = val_correct / val_size
    print(f"             Val acc: {val_acc:.4f}")

# -------- SAVE MODEL --------
torch.save(model.state_dict(), MODEL_NAME)
print(f"Model saved as {MODEL_NAME}")
