import torch
import cv2
import numpy as np

# model_path="E:/NVIDIA/isaacsim/myscripts/Thesis/lib/object_detect_lib/resources/yolo_train_dataset/yolo_v11/runs/detect/train4/weights/best.pt",


class YOLODetector:
    def __init__(
        self,
        model_path="best.pt",
        use_half=True,
        weights_only=True,
    ):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.load(model_path, weights_only=False, map_location=self.device)
        self.model.to(self.device).eval()
        if use_half:
            self.model.half()

    def preprocess(self, frame):
        # Convert from BGR to RGB
        frame = frame[:, :, ::-1]
        frame = np.ascontiguousarray(frame)
        tensor = torch.from_numpy(frame).to(self.device)
        tensor = tensor.permute(2, 0, 1)  # (H, W, 3) -> (3, H, W)
        if self.model.dtype == torch.float16:
            tensor = tensor.half()
        else:
            tensor = tensor.float()
        tensor = tensor.unsqueeze(0)  # (1, 3, H, W)
        return tensor

    def detect(self, frame):
        # Preprocess
        input_tensor = self.preprocess(frame)
        with torch.no_grad():
            results = self.model(input_tensor)
        # TODO: parse results if needed
        return results
