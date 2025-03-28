from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt


class ImagePaths:
    def __init__(self) -> None:
        self.overheadangled = "resources/overheadangled.png"
        self.topdown = "resources/topdown.png"
        self.verysideangle = "resources/verysideangle.png"
        self.sideangle = "resources/sideangle.png"
        self.topanglefar = "resources/topanglefar.png"

    def detect_cubes(self, im_pth) -> None:
        image_path = im_pth
        # Load the model
        model_path = "resources/yolo11l-obb.pt"
        model = YOLO(model_path)
        image = cv2.imread(image_path)

        # Convert BGR to RGB (for plotting)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Run inference
        results = model(image)
        # Plot results
        annotated_frame = results[0].plot()
        annotated_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB for visualization

        # Show the result
        plt.imshow(annotated_rgb)
        plt.title("YOLOv11 Detection")
        plt.axis("off")
        plt.show()


# Load the image
image_path = ImagePaths()
image_path.detect_cubes(image_path.overheadangled)
image_path.detect_cubes(image_path.topdown)
image_path.detect_cubes(image_path.verysideangle)
image_path.detect_cubes(image_path.sideangle)
image_path.detect_cubes(image_path.topanglefar)


# Optional: save the result
# cv2.imwrite("Thesis/lib/object_detect_lib/resources/overheadangled_result.png", annotated_frame)
