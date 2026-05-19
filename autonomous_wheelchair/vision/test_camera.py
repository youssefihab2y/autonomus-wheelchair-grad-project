from ultralytics import YOLO
import cv2

model = YOLO("/home/nadafouad/autonomus-wheelchair-grad-project-nav2/autonomous_wheelchair/yolov8n-oiv7.pt")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit(1)

print("Running YOLOv8 Open Images — press Q to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.5, verbose=False)
    annotated = results[0].plot()

    cv2.imshow("YOLOv8 Open Images - Laptop Camera", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
