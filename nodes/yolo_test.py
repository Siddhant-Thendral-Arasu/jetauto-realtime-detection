from ultralytics import YOLOE
import cv2

model = YOLOE('/home/ubuntu/dev_ws/yoloe-11s-seg.pt')

img = cv2.imread('/home/ubuntu/frame.jpg')
if img is None:
	print("No frame detected")
else:
	results = model.predict(img, conf=0.25)
	cv2.imwrite('/home/ubuntu/detected.jpg', results[0].plot())
	for box in results[0].boxes:
		print(f"Found: {model.names[int(box.cls)]} ({float(box.conf):.2f})")
	print("Saved to /home/ubuntu/detected.jpg")
