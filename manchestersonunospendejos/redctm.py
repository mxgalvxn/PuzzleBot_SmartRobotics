import cv2
import numpy as np
from PIL import Image

# Cargar el modelo ONNX
net = cv2.dnn.readNetFromONNX('/home/red/src/onnx_inference/scripts/best.onnx')


net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4

# Definir las clases de yolov8
CLASES_YOLO = ['RED', 'GREEN', 'YELLOW', 'RIGHT', 'CROSS', 'FORWARD', 'STOP']

# Cargar imagen
image = cv2.imread('/home/puzzlebot/Downloads/WhatsApp Image 2023-06-06 at 12.31.25 AM.jpeg')

# Preprocesar la imagen
blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
net.setInput(blob)

# Realizar la inferencia
preds = net.forward()
preds = preds.transpose((0, 2, 1))

# Extraer detecciones
class_ids, confs, boxes = [], [], []

image_height, image_width, _ = image.shape
x_factor = image_width / INPUT_WIDTH
y_factor = image_height / INPUT_HEIGHT

rows = preds[0].shape[0]

for i in range(rows):
    row = preds[0][i]
    conf = row[4]
    
    classes_score = row[4:]
    _, _, _, max_idx = cv2.minMaxLoc(classes_score)
    class_id = max_idx[1]
    if classes_score[class_id] > 0.25:
        confs.append(conf)
        label = CLASES_YOLO[int(class_id)]
        class_ids.append(label)
        
        # Extraer cajas
        x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
        left = int((x - 0.5 * w) * x_factor)
        top = int((y - 0.5 * h) * y_factor)
        width = int(w * x_factor)
        height = int(h * y_factor)
        box = np.array([left, top, width, height])
        boxes.append(box)

# Aplicar Non-Maximum Suppression (NMS)
r_class_ids, r_confs, r_boxes = [], [], []

indexes = cv2.dnn.NMSBoxes(boxes, confs, SCORE_THRESHOLD, NMS_THRESHOLD)
for i in indexes:
    r_class_ids.append(class_ids[i])
    r_confs.append(confs[i])
    r_boxes.append(boxes[i])

# Dibujar los resultados en la imagen original
for i in indexes:
    box = boxes[i]
    left = box[0]
    top = box[1]
    width = box[2]
    height = box[3]
    cv2.rectangle(image, (left, top), (left + width, top + height), (0, 255, 0), 3)

# Mostrar la imagen con las detecciones
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
Image.fromarray(image_rgb)
