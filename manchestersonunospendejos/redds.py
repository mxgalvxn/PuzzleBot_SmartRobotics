#!/usr/bin/env python
import cv2

net = cv2.dnn.readNetFromONNX('/home/puzzlebot/red/src/onnx_inference/scripts/best.onn')
blob = cv2.dnn.blobFromImage(img, 1/255 , (640, 640), swapRB=True, mean=(0,0,0), crop= False)
net.setInput(blob)
outputs= net.forward(net.getUnconnectedOutLayersNames())
for i in range(n_detections):
  detect=out[0][i]
  confidence= detect[4]
  if confidence >= conf_threshold:
    class_score= detect[5:]
    class_id= np.argmax(class_score)
    if (class_score[class_id]> score_threshold):
      score.append(confidence)
      class_ids.append(class_id)
      x, y, w, h = detect[0], detect[1], detect[2], detect[3]
      left= int((x - w/2)* x_scale )
      top= int((y - h/2)*y_scale)
      width = int(w * x_scale)
      height = int( y*y_scale)
      box= np.array([left, top, width, height])
      boxes.append(box)
indices = cv2.dnn.NMSBoxes(boxes, np.array(score), conf_threshold, nms_threshold)
for i in indices:
    box = boxes[i]
    left = box[0]
    top = box[1]
    width = box[2]
    height = box[3] 
    cv2.rectangle(img, (left, top), (left + width, top + height), (0, 0, 255), 3)
    label = "{}:{:.2f}".format(classes[class_ids[i]], score[i])
    text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1)
    dim, baseline = text_size[0], text_size[1]
    cv2.rectangle(img, (left, top), (left + dim[0], top + dim[1] + baseline), (0,0,0), cv2.FILLED)
    cv2.putText(img, label, (left, top + dim[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1, cv2.LINE_AA)
