import cv2
import numpy as np

def mouse_callback(event, x, y, flags, userdata):
    global img
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    h, w = img.shape[:2]
    if x < 0 or y < 0 or x >= w or y >= h:
        return
    pixel_color = img[y, x]
    hsv = cv2.cvtColor(np.array([[pixel_color]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0, 0]
    try:
#        print(f'({y},{x}) HSV:{tuple(int(v) for v in hsv)} ')
        print(f'({y},{x}) HSV:{tuple(int(v) for v in hsv)} AvgV:{avgV} AvgS:{avgS} AvgGray:{avgGray}')
    except NameError:
        print(f'({y},{x}) HSV:{tuple(int(v) for v in hsv)}')

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', mouse_callback)

cm0 = cv2.VideoCapture(0)
cm0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 实际宽度是320
cm0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cm0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cm0.set(cv2.CAP_PROP_FPS, 30)
while True:
    ret, img = cm0.read()
    if not ret:
        continue
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    avgV = int(np.mean(hsv_img[:, :, 2]))
    avgS = int(np.mean(hsv_img[:, :, 1]))
    avgGray = int(np.mean(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)))
    cv2.imshow('frame', img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
print()
cv2.destroyAllWindows()
