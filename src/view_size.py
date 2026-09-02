import cv2

drawing = False
start = None
rect = None
img = None

def mouse_callback(event, x, y, flags, userdata):
    global drawing, start, rect, img
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start = (x, y)
        rect = (x, y, x, y)
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        rect = (start[0], start[1], x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        rect = (start[0], start[1], x, y)
        if img is None:
            return
        h, w = img.shape[:2]
        x1, y1, x2, y2 = rect
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))
        count = (x2 - x1 + 1) * (y2 - y1 + 1)
        print(f'ROI({x1},{y1})-({x2},{y2}) 像素点个数:{count}')

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', mouse_callback)

cm0 = cv2.VideoCapture(0)
cm0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cm0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cm0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cm0.set(cv2.CAP_PROP_FPS, 30)
while True:
    ret, img = cm0.read()
    if not ret:
        continue
    display = img.copy()
    if rect is not None:
        x1, y1, x2, y2 = rect
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.imshow('frame', display)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
print()
cv2.destroyAllWindows()
