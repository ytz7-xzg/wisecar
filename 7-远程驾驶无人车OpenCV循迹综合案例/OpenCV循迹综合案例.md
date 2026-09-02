# OpenCV循迹综合案例

接下来,我们来一步一步完成OpenCV循迹综合案例,我们从处理一张普通的操场跑道图片来举例。

## 案例文件夹的创建

```
cd /home/pi
mkdir Trace
cd Trace
```



## 图片读取

```python
import cv2
import numpy as np

def main(img_path):
    """
    处理图片进行车道检测的主函数。

    参数:
     (str): 图像文件的路径。
    """
    img = cv2.imread(img_path)
    cv2.imshow('result', img)  # 显示帧
    cv2.waitKey(0)


if __name__ == "__main__":
    img_path = '123321.jpg'
    main(img_path)


```

首先我们读取123321.jpg这张图片,然后我们定义一个函数作为我们程序的主逻辑函数.

if __name__ == "__main__": 的解释: 在Python编程语言中，`if __name__ == "__main__":` 这行代码是一个常见的习语，用于判断当前的Python脚本是被作为主程序直接运行，还是被作为模块导入到其他Python脚本中。

具体来说，这里涉及到`__name__`这个内置变量：

- 当一个Python文件（模块）被直接运行时，`__name__`的值会被设置为 `"__main__"`。
- 当一个Python文件（模块）被其他文件导入时，`__name__`的值会被设置为该模块的名字。

## 图像的预处理

```python
import cv2
import numpy as np

def preprocess_frame(frame):
    """
    将帧转换为灰度图像，并使用 Canny 边缘检测。

    参数:
    frame (numpy.ndarray): 视频的原始帧。

    返回:
    edges (numpy.ndarray): 检测到的边缘。
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
    # 中间还可以加一些其他图像预处理步骤...
    
    edges = cv2.Canny(gray, 50, 150)  # 使用 Canny 边缘检测
    return edges
def main(img_path):
    """
    处理图片进行车道检测的主函数。

    参数:
     (str): 图像文件的路径。
    """
    img = cv2.imread(img_path)
    edges = preprocess_frame(img)  # 预处理帧
    cv2.imshow('result', edges)  # 显示帧
    cv2.waitKey(0)


if __name__ == "__main__":
    img_path = '123321.jpg'
    main(img_path)

```

这里,我们使用Canny边缘检测算法检测边缘,由于图片噪点较少,所以不需要做额外处理.



## 截取感兴趣区域(ROI)

```python
import cv2
import numpy as np

def preprocess_frame(frame):
    """
    将帧转换为灰度图像，应用高斯模糊，并使用 Canny 边缘检测。

    参数:
    frame (numpy.ndarray): 视频的原始帧。

    返回:
    edges (numpy.ndarray): 检测到的边缘。
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
    edges = cv2.Canny(gray, 50, 150)  # 使用 Canny 边缘检测
    return edges

def region_of_interest(edges):
    """
    对边缘图像应用掩膜，仅保留感兴趣区域。

    参数:
    edges (numpy.ndarray): 检测到的边缘。

    返回:
    cropped_edges (numpy.ndarray): 掩膜后的边缘图像。
    """
    # 获取图像的高和宽
    height, width = edges.shape
    mask = np.zeros_like(edges)  # 创建一个黑色的掩膜
    # 定义感兴趣区域的多边形
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, height * 0.6),
        (0, height * 0.6),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)  # 使用白色填充多边形
    cropped_edges = cv2.bitwise_and(edges, mask)  # 将掩膜应用到边缘图像
    return cropped_edges
def main(img_path):
    """
    处理图片进行车道检测的主函数。

    参数:
     (str): 图像文件的路径。
    """
    img = cv2.imread(img_path)
    edges = preprocess_frame(img)  # 预处理帧
    cropped_edges = region_of_interest(edges)  # 应用感兴趣区域
    cv2.imshow('result', cropped_edges)  # 显示帧
    cv2.waitKey(0)


if __name__ == "__main__":
    img_path = '123321.jpg'
    main(img_path)
```

由于图像上半部分边缘信息价值较低,所以我们并不需要它,需要将其裁掉,减少干扰.

部分代码解释:

`polygon = np.array([[    (0, height),    (width, height),    (width, height * 0.6),    (0, height * 0.6), ]], np.int32)`

- `polygon` 是一个二维数组，表示一个多边形，每个元素是一个 `(x, y)` 坐标点。

- 在这个例子中，多边形是一个矩形，其底部与图像底部对齐，顶部在图像高度的60%处。只关注图像的下半部分。

- `np.int32` 确保坐标点使用32位整数表示。

  `cv2.fillPoly(mask, polygon, 255)`

- 使用 `cv2.fillPoly` 函数将 `polygon` 定义的多边形填充到 `mask` 中。

- `255` 表示使用白色填充，因为在边缘图像中，白色代表边缘。

  `cropped_edges = cv2.bitwise_and(edges, mask)`

- 使用 `cv2.bitwise_and` 函数将 `mask` 与 `edges` 进行按位与操作。

- 结果是只有 `mask` 中白色区域（即定义的多边形内部）的 `edges` 中的边缘被保留，其他区域都被设置为黑色。



## 检测并绘制直线

```python
import cv2
import numpy as np

def preprocess_frame(frame):
    """
    将帧转换为灰度图像，应用高斯模糊，并使用 Canny 边缘检测。

    参数:
    frame (numpy.ndarray): 视频的原始帧。

    返回:
    edges (numpy.ndarray): 检测到的边缘。
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
    edges = cv2.Canny(gray, 50, 150)  # 使用 Canny 边缘检测
    return edges

def region_of_interest(edges):
    """
    对边缘图像应用掩膜，仅保留感兴趣区域。

    参数:
    edges (numpy.ndarray): 检测到的边缘。

    返回:
    cropped_edges (numpy.ndarray): 掩膜后的边缘图像。
    """
    height, width = edges.shape
    mask = np.zeros_like(edges)  # 创建一个黑色的掩膜
    # 定义感兴趣区域的多边形
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, height * 0.6),
        (0, height * 0.6),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)  # 使用白色填充多边形
    cropped_edges = cv2.bitwise_and(edges, mask)  # 将掩膜应用到边缘图像
    return cropped_edges

def detect_lines(cropped_edges):
    """
    使用霍夫线变换检测掩膜后的边缘图像中的线条。

    参数:
    cropped_edges (numpy.ndarray): 掩膜后的边缘图像。

    返回:
    lines (numpy.ndarray): 检测到的线条。
    """
    lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, maxLineGap=50)
    return lines

def draw_lines(frame, lines):
    """
    在原始帧上绘制检测到的线条。

    参数:
    frame (numpy.ndarray): 视频的原始帧。
    lines (numpy.ndarray): 检测到的线条。

    返回:
    frame (numpy.ndarray): 绘制了线条的帧。
    """
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 != x2 and y1 != y2:  # 确保线条不是垂直的
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)  # 使用绿色绘制每条线条
    return frame

def main(img_path):
    """
    处理图片进行车道检测的主函数。

    参数:
     (str): 图像文件的路径。
    """
    img = cv2.imread(img_path)
    edges = preprocess_frame(img)  # 预处理帧
    cropped_edges = region_of_interest(edges)  # 应用感兴趣区域
    lines = detect_lines(cropped_edges)  # 检测线条
    frame_with_lines = draw_lines(img, lines)  # 在帧上绘制线条
    cv2.imshow('result', frame_with_lines)  # 显示帧
    cv2.waitKey(0)


if __name__ == "__main__":
    img_path = '123321.jpg'
    main(img_path)
```

这里我们使用处理速度更快的概率霍夫直线检测来检测直线,然后为了更加直观,我们在图像画出检测出的直线.

部分代码解释:

**lines**

- lines检测到的线条,其中包含检测到的直线的参数
- 通过下标取出每个检测到的直线的两个端点 **X1 Y1 X2 Y2**
- 然后将直线的两个端点作为判断依据,过滤掉无效直线
- 最后使用`cv2.line`在图像上画出直线

cv2.line函数解释:

```python
cv2.line(image, pt1, pt2, color, thickness=None, lineType=None, shift=None)
```

- `image`: 要在其上绘制直线的图像。这个图像应该是一个二维或三维的 `numpy.ndarray` 对象。
- `pt1`: 直线起点坐标，格式为 `(x1, y1)`。
- `pt2`: 直线终点坐标，格式为 `(x2, y2)`。
- `color`: 直线的颜色。对于灰度图像，它应该是标量灰度值。对于彩色图像，它应该是一个包含BGR（蓝、绿、红）值的元组，例如 `(255, 0, 0)` 表示蓝色。
- `thickness`: 线条的粗细程度。如果设置为 `-1`，则将绘制填充的矩形（实际上是连接起点和终点的实心矩形）。默认值为 `1`。
- `lineType`: 线条的类型。这个参数可以有以下值：
  - `cv2.FILLED`: 填充线条。
  - `cv2.LINE_4`: 4连通线。
  - `cv2.LINE_8`: 8连通线。
  - `cv2.LINE_AA`: 抗锯齿线，看起来更平滑。
- `shift`: 坐标点的小数位数。通过这个参数，我们可以指定点的亚像素精度。这个参数通常与 `lineType` 结合使用，以获得更精确的线条绘制。





这个案例到此就结束了,读者可以尝试自行替换图片,自行摸索图像处理步骤.