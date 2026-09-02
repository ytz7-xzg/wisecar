# YOLO模型训练

本次案例使用模型为YOLOV5-Lite 

github地址为:[ppogg/YOLOv5-Lite: 🍅🍅🍅YOLOv5-Lite: Evolved from yolov5 and the size of model is only 900+kb (int8) and 1.7M (fp16). Reach 15 FPS on the Raspberry Pi 4B~ (github.com)](https://github.com/ppogg/YOLOv5-Lite)

将我们的模型解压到任意地址,解压后复制文件地址

### 如果你的电脑不是英伟达显卡:

我们打开Anaconda Prompt输入:

```
cd 你的模型文件地址
conda create -n yolov5-lite python=3.10
pip install -r requirements.txt
pip install numpy<2
```

注意,如果发现 pip install 安装较慢请输入以下指令进行换源:

```
python -m pip install --upgrade pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

### 如果你的电脑是英伟达显卡,且你想加速模型训练过程又不怕麻烦的人:

打开CMD窗口,输入`nvidia-smi`

![image-20240718174124344](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174124344.png)

可以看到我的CUDA版本为 **12.3**

之后我们打开英伟达 CUDA网址:  https://developer.nvidia.com/cuda-toolkit-archive

选择一个比你显卡驱动低的CUDA版本(这里我演示11.8版本):

![image-20240718174511223](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174511223.png)

![image-20240718174533125](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174533125.png)

![image-20240718174559584](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174559584.png)

下载完后,双击安装:

![image-20240718174629765](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174629765.png)

选择同意,然后选择精简:

![image-20240718174722750](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174722750.png)

**查看是否安装成功**
在命令窗口中输入`nvcc -V` 进行检查

```python
nvcc  -V
```

可以看到我们安装成功!

![image-20240718174821792](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718174821792.png)

安装完CUDA之后我们打开Anaconda Prompt输入:

```
cd 你的模型文件地址
conda create -n yolov5-lite python=3.10

```

注意此时我们需要将requirements.txt里面的有关pytoch的内容全部删掉,避免二次下载

![image-20240719124143134](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240719124143134.png)

注意,如果发现 pip install 安装较慢请输入以下指令进行换源:

```
python -m pip install --upgrade pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip install -r requirements.txt
pip install numpy<2
```

接着,我们去pytorch官网那个里面下载GPU版的pytorch 官网链接:[PyTorch](https://pytorch.org/)
![image-20240718192645098](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718192645098.png)

选择如下选项,复制这段文字运行,等待安装好后,环境配置完成.



## 模型训练

安装完成之后,我们打开Pycharm点击右上角,打开我们的模型文件:

![image-20240718172216813](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718172216813.png)

![image-20240718172236895](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718172236895.png)

打开之后,切换Python环境:

![image-20240718171827962](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718171827962.png)

![image-20240718171846250](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718171846250.png)



切换完环境后,确保准备好数据集后,我们进行下面的步骤:

```
我们设置如下几个核心配置：

--weights v5lite-s.pt

--cfg models/v5Lite-s.yaml

--data data/coco128.yaml

--img-size 320

--batch-size 16

device 0/cpu                        
```



**Yolov5-Lite** 网络模型的训练可以不一定必须使用 **CUDA** 进行加速，但是 **pytorch** 架构等依赖库一定需要满足

讲一下,要修改的地方

**data/coco128.yaml**:

```
# COCO 2017 dataset http://cocodataset.org - first 128 training images
# Train command: python train.py --data coco128.yaml
# Default dataset location is next to /yolov5:
#   /parent_folder
#     /coco128
#     /yolov5


# train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
train: 你的训练集地址  # 128 images
val: 你的测试集地址 # 128 images

# number of classes 你的目标有多少类
nc: 4

# class names  目标标签
names: [ '0','1', '2', '3' ]

```



![image-20240718193243839](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718193243839.png)

最后在控制塔输入:

```
python train.py --data coco128.yaml --cfg v5lite-s.yaml --weights v5lite-s.pt --batch-size 16 --device cpu
```

**其中device选项调节的是选择cpu训练还是GPU**

将训练环境与数据集都搞定之后，就可以点击运行按钮进行 **Yolov5-Lite** 的模型训练了！

训练成功之后，将会在当前目录下的 **run** 文件下的 **trian** 文件下找到 **expx** (x代表数字)，**expx** 则存放了第 **x** 次训练时候的各种数据内容，**包括：**历史最优权重**best_weight**，当前权重**last_weight**，训练结果**result**等等；

![image-20240718193521715](C:\Users\15892\AppData\Roaming\Typora\typora-user-images\image-20240718193521715.png)



# 模型的转换

我们训练好模型后,要将其转换为onnx格式,在控制台输入如下指令

```
python export.py --weights 你的训练完成后的.pt文件的路径 --end2end
```

即可导出模型!



# 常见问题

1.

```
ModuleNotFoundError: No module named 'numpy._core'
```

解决方法:升级numpy库 控制台输入 :  

```
pip install numpy<2
```



2.

```
ModuleNotFoundError: No module named 'requests'
```

解决方法:

```
pip install requests
```

3.

```
AttributeError: module 'numpy' has no attribute 'int'.
`np.int` was a deprecated alias for the builtin `int`. To avoid this error in existing code, use `int` by itself. Doing this will not modify any behavior and is safe. When replacing `np.int`, you may wish to use e.g. `np.int64` or `np.int32` to specify the precision. If you wish to review your current use, check the release note link for additional information.
The aliases was originally deprecated in NumPy 1.20; for more details and guidance see the original release note at:
    https://numpy.org/devdocs/release/1.20.0-notes.html#deprecations. Did you mean: 'inf'?
```

解决方法:

将报错的文件中**np.int**全部修改为 **int**

4.

````
subprocess.CalledProcessError: Command 'git tag' returned non-zero exit status 128.
````

解决方法:

未下载 v5lite-s.pt 模型文件到模型根目录. 找到案例资料中的模型文件移动到目录中即可.

5.

```
RuntimeError: Could not infer dtype of numpy.float32
```

解决方法:

定位报错的文件和语句,**显式指定dtype**：在将NumPy数组转换为Torch张量时，可以显式指定数据类型。 将其修改为:

```
c = torch.tensor(labels[:, 0], dtype=torch.float32)
```

6.

```
RuntimeError: Numpy is not available
```

解决方法: 控制台输入

```
pip install numpy<2
```

