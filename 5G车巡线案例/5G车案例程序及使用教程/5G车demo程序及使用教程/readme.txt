g++ simple_code.cpp -o simple_code -lpigpio `pkg-config --cflags --libs opencv4`   # 编译 c++  根据实际该文件名
sudo xauth add $(xauth -f ~pi/.Xauthority list|tail -1)  #增加权限
sudo ./simple_code #运行代码

./network-rc/lib/frpc/frpc -c frpc.ini  # 运行networkRC
sudo raspi-config  # 打开设置
cd 进入目录
cd .. 退出已经目录
pwd 查看当前目录
vim 文件名 使用vim编辑文件
http://car156.frp.smartcar.run/
sudo systemctl restart network-rc.service
