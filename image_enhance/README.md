# dual_modal_perception

ROS package for dual modal perception (rgbt)
## 简介
   一种快速的图像增强算法
## 安装
 - 建立ROS工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone git@github.com:huashu996/image_enhance.git --recursive
   ```
 - 编译
   cd ..
   catkin_make
   source devel/setup.bash
   roslaunch image_enhance image_enhance.launch
   ```
## 训练测试
 - 制作双模态数据集txt形式
   将数据集images images2 labels 放入data文件夹下，注意名字一样，最后执行命令
   ```Shell  
   python split_train_val.py 
   python voc_label.py 
   python voc_label2.py 




