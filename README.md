# INS

## 机械编排代码使用说明：

1. INSMechanization类的测试文件mechanization_test.cpp需要修改数据的路径和文件名才能运行。

![image-20210715212324304](README.assets/image-20210715212324304.png)

2. 数据需要按照DataStorage.hpp和mechanization_test.cpp中的对应格式来存放。

![image-20210715212409694](README.assets/image-20210715212409694.png)

![image-20210715212452706](README.assets/image-20210715212452706.png)

## 松组合代码使用说明：

1. GINS类的测试文件GINS_test.cpp需要修改数据的路径和文件名才能运行。

![image-20210715212637641](README.assets/image-20210715212637641.png)

2. 数据需要按照DataStorage.hpp和GINS_test.cpp中的对应格式来存放。

![image-20210715212825417](README.assets/image-20210715212825417.png)

![image-20210715212846952](README.assets/image-20210715212846952.png)

![image-20210715212918001](README.assets/image-20210715212918001.png)

![image-20210715212857641](README.assets/image-20210715212857641.png)

![image-20210715212738786](README.assets/image-20210715212738786.png)

3. 可以在GINS_test.cpp中根据需要选择是否开启GNSS信号中断调试，支持调整初始收敛时间、GNSS信号持续时间、GNSS信号中断配置时间和开始运动时间。

![image-20210715213313489](README.assets/image-20210715213313489.png)

​		运行结束后会输出nav文件。

4. 可以在GINS.cpp中根据需要选择不同的针对GNSS信号时间戳和IMU信号时间戳不重合的内插方法。

![image-20210715213656977](README.assets/image-20210715213656977.png)

