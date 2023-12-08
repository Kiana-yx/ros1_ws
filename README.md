# 安全距离监测项目代码汇总

## yolov5

    yolo_ros.py:订阅相机数据，可视化识别结果

### yolo_seg.py：
    
1. 效果图
   
   <div align=center>     <img src="https://pictures-kiana.oss-cn-beijing.aliyuncs.com/img/202312071629309.png" width = "400" alt="202312071629309"/>    </div> 

2. 使用说明
    
    ```
    roscore
    rosbag play -l person_out
    rosrun yolov5 yolo_seg.py 
    rviz
    ```

    订阅话题：/camera/image
    发布话题：/gray_image