# yolo_traffic_sign
Detection and Recognition Traffic Sign

## Infromation
* Yolo version : Gaussian YOLOv3
* Dataset : BDD Dataset

## Requirements
1. Autoware-AI (vision_darknet_detect & range_vision_fusion)
2. BDD Dataset
3. Camera or Video

## How to work
1. Road Traffic Sign ROI  
![0](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/3c848739-9d2a-4e27-9e00-667863f1d532)
3. Road Template Image  
![00](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/97fe4775-0b81-40dc-b605-2a0abf717644)
![01](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/320fe0e8-24ae-4503-a476-e0b42c497ee5)
![02](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/78b40b4a-eae4-4253-9cb8-b1d4b7fb9e09)
![03](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/6189939f-b191-4562-8f5f-bf0f47de9b2d)
![04](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/e3535ab2-ca2f-489c-a19c-96bff54b4c95)
4. Convert to binary images and separate them into alphabets and numbers  
![binary](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/927aa042-e608-4c2d-8757-f6e220c8f48d)
![boundsAB](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/a16d033f-95d1-49fd-8754-8cfbfdbaf89f)
![boundsOTH](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/5c519dd6-c0ee-432d-8627-e271285d0421)
5. Get the optimal rotation angle image  
![rotAB](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/62811ea4-67e0-4536-896d-a7c867de6e99)
![rotOTH](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/e7f7bad2-2cce-4ba8-a685-4b0f748db8e1)
6. Resize the image to enable template matching  
![templateBig](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/57323d14-31d6-4c67-9349-10756ed729b8)
![templateBigOTH](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/9d155e4c-35c2-467d-8781-6c5f10fdec63)
7. Store matching values if threshold is 0.6 or higher
   
## Result
![Screenshot from 2023-07-06 19-47-32](https://github.com/simfrog/yolo_traffic_sign/assets/31130917/51d00e10-9050-4fce-8e5f-9367ab369b00)  

## How to launch
#### rosrun yolo_traffic_sign yolo_traffic_sign.cpp
