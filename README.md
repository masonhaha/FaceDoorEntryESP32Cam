# FaceDoorEntryESP32Cam
esp32cam
# ESP32-CAM Face Recognition Door Lock
ESP32-CAM 双模式人脸识别智能门禁系统，支持 **WiFi网页远程控制 + 离线单机按键注册识别**，继电器开锁、SD卡存储人脸/开锁记录、NTP北京时间同步、多按键快捷操作。

## 项目特性
### 🔌 硬件功能
1. **双工作模式一键切换**
   - 单机离线模式：断开WiFi，仅按键操作，无需手机/电脑即可识别人脸开锁、本地注册新人脸
   - WiFi在线模式：Web网页实时预览摄像头、远程开锁、网页人脸注册、管理所有开锁记录
2. 硬件按键三独立功能（带消抖）
   - GPIO15：WiFi开关切换（联网/离线一键切换）
   - GPIO16：系统硬件重启
   - GPIO13：离线单机人脸注册（自动命名 User_01~User_XX）
3. 继电器电控锁输出（GPIO4），自定义开锁延时自动落锁
4. WiFi状态指示灯（GPIO33闪光灯）：联网常亮，离线熄灭
5. OV2640摄像头MTCNN人脸检测 + FaceNet特征比对，本地Flash存储人脸库
6. SD_MMC SD卡存储
   - 开锁抓拍人脸照片 / 注册人脸样本图
   - CSV格式化永久保存开锁记录（人脸开锁/手动远程开锁区分）
7. NTP网络北京时间自动同步，记录带精确时间戳
8. 完整Web管理后台
   - 实时视频流WebSocket低延迟传输
   - 人脸检测/识别/采集注册一键操作
   - 已注册人脸列表管理，支持删除单个人脸
   - 独立开锁记录页面：查看、放大抓拍照片、单条/全部记录删除
9. 系统容错设计
   - SD卡缺失自动降级运行（仅无记录存储，识别开锁正常）
   - WiFi连接失败自动切离线模式
   - 时间同步超时使用本地毫秒时间兜底
   - PSRAM自适应，有无PSRAM均可运行

## 硬件清单
| 器件 | 型号/规格 | 引脚定义 |
|------|----------|----------|
| 主控摄像头 | ESP32-CAM（OV2640） | 代码内置标准引脚 |
| 继电器模块 | 5V继电器（电控锁） | 控制引脚 GPIO4 |
| 3路独立按键 | 轻触按键（上拉输入） | WiFi切换GPIO15 / 重启GPIO16 / 离线注册GPIO13 |
| 状态LED | 板载闪光灯 | GPIO33 |
| SD卡 | TF卡（SD_MMC模式） | ESP32-CAM自带卡槽 |
| 电源 | 5V 2A 稳定电源 | 摄像头模块供电 |

## 引脚分配一览
```cpp
// 摄像头OV2640标准引脚
PWDN_GPIO_NUM     32
RESET_GPIO_NUM    -1
XCLK_GPIO_NUM      0
SIOD_GPIO_NUM     26
SIOC_GPIO_NUM     27
Y9_GPIO_NUM       35
Y8_GPIO_NUM       34
Y7_GPIO_NUM       39
Y6_GPIO_NUM       36
Y5_GPIO_NUM       21
Y4_GPIO_NUM       19
Y3_GPIO_NUM       18
Y2_GPIO_NUM        5
VSYNC_GPIO_NUM    25
HREF_GPIO_NUM     23
PCLK_GPIO_NUM     22

// 外设引脚
relay_pin         4    // 开锁继电器
WIFI_LED_PIN      33   // WiFi状态灯
WIFI_TOGGLE_PIN   15   // WiFi开关按键
RESTART_PIN       16   // 重启按键
ENROLL_PIN        13   // 离线人脸注册按键
