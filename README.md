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
开发环境依赖
1. Arduino IDE 必备库
ESP32 板库（版本1.0.5）
ArduinoWebsockets （WebSocket 视频流）
ESP32 内置自带库：WiFi、esp_camera、esp_http_server、SD_MMC、esp_timer、fd_forward/fr_forward 人脸算法库
2. 板配置关键选项
开发板：ESP32 Wrover Module（带 PSRAM）
PSRAM：启用 OPI PSRAM
Flash 大小：4MB
分区方案：Huge APP (3MB No OTA) 人脸算法占用内存较大
目录结构
plaintext
ESP32-CAM-FaceDoorLock/
├── ESP32-CAM-FaceDoorLock.ino   // 完整主程序代码
├── docs/
│   ├── hardware_wiring.png      // 硬件接线图
│   ├── web_ui_preview.png       // 网页后台界面截图
│   ├── record_page.png          // 开锁记录页面截图
├── README.md                    // 项目说明（本文档）
└── LICENSE                      // MIT开源协议
使用教程
一、基础配置修改
打开 .ino 文件，修改顶部 WiFi 参数为自家路由器：
cpp
运行
const char* ssid = "你的WiFi名称";
const char* password = "WiFi密码";
可自定义参数：
cpp
运行
#define interval 5000        // 开锁保持时长(毫秒)
#define ENROLL_CONFIRM_TIMES 5 // 人脸注册采集样本数量
#define FACE_ID_SAVE_NUMBER 50 // 最大存储人脸数量
二、烧录 & 上电流程
接线完成，插入 TF SD 卡（推荐 8G 以上 FAT32 格式）
Arduino IDE 选择对应 ESP32-Wrover 开发板，开启 PSRAM
编译上传代码至 ESP32-CAM
上电开机流程：
初始化继电器、按键、LED
挂载 SD 卡，自动创建 /photos /FACES 文件夹
摄像头初始化，加载 Flash 内部已存人脸库
自动短暂连接 WiFi 同步北京时间（5 秒超时）
自动关闭 WiFi，进入离线单机识别模式
三、离线单机模式（无 WiFi）
1. 人脸识别开锁
设备持续循环检测摄像头人脸，匹配 Flash 内注册人脸自动触发继电器开锁，SD 卡抓拍保存开锁照片，写入记录 CSV。
2. 离线注册新人脸
短按 GPIO13 注册按键，设备自动命名 User_01、User_02...，连续采集 5 帧人脸样本存入 Flash 与 SD 卡/FACES目录，采集完成自动返回识别模式。
3. 切换联网模式
短按 GPIO15 WiFi 按键，设备连接 WiFi、启动网页服务、WebSocket 视频流，GPIO33 指示灯常亮。
4. 系统重启
短按 GPIO16 重启按键，设备立即硬件复位重启。
四、WiFi 在线网页模式
按键开启 WiFi 后，串口打印设备内网 IP，浏览器直接访问 IP 进入管理主页
网页核心功能：
实时摄像头预览、启停视频流
一键人脸检测 / 自动人脸识别开锁
自定义姓名人脸注册、删除已注册人脸
一键远程手动开锁，生成人工开锁记录
开锁记录页面：访问 http://设备IP/records
分页展示全部开锁记录、区分人脸 / 手动开锁
点击放大抓拍原图、单条删除、一键清空全部记录
WebSocket 端口 82，设备自动推送门锁实时状态至页面
SD 卡文件说明
plaintext
/photos/        每次开锁抓拍的人脸照片（时间戳命名jpg）
/FACES/         人脸注册时采集的样本图片
/records.csv    开锁记录数据库，格式：时间,姓名,开锁类型,照片文件名
/temp.csv       记录删除临时缓存文件（程序自动生成销毁）
串口调试信息
波特率固定 115200，上电、人脸检测、注册、开锁、WiFi 状态、SD 卡操作均有完整日志输出，方便排查硬件 / 网络故障。
常见问题 FAQ
摄像头初始化失败
检查摄像头排线正反面、接线是否松动，供电不足会频繁初始化报错，使用 5V2A 电源。
WiFi 连接不上
仅支持 2.4G WiFi，不支持 5G；核对 SSID / 密码大小写，缩短距离。
SD 卡挂载失败
TF 卡格式化为 FAT32，确认 ESP32-CAM 卡槽接触良好，无 SD 卡仅无法存储记录，识别功能正常。
人脸不识别、识别率低
保证正面充足光源，人脸占画面足够大小；代码min_face = 80可按需调整最小识别人脸尺寸。
内存崩溃 / 重启
必须启用 PSRAM，分区方案选择 Huge APP；降低 JPEG 画质参数，减少画面分辨率。
开源协议
MIT License，可自由修改、商用，保留原项目开源声明。
plaintext

# 二、仓库简介短描述（GitHub仓库主页一句话简介）
ESP32-CAM dual-mode face recognition door lock, support offline button enrollment & WiFi web remote control, relay unlock, SD storage records, NTP time sync.

# 三、仓库标签（GitHub Topics）
esp32, esp32-cam, face-recognition, facenet, mtcnn, smart-door-lock, arduino, websocket, iot, access-control, smart-home

# 四、配套上传建议
1. 仓库新建3个文件夹：`docs`，放入硬件接线图、网页UI截图、记录页面截图；
2. 直接上传完整`.ino`代码文件；
3. 新建`LICENSE`文件写入MIT协议；
4. 复制上方完整README.md作为仓库说明；
5. 仓库简介粘贴短描述，添加全部topics标签。

# 补充：LICENSE MIT 文件内容（单独新建LICENSE）
MIT License
Copyright (c) 2026 ESP32-CAM FaceDoorLock
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
