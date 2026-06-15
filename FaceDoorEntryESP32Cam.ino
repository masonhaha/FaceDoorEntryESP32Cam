#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "SD_MMC.h"
#include "time.h"
#include "esp_system.h"

#define TAG "FaceDoorLock"

using namespace websockets;
WebsocketsServer socket_server;
camera_fb_t * fb = NULL;

// 硬件参数
#define relay_pin 4
#define interval 5000        // 开锁保持时长(毫秒)
// ===================== WiFi状态LED定义（板载闪光灯GPIO33） =====================
#define WIFI_LED_PIN 33     
long door_opened_millis = 0;
bool door_unlocked = false;
bool wifi_connected = false; // 区分WiFi在线/单机模式

// ===================== 按键配置（最高优先级） =====================
#define WIFI_TOGGLE_PIN 15     // WiFi切换按键引脚
#define RESTART_PIN    16     // 重启按键引脚
#define ENROLL_PIN     13     // 【新增】单机人脸注册按键引脚
#define DEBOUNCE_DELAY 500     // 按键消抖时间(毫秒)

// WiFi按键消抖变量
unsigned long lastDebounceTime = 0;
int lastButtonState = HIGH;
int buttonState;

// 重启按键消抖变量
unsigned long lastRestartDebounceTime = 0;
int lastRestartButtonState = HIGH;
int restartButtonState;

// 【新增】单机注册按键消抖变量
unsigned long lastEnrollDebounceTime = 0;
int lastEnrollButtonState = HIGH;
int enrollButtonState;
bool standalone_enroll = false; // 单机注册标志位
int standalone_sample_count = 0;// 单机注册采样计数

// WiFi 配置
const char* ssid = "FAST_shihome";
const char* password = "shiyaojun110";

// NTP 时间同步(北京时间)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 8 * 3600;
const int   daylightOffset_sec = 0;

// 人脸库配置
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 50
#define ENROLL_NAME_LEN 32

// OV2640 摄像头引脚定义
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// 全局状态
bool sd_card_available = false;
httpd_handle_t camera_httpd = NULL;

// 函数前置声明
void app_facenet_main();
void app_httpserver_init();
void open_door();
void sendDoorStatus(WebsocketsClient &client);
void writeUnlockRecord(const char* name, const char* type, const char* photo);
void deleteUnlockRecord(const char* photo);
void deleteAllUnlockRecords();
void standaloneFaceRecognition();
void runFaceRecognition();
void handle_message(WebsocketsClient &client, WebsocketsMessage msg);
void getCurrentDateTime(char *dateTimeStr);
void savePhotoToSD(camera_fb_t *fb, char* photoName);
void saveEnrollFaceToSD(camera_fb_t *fb, const char* personName, int sampleNum);
void wifi_connect_only();       // 仅连接WiFi（不开服务器）
void wifi_start();              // 完整启动WiFi（含服务器）
void wifi_stop();               // 关闭WiFi
bool isTimeSynced();            // 判断时间是否同步成功
// 【新增】单机注册函数
void startStandaloneEnroll();   // 开始单机人脸注册
void processStandaloneEnroll(); // 处理单机注册流程

// 图像处理结构体
typedef struct {
    uint8_t *image;
    box_array_t *net_boxes;
    dl_matrix3d_t *face_id;
} http_img_process_result;

// 人脸检测 MTCNN 参数配置
static inline mtmn_config_t app_mtmn_config() {
    mtmn_config_t mtmn_config = {0};
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    return mtmn_config;
}

mtmn_config_t mtmn_config = app_mtmn_config();
face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;
static dl_matrix3du_t *image_matrix = NULL;

// 系统状态机
typedef enum {
    START_STREAM,
    START_DETECT,
    START_RECOGNITION,
    START_ENROLL,
} en_fsm_state;

en_fsm_state g_state = START_RECOGNITION;

// 人脸注册名称缓存
typedef struct {
    char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;
httpd_resp_value st_name;

// ===================== 主页网页HTML =====================
const char* index_ov2640_html_fixed = R"rawliteral(
<!doctype html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32-CAM人脸识别门锁</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            background-color: #f0f0f0;
            margin: 0;
            padding: 20px;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.1);
        }
        .door-status {
            font-size: 22px;
            font-weight: bold;
            padding: 12px;
            border-radius: 10px;
            margin: 15px auto;
            width: 280px;
            color: white;
            background-color: #dc3545;
        }
        .controls {
            display: flex;
            flex-wrap: wrap;
            justify-content: center;
            gap: 10px;
            margin: 20px 0;
        }
        button {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 12px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            transition: all 0.3s ease;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        button:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            transform: none;
        }
        .video-container {
            position: relative;
            display: inline-block;
            margin: 20px auto;
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 4px 20px rgba(0,0,0,0.15);
        }
        #stream {
            display: block;
            width: 100%;
            max-width: 640px;
            height: auto;
        }
        .status {
            margin: 15px 0;
            padding: 10px;
            border-radius: 5px;
            font-weight: bold;
        }
        .status.success {
            background-color: #d4edda;
            color: #155224;
        }
        .status.error {
            background-color: #f8d7da;
            color: #721c24;
        }
        .status.info {
            background-color: #d1ecf1;
            color: #0c5460;
        }
        .face-list {
            background: white;
            border-radius: 10px;
            padding: 20px;
            margin: 20px auto;
            max-width: 640px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        .face-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px;
            border-bottom: 1px solid #eee;
        }
        .face-item:last-child {
            border-bottom: none;
        }
        .enroll-form {
            display: none;
            background: white;
            border-radius: 10px;
            padding: 20px;
            margin: 20px auto;
            max-width: 400px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        input[type="text"] {
            width: 100%;
            padding: 10px;
            margin: 10px 0;
            border: 1px solid #ddd;
            border-radius: 5px;
            box-sizing: border-box;
        }
        .connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 8px 15px;
            border-radius: 20px;
            font-size: 14px;
            font-weight: bold;
        }
        .connected {
            background-color: #d4edda;
            color: #155224;
        }
        .disconnected {
            background-color: #f8d7da;
            color: #721c24;
        }
        @media (max-width: 768px) {
            .controls {
                flex-direction: column;
                align-items: center;
            }
            button {
                width: 100%;
                max-width: 300px;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>📷 ESP32-CAM人脸识别门锁系统</h1>
        <p>智能门禁 | 人脸识别 | 自动开锁</p>
    </div>
    
    <div class="door-status" id="doorStatus">🔒 门锁状态：已锁定</div>
    
    <div class="connection-status disconnected" id="connectionStatus">未连接</div>
    
    <div class="controls">
        <button id="startStream" onclick="startStreaming()">▶️ 开始预览</button>
        <button id="stopStream" onclick="stopStreaming()" disabled>⏹️ 停止预览</button>
        <button id="startDetect" onclick="startDetection()">🔍 人脸检测</button>
        <button id="startRecog" onclick="startRecognition()">👁️ 人脸识别</button>
        <button id="enrollBtn" onclick="toggleEnrollForm()">👤 注册人脸</button>
        <button onclick="window.location.href='/records'">📜 开锁记录</button>
        <button onclick="unlockDoor()">🔑 立即开锁</button>
    </div>
    
    <div id="statusDiv" class="status info" style="display: none;"></div>
    
    <div class="video-container">
        <img id="stream" src="" alt="Camera Stream">
    </div>
    
    <div id="enrollForm" class="enroll-form">
        <h3>👤 注册新人脸</h3>
        <input type="text" id="personName" placeholder="请输入姓名" maxlength="32">
        <button onclick="captureFace()">📸 开始采集</button>
        <button onclick="toggleEnrollForm()">❌ 取消</button>
        <div id="enrollStatus" class="status info" style="margin-top: 10px; display: none;"></div>
    </div>
    
    <div class="face-list">
        <h3>📋 已注册人脸列表</h3>
        <div id="faceListContainer">
            <p>暂无注册人脸</p>
        </div>
        <button onclick="loadFaceList()" style="margin-top: 10px;">🔄 刷新列表</button>
    </div>

    <script>
        const streamEndpoint = 'ws://' + window.location.hostname + ':82';
        let websocket = null;
        let streamActive = false;
        let streamElement = document.getElementById('stream');
        let statusDiv = document.getElementById('statusDiv');
        let connectionStatus = document.getElementById('connectionStatus');
        let doorStatus = document.getElementById('doorStatus');
        
        function showStatus(message, type = 'info') {
            statusDiv.textContent = message;
            statusDiv.className = `status ${type}`;
            statusDiv.style.display = 'block';
            setTimeout(() => {
                statusDiv.style.display = 'none';
            }, 3000);
        }
        
        function updateDoorStatus(status) {
            if (status === "unlocked") {
                doorStatus.textContent = "🔓 门锁状态：已解锁";
                doorStatus.style.backgroundColor = "#28a745";
            } else {
                doorStatus.textContent = "🔒 门锁状态：已锁定";
                doorStatus.style.backgroundColor = "#dc3545";
            }
        }
        
        function updateConnectionStatus(connected) {
            if (connected) {
                connectionStatus.textContent = '已连接';
                connectionStatus.className = 'connection-status connected';
            } else {
                connectionStatus.textContent = '未连接';
                connectionStatus.className = 'connection-status disconnected';
            }
        }
        
        function connectWebSocket() {
            websocket = new WebSocket(streamEndpoint);
            
            websocket.onopen = function(event) {
                console.log('WebSocket连接已建立');
                updateConnectionStatus(true);
                loadFaceList();
            };
            
            websocket.onmessage = function(event) {
                if(typeof event.data === 'string') {
                    if (event.data === "door_status:locked" || event.data === "door_status:unlocked") {
                        let s = event.data.split(":")[1];
                        updateDoorStatus(s);
                        return;
                    }
                    handleCommandMessage(event.data);
                } else {
                    handleImageMessage(event.data);
                }
            };
            
            websocket.onclose = function(event) {
                console.log('WebSocket连接已关闭');
                updateConnectionStatus(false);
                if(streamActive) {
                    setTimeout(connectWebSocket, 1000);
                }
            };
            
            websocket.onerror = function(error) {
                console.error('WebSocket错误:', error);
                updateConnectionStatus(false);
            };
        }
        
        function handleCommandMessage(data) {
            console.log('收到命令:', data);
            
            if(data === 'STREAMING') {
                startStreamDisplay();
            } else if(data === 'DETECTING') {
                showStatus('正在检测人脸...', 'info');
            } else if(data === 'RECOGNISING') {
                showStatus('正在识别人脸...', 'info');
            } else if(data === 'FACE DETECTED') {
                showStatus('检测到人脸!', 'success');
            } else if(data === 'FACE NOT RECOGNISED') {
                showStatus('未识别到已注册人脸', 'error');
            } else if(data.startsWith('SAMPLE NUMBER')) {
                document.getElementById('enrollStatus').textContent = data;
                document.getElementById('enrollStatus').style.display = 'block';
            } else if(data.startsWith('FACE CAPTURED FOR')) {
                showStatus(data, 'success');
                toggleEnrollForm();
                loadFaceList();
            } else if(data.startsWith('listface:')) {
                addFaceToList(data.substring(9));
            } else if(data === 'delete_faces') {
                clearFaceList();
            } else if(data === 'door_open') {
                showStatus('门已打开!', 'success');
            } else if(data === 'CAPTURING') {
                showStatus('开始采集人脸数据...', 'info');
            } else if(data === 'NO FACE DETECTED') {
            } else {
                console.log('未知命令:', data);
            }
        }
        
        function handleImageMessage(data) {
            if(streamActive) {
                var blob = new Blob([data], {type: 'image/jpeg'});
                var url = URL.createObjectURL(blob);
                var oldSrc = streamElement.src;
                streamElement.src = url;
                if(oldSrc && oldSrc.startsWith('blob:')) {
                    URL.revokeObjectURL(oldSrc);
                }
            }
        }
        
        function startStreamDisplay() {
            streamActive = true;
            document.getElementById('startStream').disabled = true;
            document.getElementById('stopStream').disabled = false;
            showStatus('视频流已启动', 'success');
        }
        
        function startStreaming() {
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('stream');
            } else {
                showStatus('WebSocket未连接', 'error');
            }
        }
        
        function stopStreaming() {
            streamActive = false;
            streamElement.src = '';
            document.getElementById('startStream').disabled = false;
            document.getElementById('stopStream').disabled = true;
            showStatus('视频流已停止', 'info');
        }
        
        function startDetection() {
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('detect');
            } else {
                showStatus('WebSocket未连接', 'error');
            }
        }
        
        function startRecognition() {
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('recognise');
            } else {
                showStatus('WebSocket未连接', 'error');
            }
        }
        
        function toggleEnrollForm() {
            const form = document.getElementById('enrollForm');
            form.style.display = form.style.display === 'block' ? 'none' : 'block';
            if(form.style.display === 'block') {
                document.getElementById('personName').focus();
            }
        }
        
        function captureFace() {
            const name = document.getElementById('personName').value.trim();
            if(!name) {
                showStatus('请输入姓名', 'error');
                return;
            }
            
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('capture:' + name);
                document.getElementById('enrollStatus').textContent = '正在采集人脸数据...';
                document.getElementById('enrollStatus').style.display = 'block';
            } else {
                showStatus('WebSocket未连接', 'error');
            }
        }
        
        function addFaceToList(name) {
            const container = document.getElementById('faceListContainer');
            const existingItem = document.querySelector(`.face-item[data-name="${name}"]`);
            
            if(!existingItem) {
                const item = document.createElement('div');
                item.className = 'face-item';
                item.setAttribute('data-name', name);
                item.innerHTML = `
                    <span>${name}</span>
                    <button onclick="removeFace('${name}')" style="background: #dc3545; padding: 5px 10px;">删除</button>
                `;
                if(container.firstChild.textContent === '暂无注册人脸') {
                    container.innerHTML = '';
                }
                container.appendChild(item);
            }
        }
        
        function removeFace(name) {
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('remove:' + name);
            } else {
                showStatus('WebSocket未连接', 'error');
            }
        }
        
        function clearFaceList() {
            document.getElementById('faceListContainer').innerHTML = '<p>暂无注册人脸</p>';
        }
        
        function loadFaceList() {
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('list');
            }
        }

        function unlockDoor() {
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send('unlock');
                showStatus('已手动开锁！', 'success');
            } else {
                showStatus('WebSocket未连接', 'error');
            }
        }
        
        window.onload = function() {
            connectWebSocket();
        };
        
        window.onbeforeunload = function() {
            if(websocket) {
                websocket.close();
            }
        };
    </script>
</body>
</html>
)rawliteral";

// ===================== 开锁记录页面HTML =====================
const char* record_manager_html = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-CAM人脸识别门锁 - 开锁记录</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
        }
        .container {
            max-width: 1000px;
            margin: 0 auto;
            background: white;
            border-radius: 10px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            overflow: hidden;
        }
        .header {
            background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
            color: white;
            padding: 20px;
            text-align: center;
        }
        .header h1 {
            margin: 0;
            font-size: 2em;
        }
        .controls {
            padding: 20px;
            background: #f8f9fa;
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            align-items: center;
            justify-content: center;
        }
        button {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            transition: all 0.3s ease;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        button:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            transform: none;
        }
        .status {
            padding: 10px 20px;
            margin: 10px 0;
            border-radius: 5px;
            text-align: center;
            font-weight: bold;
        }
        .status.success {
            background: #d4edda;
            color: #155224;
        }
        .status.error {
            background: #f8d7da;
            color: #721c24;
        }
        .status.info {
            background: #d1ecf1;
            color: #0c5460;
        }
        .record-list {
            padding: 0;
            margin: 0;
            list-style: none;
        }
        .record-item {
            display: flex;
            align-items: center;
            padding: 12px 15px;
            border-bottom: 1px solid #eee;
            transition: background 0.2s;
            gap: 15px;
        }
        .record-item:hover {
            background-color: #f8f9fa;
        }
        .record-thumb {
            width: 60px;
            height: 60px;
            object-fit: cover;
            border-radius: 8px;
            border: 1px solid #ddd;
        }
        .record-info {
            flex: 1;
            text-align: left;
        }
        .record-time {
            font-size: 16px;
            font-weight: 600;
            color: #333;
            margin-bottom: 4px;
        }
        .record-user {
            font-size: 14px;
            color: #666;
        }
        .record-type {
            padding: 3px 8px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
        }
        .type-face {
            background: #d1ecf1;
            color: #0c5460;
        }
        .type-manual {
            background: #fff3cd;
            color: #856404;
        }
        .record-actions {
            display: flex;
            gap: 8px;
        }
        .btn-small {
            padding: 6px 12px;
            font-size: 12px;
            white-space: nowrap;
        }
        .loading {
            text-align: center;
            padding: 40px;
            font-size: 18px;
        }
        .empty-state {
            text-align: center;
            padding: 60px 20px;
            color: #666;
        }
        .stats {
            display: flex;
            justify-content: space-around;
            padding: 15px;
            background: #f8f9fa;
            border-top: 1px solid #eee;
        }
        .stat-item {
            text-align: center;
        }
        .stat-number {
            font-size: 1.5em;
            font-weight: bold;
            color: #4facfe;
        }
        .modal {
            display: none;
            position: fixed;
            z-index: 1000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            background-color: rgba(0,0,0,0.8);
        }
        .modal-content {
            margin: 5% auto;
            display: block;
            max-width: 90%;
            max-height: 80%;
        }
        .close {
            position: absolute;
            top: 15px;
            right: 35px;
            color: white;
            font-size: 40px;
            font-weight: bold;
            cursor: pointer;
        }
        @media (max-width: 768px) {
            .record-item {
                flex-direction: column;
                align-items: flex-start;
                gap: 10px;
            }
            .record-actions {
                width: 100%;
                justify-content: flex-end;
            }
            .controls {
                flex-direction: column;
                align-items: stretch;
            }
            button {
                width: 100%;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>📷 ESP32-CAM 人脸识别门锁</h1>
            <p>开锁记录管理</p>
        </div>
        <div class="controls">
            <button id="refreshBtn">🔄 刷新记录</button>
            <button id="deleteAllBtn">🗑️ 清空所有记录</button>
            <button onclick="window.location.href='/'">🏠 返回主页</button>
            <span id="ipDisplay"></span>
        </div>
        <div id="status" class="status info" style="display: none;"></div>
        <div id="content">
            <div class="loading" id="loading">🔄 正在加载开锁记录...</div>
            <ul id="recordList" class="record-list"></ul>
        </div>
        <div class="stats">
            <div class="stat-item">
                <div class="stat-number" id="totalRecords">0</div>
                <div>总开锁次数</div>
            </div>
            <div class="stat-item">
                <div class="stat-number" id="faceCount">0</div>
                <div>人脸开锁</div>
            </div>
            <div class="stat-number" id="lastTime">--:--:--</div>
        </div>
    </div>
    <div id="imageModal" class="modal">
        <span class="close">&times;</span>
        <img class="modal-content" id="modalImage">
    </div>

    <script>
        let ip = window.location.hostname;
        let records = [];

        function showStatus(message, type = 'info') {
            const statusDiv = document.getElementById('status');
            statusDiv.textContent = message;
            statusDiv.className = `status ${type}`;
            statusDiv.style.display = 'block';
            setTimeout(() => { statusDiv.style.display = 'none'; }, 3000);
        }

        function updateStats() {
            const total = records.length;
            const face = records.filter(r => r.type === "人脸开锁").length;
            document.getElementById('totalRecords').textContent = total;
            document.getElementById('faceCount').textContent = face;
            document.getElementById('lastTime').textContent = total > 0 ? records[0].time : "--:--:--";
        }

        async function loadRecords() {
            try {
                document.getElementById('loading').style.display = 'block';
                document.getElementById('recordList').innerHTML = '';
                
                const res = await fetch(`/listrecords`);
                if (!res.ok) throw new Error("加载失败");
                records = await res.json();
                
                if (records.length === 0) {
                    document.getElementById('loading').style.display = 'none';
                    document.getElementById('recordList').innerHTML = `
                        <div class="empty-state">
                            <div>📜</div>
                            <h3>暂无开锁记录</h3>
                        </div>`;
                    updateStats();
                    return;
                }

                const list = document.getElementById('recordList');
                list.innerHTML = '';
                records.forEach(item => {
                    const li = document.createElement('li');
                    li.className = 'record-item';
                    const thumb = item.photo ? `/getfile?filename=${item.photo}` : 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';
                    const typeClass = item.type === "人脸开锁" ? "type-face" : "type-manual";
                    
                    li.innerHTML = `
                        <img src="${thumb}" class="record-thumb" alt="照片">
                        <div class="record-info">
                            <div class="record-time">${item.time}</div>
                            <div class="record-user">${item.name}</div>
                        </div>
                        <span class="record-type ${typeClass}">${item.type}</span>
                        <div class="record-actions">
                            <button class="btn-small" onclick="showImage('${item.photo}')">👁️ 查看</button>
                            <button class="btn-small" style="background: #dc3545;" onclick="deleteRecord('${item.photo || ''}')">🗑️ 删除</button>
                        </div>
                    `;
                    list.appendChild(li);
                });
                document.getElementById('loading').style.display = 'none';
                updateStats();
            } catch (e) {
                showStatus("加载记录失败：" + e.message, "error");
                document.getElementById('loading').style.display = 'none';
            }
        }

        function showImage(filename) {
            const modal = document.getElementById('imageModal');
            const img = document.getElementById('modalImage');
            img.src = `/getfile?filename=${filename}`;
            modal.style.display = 'block';
        }

        function deleteRecord(photo) {
            if (!confirm("确定删除该条记录？")) return;
            const ws = new WebSocket(`ws://${ip}:82`);
            ws.onopen = () => ws.send(`delete_record:${photo}`);
            ws.onmessage = (e) => {
                if (e.data === "record_deleted") {
                    showStatus("删除成功", "success");
                    setTimeout(loadRecords, 800);
                } else {
                    showStatus("删除失败", "error");
                }
                ws.close();
            };
        }

        function deleteAllRecords() {
            if (!confirm("确定清空所有开锁记录？不可恢复！")) return;
            const ws = new WebSocket(`ws://${ip}:82`);
            ws.onopen = () => ws.send("delete_all_records");
            ws.onmessage = (e) => {
                if (e.data === "all_records_deleted") {
                    showStatus("清空成功", "success");
                    setTimeout(loadRecords, 800);
                }
                ws.close();
            };
        }

        document.querySelector('.close').onclick = () => {
            document.getElementById('imageModal').style.display = 'none';
        };
        window.onclick = (e) => {
            const modal = document.getElementById('imageModal');
            if (e.target === modal) modal.style.display = 'none';
        }

        window.onload = () => {
            document.getElementById('ipDisplay').textContent = `设备IP: ${ip}`;
            document.getElementById('refreshBtn').onclick = loadRecords;
            document.getElementById('deleteAllBtn').onclick = deleteAllRecords;
            loadRecords();
            setInterval(loadRecords, 30000);
        };
    </script>
</body>
</html>
)rawliteral";

// ===================== HTTP 接口处理函数 =====================
static esp_err_t options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// JSON转义函数
String jsonEscape(const String &str) {
    String escaped;
    for (char c : str) {
        switch (c) {
            case '"':  escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\n': escaped += "\\n";  break;
            case '\r': escaped += "\\r";  break;
            case '\t': escaped += "\\t";  break;
            default:   escaped += c;      break;
        }
    }
    return escaped;
}

// 开锁记录读取接口
static esp_err_t list_records_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    String json = "[";
    File file = SD_MMC.open("/records.csv", FILE_READ);
    
    if (file) {
        bool first = true;
        char line[256];
        
        while (file.available()) {
            file.readBytesUntil('\n', line, sizeof(line)-1);
            line[sizeof(line)-1] = '\0';
            
            size_t len = strlen(line);
            if (len < 8) continue;

            char time[64] = {0};
            char name[64] = {0};
            char type[32] = {0};
            char photo[128] = {0};
            
            if (sscanf(line, "%63[^,],%63[^,],%31[^,],%127[^\n]", time, name, type, photo) == 4) {
                if (!first) json += ",";
                first = false;
                
                json += "{\"time\":\"" + jsonEscape(time) +
                        "\",\"name\":\"" + jsonEscape(name) +
                        "\",\"type\":\"" + jsonEscape(type) +
                        "\",\"photo\":\"" + jsonEscape(photo) + "\"}";
            }
        }
        file.close();
    }
    
    json += "]";
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
}

static esp_err_t get_file_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char filename[64] = {0};
    
    if (httpd_req_get_url_query_str(req, filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "参数错误");
        return ESP_OK;
    }

    char fname[64] = {0};
    char *param = strstr(filename, "filename=");
    if (!param) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "文件不存在");
        return ESP_OK;
    }
    sscanf(param + 9, "%[^&]", fname);

    String filePath = "/photos/" + String(fname);
    File file = SD_MMC.open(filePath.c_str());
    if (!file || file.isDirectory()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "文件不存在");
        file.close();
        return ESP_OK;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    uint8_t buffer[1024];
    while (file.available()) {
        int len = file.read(buffer, sizeof(buffer));
        httpd_resp_send_chunk(req, (char*)buffer, len);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    file.close();
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    return httpd_resp_send(req, index_ov2640_html_fixed, strlen(index_ov2640_html_fixed));
}

static esp_err_t record_manager_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    return httpd_resp_send(req, record_manager_html, strlen(record_manager_html));
}

void app_httpserver_init() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7; 
    
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        Serial.println("HTTP Server Started");
        httpd_uri_t index_uri = {"/", HTTP_GET, index_handler, NULL};
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_uri_t record_uri = {"/records", HTTP_GET, record_manager_handler, NULL};
        httpd_register_uri_handler(camera_httpd, &record_uri);
        httpd_uri_t list_uri = {"/listrecords", HTTP_GET, list_records_handler, NULL};
        httpd_register_uri_handler(camera_httpd, &list_uri);
        httpd_uri_t list_opt_uri = {"/listrecords", HTTP_OPTIONS, options_handler, NULL};
        httpd_register_uri_handler(camera_httpd, &list_opt_uri);
        httpd_uri_t getfile_uri = {"/getfile", HTTP_GET, get_file_handler, NULL};
        httpd_register_uri_handler(camera_httpd, &getfile_uri);
    }
}

// ===================== 人脸库初始化 =====================
void app_facenet_main() {
    face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    if(!aligned_face) aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!image_matrix) image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
    read_face_id_from_flash_with_name(&st_face_list);
    Serial.printf("✅ 已加载注册人脸数量：%d\n", st_face_list.count);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) {
    ESP_LOGD(TAG, "START ENROLLING");
    int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
    ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d", st_name.enroll_name, ENROLL_CONFIRM_TIMES - left_sample_face);
    return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client) {
    client.send("delete_faces");
    face_id_node *head = st_face_list.head;
    char add_face[64];
    for (int i = 0; i < st_face_list.count; i++) {
        sprintf(add_face, "listface:%s", head->id_name);
        client.send(add_face);
        head = head->next;
    }
    return ESP_OK;
}

// ===================== 记录 & 文件操作 =====================
void writeUnlockRecord(const char* name, const char* type, const char* photo) {
    if (!sd_card_available) return;
    File file = SD_MMC.open("/records.csv", FILE_APPEND);
    if (file) {
        char timeStr[32];
        getCurrentDateTime(timeStr);
        file.printf("%s,%s,%s,%s\n", timeStr, name, type, photo);
        file.close();
        Serial.printf("📝 记录开锁：%s | %s | %s\n", timeStr, name, type);
    }
}

void deleteUnlockRecord(const char* photo) {
    if (!sd_card_available || strlen(photo) < 5) return;
    
    String photoPath = "/photos/" + String(photo);
    SD_MMC.remove(photoPath.c_str());
    Serial.printf("🗑️ 删除照片：%s\n", photoPath.c_str());

    File oldFile = SD_MMC.open("/records.csv", FILE_READ);
    if (!oldFile) return;
    
    File newFile = SD_MMC.open("/temp.csv", FILE_WRITE);
    if (!newFile) {
        oldFile.close();
        return;
    }

    char line[128];
    while (oldFile.available()) {
        oldFile.readBytesUntil('\n', line, sizeof(line));
        if (strstr(line, photo) == NULL && strlen(line) > 3) {
            newFile.println(line);
        }
    }

    oldFile.close();
    newFile.close();

    SD_MMC.remove("/records.csv");
    SD_MMC.rename("/temp.csv", "/records.csv");
    Serial.println("✅ 记录删除成功");
}

void deleteAllUnlockRecords() {
    if (!sd_card_available) return;
    
    File dir = SD_MMC.open("/photos");
    if(dir && dir.isDirectory()){
        File file = dir.openNextFile();
        while (file) {
            if(!file.isDirectory()){
                String path = "/photos/" + String(file.name());
                SD_MMC.remove(path.c_str());
            }
            file = dir.openNextFile();
        }
        dir.close();
    }
    
    SD_MMC.remove("/records.csv");
    Serial.println("🗑️ 所有记录+照片已清空");
}

void sendDoorStatus(WebsocketsClient &client) {
    client.send(door_unlocked ? "door_status:unlocked" : "door_status:locked");
}

// 开锁核心函数
void open_door() {
    if(door_unlocked) return;
    door_unlocked = true;
    digitalWrite(relay_pin, HIGH);
    Serial.println("🔓 继电器触发，门已解锁");
    door_opened_millis = millis();
}

void getCurrentDateTime(char *dateTimeStr) {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        sprintf(dateTimeStr, "20250101_000000");
        return;
    }
    sprintf(dateTimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void savePhotoToSD(camera_fb_t *fb, char* photoName) {
    if (!sd_card_available || !fb) return;
    if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");
    
    struct tm timeinfo;
    char timeStr[32];
    if(getLocalTime(&timeinfo)){
        sprintf(timeStr, "%04d%02d%02d_%02d%02d%02d",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        sprintf(timeStr, "unknown_%lu", millis());
    }
    
    sprintf(photoName, "%s.jpg", timeStr);
    String filePath = "/photos/" + String(photoName);
    
    File file = SD_MMC.open(filePath.c_str(), FILE_WRITE);
    if (file) {
        file.write(fb->buf, fb->len);
        file.close();
        Serial.printf("✅ 照片保存成功：%s\n", filePath.c_str());
    } else {
        Serial.println("❌ 照片保存失败");
    }
}

void saveEnrollFaceToSD(camera_fb_t *fb, const char* personName, int sampleNum) {
    if (!sd_card_available) return;
    if (!SD_MMC.exists("/FACES")) SD_MMC.mkdir("/FACES");
    char filename[64];
    sprintf(filename, "/FACES/%s_%d.jpg", personName, sampleNum);
    File file = SD_MMC.open(filename, FILE_WRITE);
    if (file) { 
        file.write(fb->buf, fb->len); 
        file.close(); 
    }
}

// ===================== 网页版人脸识别（WiFi在线）====================
void runFaceRecognition() {
    if(g_state != START_RECOGNITION || door_unlocked) return;

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("⚠️ 网页识别：获取摄像头帧失败");
        return;
    }

    http_img_process_result out_res = {0};
    out_res.image = image_matrix->item;
    fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);
    out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

    if (out_res.net_boxes) {
        Serial.println("👤 网页识别：检测到人脸");
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {
            out_res.face_id = get_face_id(aligned_face);
            if (st_face_list.count > 0) {
                face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
                if (f) {
                    Serial.printf("✅ 网页识别：匹配到用户 %s，执行开锁\n", f->id_name);
                    open_door();
                    char photo[64];
                    savePhotoToSD(fb, photo);
                    writeUnlockRecord(f->id_name, "人脸开锁", photo);
                } else {
                    Serial.println("❌ 网页识别：人脸不匹配");
                }
            } else {
                Serial.println("⚠️ 网页识别：无注册人脸");
            }
            dl_matrix3d_free(out_res.face_id);
        }
    }

    esp_camera_fb_return(fb);
}

// ===================== 单机版人脸识别（WiFi断开）====================
void standaloneFaceRecognition() {
    if(g_state != START_RECOGNITION || door_unlocked) return;

    camera_fb_t *fb_local = esp_camera_fb_get();
    if (!fb_local) {
        Serial.println("⚠️ 单机识别：获取摄像头帧失败");
        return;
    }

    fmt2rgb888(fb_local->buf, fb_local->len, fb_local->format, image_matrix->item);
    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

    if (net_boxes) {
        Serial.println("👤 单机识别：检测到人脸");
        if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
            dl_matrix3d_t *face_id = get_face_id(aligned_face);
            if (st_face_list.count > 0) {
                face_id_node *match_face = recognize_face_with_name(&st_face_list, face_id);
                if (match_face) {
                    Serial.printf("✅ 单机识别：匹配到用户 %s，执行开锁\n", match_face->id_name);
                    open_door();
                    char photo[64];
                    savePhotoToSD(fb_local, photo);
                    writeUnlockRecord(match_face->id_name, "人脸开锁", photo);
                } else {
                    Serial.println("❌ 单机识别：人脸不匹配");
                }
            } else {
                Serial.println("⚠️ 单机识别：无注册人脸");
            }
            dl_matrix3d_free(face_id);
        }
    }

    esp_camera_fb_return(fb_local);
}

// 【新增】开始单机人脸注册（自动生成姓名）
void startStandaloneEnroll() {
    if (g_state == START_ENROLL || door_unlocked) return;
    
    // 自动生成姓名 User_01, User_02...
    sprintf(st_name.enroll_name, "User_%02d", st_face_list.count + 1);
    standalone_enroll = true;
    standalone_sample_count = 0;
    g_state = START_ENROLL;
    
    Serial.println("\n================================");
    Serial.printf("📝 开始单机人脸注册 | 姓名：%s\n", st_name.enroll_name);
    Serial.println("请正对摄像头，系统将自动采集5张人脸样本");
    Serial.println("================================");
}

// 【新增】处理单机注册流程
void processStandaloneEnroll() {
    if (!standalone_enroll || g_state != START_ENROLL) return;

    camera_fb_t *fb_local = esp_camera_fb_get();
    if (!fb_local) return;

    // 人脸检测+对齐
    fmt2rgb888(fb_local->buf, fb_local->len, fb_local->format, image_matrix->item);
    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

    if (net_boxes) {
        if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
            dl_matrix3d_t *face_id = get_face_id(aligned_face);
            // 采集样本
            int left_sample = do_enrollment(&st_face_list, face_id);
            standalone_sample_count = ENROLL_CONFIRM_TIMES - left_sample;
            
            Serial.printf("✅ 采集样本：%d/5\n", standalone_sample_count);
            saveEnrollFaceToSD(fb_local, st_name.enroll_name, standalone_sample_count);

            // 采集完成
            if (left_sample == 0) {
                Serial.println("\n================================");
                Serial.printf("🎉 人脸注册完成！姓名：%s\n", st_name.enroll_name);
                Serial.println("================================");
                
                // 重置状态
                standalone_enroll = false;
                g_state = START_RECOGNITION;
                read_face_id_from_flash_with_name(&st_face_list); // 重新加载人脸库
            }
            dl_matrix3d_free(face_id);
        }
    } else {
        Serial.println("⚠️ 未检测到人脸，请调整位置");
    }

    esp_camera_fb_return(fb_local);
    delay(500); // 采样间隔
}

// ===================== WebSocket 消息处理 =====================
void handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
    String data = msg.data();
    
    if (data == "unlock") {
        open_door();
        writeUnlockRecord("管理员", "人工开锁", "");
        sendDoorStatus(client);
        client.send("door_open");
        return;
    }

    if (data == "stream") { g_state = START_STREAM; client.send("STREAMING"); }
    if (data == "detect") { g_state = START_DETECT; client.send("DETECTING"); }
    if (data == "recognise") { g_state = START_RECOGNITION; client.send("RECOGNISING"); }

    if (data.startsWith("capture:")) {
        g_state = START_ENROLL;
        data.substring(8).toCharArray(st_name.enroll_name, ENROLL_NAME_LEN);
        client.send("CAPTURING");
    }

    if (data.startsWith("remove:")) {
        char person[ENROLL_NAME_LEN];
        data.substring(7).toCharArray(person, ENROLL_NAME_LEN);
        delete_face_id_in_flash_with_name(&st_face_list, person);
        send_face_list(client);
    }

    if (data.startsWith("delete_record:")) {
        char photo[64] = {0};
        data.substring(13).toCharArray(photo, 63);
        deleteUnlockRecord(photo);
        client.send("record_deleted");
    }

    if (data == "delete_all_records") {
        deleteAllUnlockRecords();
        client.send("all_records_deleted");
    }

    if (data == "list") { 
        send_face_list(client); 
        sendDoorStatus(client); 
    }
}

// ===================== WiFi控制函数 =====================
void wifi_connect_only() {
    if(wifi_connected) return;
    
    Serial.println("\n📶 开机自动启动WiFi（仅连接，不开服务器）...");
    WiFi.begin(ssid, password);
    int wifiRetry = 0;
    while (WiFi.status() != WL_CONNECTED && wifiRetry < 30) {
        delay(50);
        Serial.print(".");
        wifiRetry++;
    }
    
    if(WiFi.status() == WL_CONNECTED){
        wifi_connected = true;
        digitalWrite(WIFI_LED_PIN, HIGH);
        Serial.println("\n✅ WiFi 连接成功！IP: " + WiFi.localIP().toString());
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    }else{
        wifi_connected = false;
        digitalWrite(WIFI_LED_PIN, LOW);
        Serial.println("\n❌ WiFi 连接失败");
    }
}

void wifi_start() {
    if(wifi_connected) return;
    
    Serial.println("\n📶 手动启动WiFi + 服务...");
    WiFi.begin(ssid, password);
    int wifiRetry = 0;
    while (WiFi.status() != WL_CONNECTED && wifiRetry < 20) {
        delay(50);
        Serial.print(".");
        wifiRetry++;
    }
    
    if(WiFi.status() == WL_CONNECTED){
        wifi_connected = true;
        digitalWrite(WIFI_LED_PIN, HIGH);
        Serial.println("\n✅ WiFi 连接成功！IP: " + WiFi.localIP().toString());
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        app_httpserver_init();
        socket_server.listen(82);
        Serial.println("✅ HTTP + WebSocket 服务启动完成");
    }else{
        wifi_connected = false;
        digitalWrite(WIFI_LED_PIN, LOW);
        Serial.println("\n❌ WiFi 连接失败");
    }
}

void wifi_stop() {
    if(!wifi_connected) return;
    
    Serial.println("\n📶 关闭WiFi...");
    if(camera_httpd != NULL){
        httpd_stop(camera_httpd);
        camera_httpd = NULL;
    }
    WiFi.disconnect(true);
    wifi_connected = false;
    digitalWrite(WIFI_LED_PIN, LOW);
    Serial.println("✅ WiFi 已关闭，进入纯单机模式");
}

// 判断时间是否同步成功
bool isTimeSynced() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return false;
    return (timeinfo.tm_year + 1900) > 2023;
}

// ===================== 系统初始化 =====================
void setup() {
    Serial.begin(115200);
    Serial.println("==================== 系统启动 ====================");
    door_unlocked = false;
    pinMode(relay_pin, OUTPUT);
    digitalWrite(relay_pin, LOW);
    Serial.printf("🔌 继电器引脚 %d 初始电平：LOW\n", relay_pin);

    // 初始化WiFi状态LED
    pinMode(WIFI_LED_PIN, OUTPUT);
    digitalWrite(WIFI_LED_PIN, LOW);
    Serial.println("💡 WiFi状态LED初始化完成（GPIO33）");

    // 初始化按键
    pinMode(WIFI_TOGGLE_PIN, INPUT_PULLUP);
    buttonState = digitalRead(WIFI_TOGGLE_PIN);
    Serial.printf("🎛️ WiFi切换按键初始化完成 引脚：%d\n", WIFI_TOGGLE_PIN);

    pinMode(RESTART_PIN, INPUT_PULLUP);
    restartButtonState = digitalRead(RESTART_PIN);
    Serial.printf("🎛️ 重启按键初始化完成 引脚：%d\n", RESTART_PIN);

    // 【新增】初始化单机注册按键
    pinMode(ENROLL_PIN, INPUT_PULLUP);
    enrollButtonState = digitalRead(ENROLL_PIN);
    Serial.printf("🎛️ 单机注册按键初始化完成 引脚：%d\n", ENROLL_PIN);

    // SD卡初始化
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("❌ SD卡挂载失败！");
        sd_card_available = false;
    } else {
        Serial.println("✅ SD卡挂载成功！");
        sd_card_available = true;
        SD_MMC.mkdir("/photos");
        SD_MMC.mkdir("/FACES");
    }

    // 摄像头初始化
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    if (psramFound()) {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 2;
        Serial.println("✅ 检测到PSRAM");
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 20;
        config.fb_count = 1;
        Serial.println("⚠️ 未检测到PSRAM");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ 摄像头初始化失败 错误码: 0x%x\n", err);
        return;
    }
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);
    Serial.println("✅ 摄像头初始化成功");

    // 加载人脸库
    app_facenet_main();

    // 开机自动同步时间
    Serial.println("\n==================== 开机自动同步时间 ====================");
    wifi_connect_only();
    unsigned long syncTimeout = millis() + 5000;
    while (!isTimeSynced() && millis() < syncTimeout) {
        Serial.print(".");
        delay(200);
    }
    if (isTimeSynced()) {
        char timeStr[32];
        getCurrentDateTime(timeStr);
        Serial.println("\n✅ 时间同步成功！当前时间：" + String(timeStr));
    } else {
        Serial.println("\n❌ 时间同步超时，使用默认时间");
    }
    wifi_stop();
    Serial.println("==================== 开机流程完成，进入单机人脸识别模式 ====================");

    Serial.println("\n==================================");
    Serial.println("✅ 人脸识别门锁启动完成");
    Serial.println("==================================");
}

// ===================== 主循环 =====================
void loop() {
    static WebsocketsClient client;

    // 最高优先级：三按键独立检测
    // 1. WiFi切换按键
    int wifiReading = digitalRead(WIFI_TOGGLE_PIN);
    if (wifiReading != lastButtonState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (wifiReading != buttonState) {
            buttonState = wifiReading;
            if (buttonState == LOW) {
                Serial.println("\n================================");
                Serial.println("🎛️ WiFi切换按键触发");
                wifi_connected ? wifi_stop() : wifi_start();
                Serial.println("================================");
            }
        }
    }
    lastButtonState = wifiReading;

    // 2. 重启按键
    int restartReading = digitalRead(RESTART_PIN);
    if (restartReading != lastRestartButtonState) {
        lastRestartDebounceTime = millis();
    }
    if ((millis() - lastRestartDebounceTime) > DEBOUNCE_DELAY) {
        if (restartReading != restartButtonState) {
            restartButtonState = restartReading;
            if (restartButtonState == LOW) {
                Serial.println("\n================================");
                Serial.println("🔴 重启按键触发，系统硬件复位...");
                Serial.println("================================");
                delay(100);
                esp_restart();
            }
        }
    }
    lastRestartButtonState = restartReading;

    // 3. 【新增】单机人脸注册按键
    // 3. 【优化】单机人脸注册按键（GPIO14）
    // 3. 【修复】单机人脸注册按键（防开机误触发）
    int enrollReading = digitalRead(ENROLL_PIN);
    if (enrollReading != lastEnrollButtonState) {
        lastEnrollDebounceTime = millis();
    }
    // 增加开机延时过滤 + 严格消抖
    if (millis() > 5000 && (millis() - lastEnrollDebounceTime) > 500) {
        if (enrollReading != enrollButtonState) {
            enrollButtonState = enrollReading;
            if (enrollButtonState == LOW) {
                Serial.println("\n================================");
                Serial.println("🎛️ 单机注册按键 已按下！");
                startStandaloneEnroll(); 
                Serial.println("================================");
            }
        }
    }
    lastEnrollButtonState = enrollReading;

    // 自动超时落锁
    if (door_unlocked && millis() - door_opened_millis > interval) {
        door_unlocked = false;
        digitalWrite(relay_pin, LOW);
        Serial.println("🔒 超时，门已自动锁定");
    }

    // 核心逻辑：优先处理单机注册 → 再处理人脸识别
    if(!wifi_connected) {
        if (standalone_enroll) {
            processStandaloneEnroll(); // 单机注册流程
        } else {
            standaloneFaceRecognition(); // 单机人脸识别
        }
    } else {
        if (client.available()) {
            runFaceRecognition();
        } else {
            standaloneFaceRecognition();
        }
    }

    // WiFi在线时的WebSocket服务
    if(wifi_connected) {
        socket_server.poll();
        
        if(!client.available()){
            client = socket_server.accept();
            if(client.available()){
                client.onMessage(handle_message);
                send_face_list(client);
                sendDoorStatus(client);
                Serial.println("✅ WebSocket客户端已连接 → 网页版人脸识别");
            }
        }

        if(client.available()){
            client.poll();
            fb = esp_camera_fb_get();
            if (fb) {
                client.sendBinary((const char*)fb->buf, fb->len);

                if(g_state == START_ENROLL) {
                    fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
                    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
                    
                    if (net_boxes) {
                        client.send("FACE DETECTED");
                        if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
                            dl_matrix3d_t *face_id = get_face_id(aligned_face);
                            int left_sample = do_enrollment(&st_face_list, face_id);
                            
                            char sampleMsg[64];
                            sprintf(sampleMsg, "SAMPLE NUMBER %d/%d", ENROLL_CONFIRM_TIMES - left_sample, ENROLL_CONFIRM_TIMES);
                            client.send(sampleMsg);
                            saveEnrollFaceToSD(fb, st_name.enroll_name, ENROLL_CONFIRM_TIMES - left_sample);

                            if (left_sample == 0) {
                                char doneMsg[64];
                                sprintf(doneMsg, "FACE CAPTURED FOR %s", st_name.enroll_name);
                                client.send(doneMsg);
                                g_state = START_RECOGNITION;
                                send_face_list(client);
                            }
                            dl_matrix3d_free(face_id);
                        }
                    } else {
                        client.send("NO FACE DETECTED");
                    }
                }
                esp_camera_fb_return(fb);

                static unsigned long t = 0;
                if (millis() - t > 300) { 
                    t = millis(); 
                    sendDoorStatus(client); 
                }
            }
        }
    }

    delay(10);
}