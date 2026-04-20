# LiDAR-Based Driving on Raspberry Pi 5 using ROS 2 Jazzy

[![Video Label](https://img.youtube.com/vi/d7c770zymt0/maxresdefault.jpg)](https://youtu.be/d7c770zymt0)

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [전체 블록도](#2-전체-블록도)
3. [ROS2 토픽 데이터 구조](#3-ros2-토픽-데이터-구조)
4. [생성자](#4-생성자--linedetectorlinedetector)
5. [preprocess\_image() — 이미지 전처리](#5-preprocess_image----이미지-전처리)
6. [find\_target\_line() — 장애물 탐색](#6-find_target_line----장애물-탐색)
7. [draw\_result() — 시각화](#7-draw_result----시각화)
8. [getch() / kbhit() — 키보드 입력](#8-getch--kbhit----키보드-입력)
9. [mysub\_callback() — 메인 콜백](#9-mysub_callback----메인-콜백)
10. [실행 결과](#10-실행-결과)

---

## 1. 시스템 개요

ROS2 환경에서 LiDAR 스캔 데이터를 Raspberry Pi 5에서 WSL2로 전송하고, WSL2에서 2D 맵 변환 및 장애물 탐색 후 계산된 모터 속도를 다시 Raspberry Pi 5로 보내 Dynamixel 모터를 제어함.

| 항목 | 내용 |
|------|------|
| 하드웨어 | Raspberry Pi 5 + RPLIDAR A1 + Dynamixel MX-12W |
| OS | Ubuntu 24.04 (RPI5) ↔ WSL2 Ubuntu 24.04 |
| 미들웨어 | ROS2 Jazzy |
| 통신 방식 | ROS2 Topic (KeepLast 10, Reliable) |
| 알고리즘 | Connected Components + P제어 기반 속도 제어 |

---

## 2. 전체 블록도

<img width="1138" height="868" alt="image" src="https://github.com/user-attachments/assets/f9ec83d8-71a9-4332-a41a-5ce5cda02bf5" />

> **ROS_DOMAIN_ID** 를 로봇 번호와 동일하게 설정해야 같은 네트워크에서 토픽이 올바르게 구독됨.

---

## 3. ROS2 토픽 데이터 구조

<img width="942" height="787" alt="image" src="https://github.com/user-attachments/assets/5d5d97ad-6157-4e2b-9325-53beff6468ae" />
<img width="890" height="895" alt="image" src="https://github.com/user-attachments/assets/e8af8271-9297-4e2a-b1e4-e05a9fb05eb5" />

### `scan`
- **타입**: `sensor_msgs/msg/LaserScan`
- **방향**: RPI5 → WSL2

```
LaserScan
├── ranges[]        : float32[]  ← 각 방향 거리값 (m)
├── angle_min       : float32    ← 시작 각도 (rad)
├── angle_max       : float32    ← 끝 각도 (rad)
├── angle_increment : float32    ← 포인트 간 각도 간격
└── scan_time       : float32    ← 한 회전 소요 시간
```

### `topic_dxlpub`
- **타입**: `geometry_msgs/msg/Vector3`
- **방향**: WSL2 → RPI5

```
Vector3
├── x : float64  ← 왼쪽 모터 목표 속도
├── y : float64  ← 오른쪽 모터 목표 속도 (음수 = 역방향)
└── z : 0.0      ← 미사용
```

---

## 4. 생성자 — `LineDetector::LineDetector()`

```cpp
LineDetector::LineDetector() : Node("sllidar_client"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
```
- `Node("sllidar_client")` : 노드 이름 설정
- `tmp_pt_l(125, 125)` : 왼쪽 장애물 초기 추적 위치 (화면 좌측 1/4 지점)
- `tmp_pt_r(375, 125)` : 오른쪽 장애물 초기 추적 위치 (화면 우측 3/4 지점)
- `first_run_(true)` : 첫 실행 플래그 초기화
- `mode(false)` : 정지 상태로 시작

```cpp
    vel.x = 0; vel.y = 0; vel.z = 0;
```
- 모터 속도 벡터 0으로 초기화

```cpp
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile, bind(&LineDetector::mysub_callback, this, placeholders::_1));
```
- `scan` 토픽 구독, 데이터 수신 시마다 `mysub_callback` 호출

```cpp
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
```
- 계산된 좌/우 모터 속도를 `topic_dxlpub` 으로 퍼블리시

---

## 5. `preprocess_image()` — 이미지 전처리

2D 맵 이미지를 장애물 탐색에 적합한 형태로 변환함.

```cpp
cv::cvtColor(result, frame_gray, cv::COLOR_BGR2GRAY);
```
- 컬러(BGR) 이미지를 단일 채널 흑백으로 변환

```cpp
cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);
```
- 픽셀값 100 기준 반전 이진화
- `THRESH_BINARY_INV` : 어두운 점(장애물)을 흰색으로 뒤집어 객체로 인식하게 함

```cpp
return frame_binary(cv::Rect(0, 0, 500, 250));
```
- 전체 500×500 이미지에서 상단 절반(전방 영역)만 ROI로 잘라 반환
- 불필요한 후방 영역 제거로 오검출 감소

---

## 6. `find_target_line()` — 장애물 탐색

Connected Components 결과에서 좌/우 가장 가까운 장애물 인덱스를 찾는 핵심 함수.

```cpp
cv::Point robot_pos(250, 250);
```
- 라이다 맵에서 로봇 위치는 항상 화면 중앙(250, 250)으로 고정

```cpp
double dist = cv::norm(robot_pos - obj_pos);
```
- 로봇 위치와 각 객체 중심 사이의 유클리드 거리 계산

```cpp
if (cx < 250) {
    if (dist < min_dist_l) { min_dist_l = dist; l_idx = i; }
} else {
    if (dist < min_dist_r) { min_dist_r = dist; r_idx = i; }
}
```
- x좌표 250 기준으로 좌/우 영역 분리
- 각 영역에서 로봇과 가장 가까운 객체를 타겟으로 선택

```cpp
if (l_idx != -1) {
    tmp_pt_l = cv::Point(centroids.at<double>(l_idx, 0), ...);
} else {
    tmp_pt_l = cv::Point(0, 0);    // 못 찾으면 왼쪽 구석으로
}
if (r_idx != -1) {
    tmp_pt_r = cv::Point(centroids.at<double>(r_idx, 0), ...);
} else {
    tmp_pt_r = cv::Point(500, 0);  // 못 찾으면 오른쪽 구석으로
}
```
- 탐색 성공 시 `tmp_pt_` 갱신
- 해당 방향에 장애물 없을 시 기준점을 화면 가장자리로 밀어 오차 계산에 반영

---

## 7. `draw_result()` — 시각화

탐색 결과를 화면에 그리는 함수.

```cpp
if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
```
- 흑백 이미지인 경우 컬러로 변환 (색깔 표시를 위해)

```cpp
if (i == left_idx) color = cv::Scalar(0, 255, 0);  // 왼쪽: 초록
else               color = cv::Scalar(0, 0, 255);  // 오른쪽: 빨강

cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
cv::circle(result, cv::Point(x, y), 3, color, -1);
```
- 왼쪽 타겟은 초록, 오른쪽 타겟은 빨강으로 바운딩 박스 + 중심점 표시

```cpp
cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1);
cv::arrowedLine(result, robot_pos, r_box_bottom_left,  cv::Scalar(0, 0, 255), 1);
```
- 로봇 위치(중앙)에서 각 장애물을 향한 방향 화살표 표시

```cpp
int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
cv::arrowedLine(result, robot_pos, cv::Point(target_x, 100), cv::Scalar(255, 0, 0), 1);
```
- 좌/우 장애물의 중간점을 향한 파란 화살표로 목표 주행 방향 표시

```cpp
cv::circle(result, tmp_pt_l, 3, cv::Scalar(0, 255, 0), -1);
cv::circle(result, tmp_pt_r, 3, cv::Scalar(0, 0, 255), -1);
```
- 장애물을 잃었을 때도 마지막 기억 위치에 점 유지

---

## 8. `getch()` / `kbhit()` — 키보드 입력

엔터 없이 즉시 키 입력을 감지하는 함수.

```cpp
newt.c_lflag &= ~(ICANON | ECHO);
tcsetattr(STDIN_FILENO, TCSANOW, &newt);
```
- 터미널을 raw 모드로 전환 (엔터 대기 제거, 입력 에코 끔)

```cpp
// kbhit()에서
fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
ch = getchar();
```
- Non-blocking 모드로 전환해 키 입력 없어도 즉시 리턴

```cpp
if (ch != EOF) {
    ungetc(ch, stdin);  // 읽은 문자 버퍼에 되돌려 놓음
    return true;
}
```
- 키가 눌렸으면 `true` 반환, 문자는 버퍼에 돌려놔서 이후 `getch()`가 다시 읽을 수 있게 함

---

## 9. `mysub_callback()` — 메인 콜백

라이다 데이터가 수신될 때마다 실행되는 핵심 함수.

### 2D 맵 생성

```cpp
int count = scan->scan_time / scan->time_increment;
```
- 한 바퀴 스캔의 총 포인트 수 계산

```cpp
float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1);
```
- 극좌표 `(r, θ)` → 직교좌표 `(x, y)` 변환 후 빨간 점으로 2D 맵에 표시
- `scale = 8.0` : 거리값을 픽셀 크기에 맞게 키우는 배율

```cpp
cv::warpAffine(scan_video, result, rotate, scan_video.size());
cv::flip(result, result, 1);
```
- 180도 회전 후 좌우 반전 → 라이다 좌표계와 화면 좌표계 방향 보정

### 오차 계산 및 속도 제어

```cpp
int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
```
- 화면 중앙(250)과 좌/우 장애물 중간점의 차이로 조향 오차 계산
- 양수 : 중간점이 왼쪽으로 치우침 / 음수 : 오른쪽으로 치우침

```cpp
if (ch == 'q') mode = false;    // 정지
else if (ch == 's') mode = true; // 주행 시작
```
- `s` 키: 주행 시작 / `q` 키: 정지

```cpp
if (error == 0 || error < -60 || error > 60) {
    vel.x = 50;   vel.y = -50;   // 이상 상황 → 직진
} else {
    vel.x = 50 - k * error;      // 왼쪽 모터
    vel.y = -(50 + k * error);   // 오른쪽 모터 (k = 0.45)
}
```
- 오차가 60픽셀 초과하거나 0이면 직진 (탐색 실패 대응)
- 정상 범위에서는 P제어로 오차에 비례해 좌/우 속도 차등 조정

### 동영상 저장

```cpp
static cv::VideoWriter video_writer;
static bool is_writer_initialized = false;
```
- `static` 선언으로 콜백 종료 후에도 객체 유지

```cpp
if (!is_writer_initialized) {
    video_writer.open("lidar_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, ...);
    if (video_writer.isOpened()) { is_writer_initialized = true; }
}
if (is_writer_initialized) { video_writer.write(result); }
```
- 첫 프레임에서 한 번만 초기화, 이후 매 프레임 mp4로 저장
- `is_writer_initialized = true` 가 없으면 매 프레임마다 파일을 새로 열어 덮어씀 (버그 주의)

---

## 10. 실행 결과

| 항목 | 내용 |
|------|------|
| 구독 토픽 | `scan` |
| 퍼블리시 토픽 | `topic_dxlpub` |
| 노드 이름 | `sllidar_client` |
| ROI 영역 | 500×250 (라이다 맵 전방 절반) |
| 조향 게인 (k) | 0.45 |
| 저장 파일 | `lidar_video.mp4` |

```
[INFO] [sllidar_client]: Received Image : err:-8, leftvel: 53.60, rightvel: -46.40, time:5.21 ms
[INFO] [sllidar_client]: Received Image : err:0,  leftvel: 50.00, rightvel: -50.00, time:4.98 ms
Video Writer Initialized Successfully.
```

| 항목 | 설명 |
|------|------|
| `err` | 양수: 중간점이 왼쪽 치우침 / 음수: 오른쪽 치우침 |
| `leftvel` | 왼쪽 모터 목표 속도 |
| `rightvel` | 오른쪽 모터 목표 속도 (음수 = 역방향) |
| `time` | 한 프레임 처리 시간 (ms) |


[![Video Label](https://img.youtube.com/vi/d7c770zymt0/maxresdefault.jpg)](https://youtu.be/d7c770zymt0)
