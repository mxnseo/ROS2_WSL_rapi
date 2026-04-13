# LiDAR-Based Driving on Raspberry Pi 5 using ROS 2 Jazzy

[![Video Label](https://img.youtube.com/vi/d7c770zymt0/maxresdefault.jpg)](https://youtu.be/d7c770zymt0)

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [전체 블록도](#2-전체-블록도)
3. [ROS2 토픽 데이터 구조](#3-ros2-토픽-데이터-구조)
4. [preprocess\_image() — 이미지 전처리](#4-preprocess_image----이미지-전처리)
5. [find\_target\_line() — 라인·장애물 탐색](#5-find_target_line----라인장애물-탐색)
6. [draw\_result() — 시각화](#6-draw_result----시각화)
7. [mysub\_callback() — 메인 콜백](#7-mysub_callback----메인-콜백)
8. [속도 제어 로직](#8-속도-제어-로직)
9. [패키지별 파라미터 비교](#9-패키지별-파라미터-비교)

---

## 1. 시스템 개요

ROS2 환경에서 카메라 이미지 및 LiDAR 스캔 데이터를 Raspberry Pi 5에서 WSL2로 전송하고, WSL2에서 영상 처리 및 장애물 탐색 후 계산된 모터 속도를 다시 Raspberry Pi 5로 보내 Dynamixel 모터를 제어함.

| 항목 | 내용 |
|------|------|
| 하드웨어 | Raspberry Pi 5 + RPLIDAR A1 + Dynamixel MX-12W |
| OS | Ubuntu 24.04 (RPI5) ↔ WSL2 Ubuntu 24.04 |
| 미들웨어 | ROS2 Jazzy |
| 통신 방식 | ROS2 Topic (KeepLast 10, Reliable) |
| 알고리즘 | Connected Components + 오차 기반 속도 제어 |

---

## 2. 전체 블록도
<img width="1138" height="868" alt="image" src="https://github.com/user-attachments/assets/f9ec83d8-71a9-4332-a41a-5ce5cda02bf5" />


> **ROS_DOMAIN_ID** 를 로봇 번호와 동일하게 설정해야 같은 네트워크에서 토픽이 올바르게 구독됨.

---

## 3. ROS2 토픽 데이터 구조
<img width="942" height="787" alt="image" src="https://github.com/user-attachments/assets/5d5d97ad-6157-4e2b-9325-53beff6468ae" />

### `image/compressed_12`
- **타입**: `sensor_msgs/msg/CompressedImage`
- **방향**: RPI5 → WSL2
- **주기**: 40fps

```
CompressedImage
├── header.stamp   : 타임스탬프
├── format         : "jpeg"
└── data           : uint8[]  ← JPEG 바이트 배열
```

### `scan`
- **타입**: `sensor_msgs/msg/LaserScan`
- **방향**: RPI5 → WSL2
- **주기**: 10Hz

```
LaserScan
├── ranges[]       : float32[]  ← 각 방향 거리값 (m)
├── angle_min      : float32    ← 시작 각도 (rad)
├── angle_max      : float32    ← 끝 각도 (rad)
├── angle_increment: float32    ← 포인트 간 각도 간격
└── scan_time      : float32    ← 한 회전 소요 시간
```

### `topic_dxlpub`
- **타입**: `geometry_msgs/msg/Vector3`
- **방향**: WSL2 → RPI5

```
Vector3
├── x : float64  ← 왼쪽 모터 목표 속도
├── y : float64  ← 오른쪽 모터 목표 속도 (음수 = 반대 방향)
└── z : 0.0      ← 미사용
```

---

## 4. `preprocess_image()` — 이미지 전처리

입력 이미지를 분석 가능한 이진화 ROI로 변환하는 파이프라인.

```
BGR 컬러 → 흑백 변환 → 밝기 보정 → 이진화 → ROI 추출
```

### 라인별 코드 설명

```cpp
cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
```
- BGR 3채널 컬러 이미지를 단일 채널 흑백으로 변환
- 이후 밝기 연산 및 이진화를 위한 전처리

```cpp
cv::Scalar bright_avg = cv::mean(frame_gray);
frame_gray = frame_gray + (100 - bright_avg[0]);
```
- `cv::mean()` 으로 프레임 전체의 평균 밝기 계산
- 목표 밝기(90~120, 패키지별 상이)와의 차이만큼 픽셀값 가감
- 조명 변화에 강건한 적응형 밝기 보정

```cpp
cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);
```
- 픽셀값 130 기준 이진화 (흰색/검정 분리)
- 라이다 패키지는 `THRESH_BINARY_INV` 사용 → 어두운 점(장애물)을 흰색으로 반전

```cpp
return frame_binary(cv::Rect(0, 240, 640, 120));
// 또는
return frame_binary(cv::Rect(0, 0, 500, 250));  // 라이다용
```
- 전체 이미지에서 관심 영역(ROI)만 잘라 반환
- 카메라: 하단 120~150픽셀 영역 / 라이다: 500×250 전방 영역
- 불필요한 배경 제거로 오검출 감소 및 처리 속도 향상

---

## 5. `find_target_line()` — 라인·장애물 탐색

Connected Components 레이블링 결과에서 추적할 객체를 선택하는 핵심 알고리즘.

### 첫 실행 (`first_run_ == true`)

```cpp
int cx = cvRound(centroids.at<double>(i, 0));
int dist_from_center = abs(320 - cx);
```
- 화면 중앙(320) 또는 좌/우 1/4·3/4 지점을 기준으로 초기 탐색
- 인덱스 0번(배경) 제외, 면적 100 이하 노이즈 무시

```cpp
if (best_idx != -1) {
    tmp_pt_ = cv::Point(centroids.at<double>(best_idx, 0), ...);
    first_run_ = false;
}
```
- 기준점과 가장 가까운 객체를 초기 타겟으로 확정
- `tmp_pt_` 에 위치 저장 후 플래그 해제

### 연속 추적 (이후 프레임)

```cpp
int dist = cv::norm(cv::Point(x, y) - tmp_pt_);
```
- 현재 객체 위치와 이전 프레임 타겟 위치 간 유클리드 거리 계산

```cpp
if (dist < min_dist && dist <= search_radius) {
    min_dist = dist;
    min_idx  = i;
}
```
- `search_radius` (50~150px, 패키지별 상이) 이내 객체만 같은 타겟으로 인정
- 범위 밖 객체는 다른 라인으로 간주해 무시

```cpp
if (min_idx != -1)
    tmp_pt_ = cv::Point(centroids.at<double>(min_idx, 0), ...);
return min_idx;
```
- 탐색 성공 시 `tmp_pt_` 갱신, 실패 시 이전 위치 유지 (라인 일시 소실 대응)
- 찾은 인덱스 반환 (-1이면 탐색 실패)

---

## 6. `draw_result()` — 시각화

처리 결과를 화면에 그리는 시각화 함수.

```cpp
cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
```
- 이진화 이미지를 컬러로 변환 (색깔 표시를 위해 필요)

```cpp
cv::rectangle(result, cv::Rect(left, top, width, height),
              cv::Scalar(255, 0, 0), 1);  // 파란색: 일반 객체
cv::circle(result, cv::Point(x, y), 5,
           cv::Scalar(255, 0, 0), -1);
```
- 면적 100 이상의 모든 객체를 파란색 사각형+점으로 표시

```cpp
// 타겟 객체는 빨간색으로 강조
if (i == left_idx || i == right_idx)
    color = cv::Scalar(0, 0, 255);
```
- 추적 중인 타겟은 빨간색으로 구분 표시

```cpp
cv::arrowedLine(result, robot_pos, l_box_bottom_right,
                cv::Scalar(0, 255, 0), 1);  // 초록: 왼쪽 장애물 방향
cv::arrowedLine(result, robot_pos, target_pos,
                cv::Scalar(255, 0, 0), 1);  // 파란: 목표 주행 방향
```
- 로봇 위치(중앙)에서 각 장애물 방향으로 화살표 (라이다 패키지)
- 좌/우 중간점을 향한 파란 화살표로 목표 주행 방향 표시

```cpp
cv::circle(result, tmp_pt_, 8, cv::Scalar(0, 0, 255), -1);
```
- 타겟을 잃었을 때도 `tmp_pt_` (마지막 기억 위치)에 빨간 점 항상 표시

---

## 7. `mysub_callback()` — 메인 콜백

### 카메라 패키지

```cpp
cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
if (frame_color.empty()) return;
```
- 압축 이미지 메시지를 OpenCV Mat으로 디코딩
- 디코딩 실패 시 즉시 함수 종료

```cpp
cv::Mat roi = preprocess_image(frame_color);
cv::connectedComponentsWithStats(roi, labels, stats, centroids);
```
- 전처리 → Connected Components 레이블링 수행
- `stats`: 각 객체의 위치·크기·면적 / `centroids`: 중심 좌표

```cpp
int error = 320 - tmp_pt_.x;  // 단일 라인
// 또는
int error = center_x - ((tmp_pt_l.x + tmp_pt_r.x) / 2);  // 좌/우 두 라인
```
- 화면 중앙과 타겟(들) 위치의 차이로 오차 계산
- 양수: 타겟이 왼쪽으로 치우침 / 음수: 오른쪽으로 치우침

### 라이다 패키지

```cpp
int count = scan->scan_time / scan->time_increment;
float x = 250 + scan->ranges[i] * (10.0 * scale) * std::sin(angle);
float y = 250 - scan->ranges[i] * (10.0 * scale) * std::cos(angle);
```
- 총 포인트 수 계산
- 극좌표 `(r, θ)` → 직교좌표 `(x, y)` 변환 후 2D 맵에 점 표시

```cpp
cv::warpAffine(scan_video, result, rotate, scan_video.size());
cv::flip(result, result, 1);
```
- 180도 회전 + 좌우 반전으로 라이다 좌표계와 화면 좌표계 방향 보정

---

## 8. 속도 제어 로직

```cpp
if (kbhit()) {
    int ch = getch();
    if (ch == 'q') mode = false;   // 정지
    else if (ch == 's') mode = true; // 구동 시작
}
```
- 터미널 raw 모드로 전환해 엔터 없이 즉시 키 입력 감지
- `s`: 구동 시작 / `q`: 정지

```cpp
vel.x =  base - k * error;   // 왼쪽 모터
vel.y = -(base + k * error);  // 오른쪽 모터
```
- 오차가 양수(타겟이 왼쪽): 왼쪽 속도 감소, 오른쪽 증가 → 왼쪽으로 조향
- 오차가 음수(타겟이 오른쪽): 반대 방향으로 조향

---

## 9. 패키지별 파라미터 비교

| 파라미터 | linetracer_sim | lanefollow_sim | lanefollow_real | lidardrive | lidarsim |
|----------|:--------------:|:--------------:|:---------------:|:----------:|:--------:|
| 기본 속도 (base) | 110 | 100 | 100 | 50 | 100 |
| 조향 게인 (k) | 0.07 | 0.18 | 0.2 | 0.45 | 0.3 |
| 탐색 반경 (px) | 60 | 150 | 50 | 없음 | 없음 |
| 밝기 보정 목표 | 120 | 100 | 90 | - | - |
| 이진화 임계값 | 130 | 120 | 120 | 100(반전) | 100(반전) |
| ROI | y=360~480 | 하단 1/4 | 하단 1/4 | 500×250 | 500×250 |
| 타겟 수 | 1 (단일 라인) | 2 (좌/우) | 2 (좌/우) | 2 (좌/우) | 2 (좌/우) |
| first_run_ 해제 | 하나 발견 시 | 하나 발견 시 | 둘 다 발견 시 | 없음 | 없음 |
| 영상 저장 | 없음 | 없음 | 없음 | mp4 저장 | 없음 |

---

## 실행 결과 로그 예시

```
[INFO] [sllidar_client]: Received Image : err:-8, leftvel: 53.60, rightvel: -46.40, time:5.21 ms
[INFO] [camsub_wsl_12]:  Received Image : err:3,  leftvel: 100.60, rightvel: -99.40, time:3.87 ms
Video Writer Initialized Successfully.
```

| 항목 | 설명 |
|------|------|
| `err` | 양수: 타겟이 왼쪽으로 치우침 / 음수: 오른쪽으로 치우침 |
| `leftvel` | 왼쪽 모터에 전달되는 목표 속도 |
| `rightvel` | 오른쪽 모터에 전달되는 목표 속도 (음수 = 역방향) |
| `time` | 한 프레임 처리에 걸린 시간 (ms) |
