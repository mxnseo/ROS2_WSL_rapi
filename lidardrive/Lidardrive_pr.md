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
| 알고리즘 | 오차 기반 속도 제어 |

---

## 2. 전체 블록도
<img width="1138" height="868" alt="image" src="https://github.com/user-attachments/assets/f9ec83d8-71a9-4332-a41a-5ce5cda02bf5" />


> **ROS_DOMAIN_ID** 를 로봇 번호와 동일하게 설정해야 같은 네트워크에서 토픽이 올바르게 구독됨.

---

## 3. ROS2 토픽 데이터 구조

<img width="942" height="787" alt="image" src="https://github.com/user-attachments/assets/5d5d97ad-6157-4e2b-9325-53beff6468ae" />
<img width="890" height="895" alt="image" src="https://github.com/user-attachments/assets/e8af8271-9297-4e2a-b1e4-e05a9fb05eb5" />
