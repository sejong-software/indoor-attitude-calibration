# 영상 기반 실내 드론 자세 보정 시스템 설계

## 참여 인력
- 김응주(팀장) (garon0927@sju.ac.kr)
- 강산희 (sanh2@sju.ac.kr)
- 권영서 (kys@sju.ac.kr)
- 이주한 (akffldhs12@naver.com)
- 김학주 (hakzoo1@naver.com)

## 사용 언어
- C#, matlab, unity

## 목표
- 영상 정보를 활용한 정확한 방향각 계산
- 방향각의 오차를 추정 및 보상하는 알고리즘 개발
- 시뮬레이터를 통한 알고리즘 성능 검증

## 주요 연구 내용
- IMU 및 지자기 센서를 이용한 자세 계산 알고리즘 개발
- 영상 정보를 활용한 방향각 오차 보상 알고리즘 개발
- 자세 계산 및 방향각 오차 보상 알고리즘 성능 검증을 위한 시뮬레이터 개발

### IMU 및 지자기 센서를 이용한 상보필터 개념도
![image](https://user-images.githubusercontent.com/46476876/107909203-13ef5c80-6f9b-11eb-96f8-d867f2901e6a.png)
### 방위각 오차 보상 알고리즘
![image](https://user-images.githubusercontent.com/46476876/107909233-1ea9f180-6f9b-11eb-8ece-d2877b3d1f9c.png)
### 시뮬레이터 구성
![image](https://user-images.githubusercontent.com/46476876/107909280-2ff2fe00-6f9b-11eb-9dc3-932423718d83.png)

## 결과물
### 논문
[논문](https://github.com/sejong-software/indoor-attitude-calibration/blob/main/%EB%8B%A4%EC%A4%91%20%EC%84%BC%EC%84%9C%20%EB%B0%8F%20%EC%98%81%EC%83%81%20%EC%A0%95%EB%B3%B4%EB%A5%BC%20%EC%9D%B4%EC%9A%A9%ED%95%9C%20%EC%8B%A4%EB%82%B4%20%EB%93%9C%EB%A1%A0%EC%9D%98%20%EC%9E%90%EC%84%B8%20%EB%B3%B4%EC%A0%95%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EA%B0%9C%EB%B0%9C.pdf)
이주한, 김응주, 강산희, 권영서, 김학주, 송진우. (2020). 다중 센서 및 영상 정보를 이용한 실내 드론의 자세 보정 알고리즘 개발. 정보 및 제어 논문집, (), 51-52.
### 시뮬레이터
![image](https://user-images.githubusercontent.com/46476876/107909328-4a2cdc00-6f9b-11eb-9226-e1ca3f1ee881.png)
### 특허
송진우, 이주한, 김응주, 김용훈, 강산희. 실내 환경에서 운행하는 무인 비행체의 방위각의 추정. 01-2020-0171022.2020.

