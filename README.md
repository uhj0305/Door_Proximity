# Door_Proximity
/*
 * iOS 기기 도어락 근접 도어 오픈(iOS device door lock proximity door opening)
 * 
  현관 도어락 근접 도어 자동 제어 모듈 펌웨어(Door lock proximity door automatic control module firmware)
  [KOR]
  - 인증된 아이폰/아이패드/애플워치 연결을 위한(iOS) IRK 지원
  - 페어링 전용 학습 버튼 입력(GPIO 2) 및 암호없음
  - 도어제어 접점신호 출력(GPIO 42) 
  - 재실 신호 출력(GPIO 41)
  - 도어 근접하면 자동으로 열림(GPIO 6 핀으로 펄스 출력)
  - 도어 제어장치 한개에 최대 20개의 iOS 기기 학습 지원
  - Paring 스위치를 눌러 학습(20개 초과 학습되면 가장 처음 장치가 삭제됨)
  - 학습된 위치의 RSSI를 임계값으로 저장함(학습위치보다 더 가까워지면 열림)
  - Arduino\Library\AduinoBLE 폴더 압축 후 삭제 후 사용
  - 지원하는 아두이노 디바이스 ESP32 NORMAL/C3/S2/S3
  [ENG]
  - IRK support for authenticated iPhone/iPad/Apple Watch connection (iOS)
  - Pairing-only learning button input (GPIO 2) and no password
  - Door control contact signal output (GPIO 42)
  - Presence signal output (GPIO 41)
  - Automatically opens when the door approaches (pulse output through GPIO 6 pin)
  - Supports learning of up to 20 iOS devices for one door control device
  - Learning by pressing the Paring switch (if more than 20 devices are learned, the first device is deleted)
  - Saves the RSSI of the learned location as a threshold (opens when closer than the learned location)
  - Use after compressing and deleting the Arduino\Library\AduinoBLE folder.
  - Supported Arduino devices ESP32 NORMAL/C3/S2/S3

  2022 Aaron Woo, uhj0305@gmail.com
*/
