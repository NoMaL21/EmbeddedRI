# 라즈베리 인더스트리 최종 데모 파일

1. 아이디어 소개
- RTC 모듈을 사용한 시간측정으로 사용자가 원하는 시간에 알림 전송
- YL-40 모듈을 사용하여 온도를 측정, 스마트폰으로 전송
- 블루투스 모듈을 사용하여 스마트폰과의 통신기능

2. 전체 시스템 구조
![image](https://user-images.githubusercontent.com/89987277/208411617-39e1fbd9-b0fb-452b-b4e5-de9b9db4e21a.png)

3. 제한 조건 구현 내용 및 가산점 요소
- 제한 조건
  + 시간 측정과 온도 측정 함수를 나눠 멀티쓰레드 사용
  + 시간 감지와 온도 측정 변수에 뮤텍스 사용
- 가산점 요소
 + 라즈베리파이와 스마트폰 간의 블루투스 통신

4. 문제점 및 해결 방안
> 1. 저항 값 섭씨 온도 변경 문제 > 데이터 시트로 변환방법 검색, PPT 자료 확인
> 2. 스마트폰 통신 문제 > 파이썬 코드 사용 불가로 인한 숙련도 부족, 텔레그램 X 블루투스 통신 사용

5. 실행사진
![실험 사진(축소)](https://user-images.githubusercontent.com/89987277/208416353-7f99b17b-32ac-414b-9c6a-b5abca1c1c8a.jpg)
