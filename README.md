# HIT-Paper-SourceCode

논문에서 사용한 기술은 Kubernetes 환경에서 ROS와 Spring Boot가 운영되며 Ros Bridge 를 이용한 Web Socket 통신으로 ROS와 Web 인터페이스가 연결된다. 사용자는 웹 인터페이스에서 로봇을 실시간으로 제어할 수 있다. 이 논문에서는 Turtlebot3 SLAM과 Navigation을 통해 로봇에 움직임을 실험하고 Prometheus와 Grafana를 통해 Kubernetes에 성능 관찰과 HPA 기술을 이용한 Kubernetes 환경에서 자원 관리 방법을 소개한다.

## 실험 결과

### 웹 인터페이스와 SLAM에서 실시간 로봇 움직임 과정

![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/96232d3c-d6ab-4e78-af6f-58b0d5ef9bb3)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/88eccd6f-bdad-4a86-85bc-d5c5329dc751)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/5457c903-4065-4254-868a-1212cfdacd41)


### 웹 인터페이스와 Navigation에서 실시간 로봇 움직임 과정

![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/534e00d0-b3ae-47aa-a49d-1d0bfa4b282a)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/2b9543db-99be-4bb3-b91f-5f5c762730a6)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/a78fae70-7766-4fac-8705-02b6bbc11a08)


### Prometheus를 이용해 메트릭 수집 후 CPU, 메모리 네트워크 상태 확인

![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/826757d4-d032-4b25-8f97-f1efd9e54d25)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/0f7de529-e6cb-4d9b-bdbd-52de915ccf6f)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/45dc58e8-c740-4d18-b529-0540f4421ee2)


### Prometheus를 이용해 HPA 부하 과정 실험

1. 부하 전 

![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/6f2df8a6-e229-4ffa-9b06-1c6a00241b4f)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/6119a164-2c26-4823-896e-fd033d9d2b25)

2. 부하 후

![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/9d4ea6e8-2ee7-4391-9f66-af85dc3bb076)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/cae440b2-68cb-4917-8f02-06d55139386e)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/fbe26b60-28c9-4c4b-8a22-d0c644b22ce5)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/7e69427c-91bd-43b1-a41a-00136917aa77)
![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/50dbc2cc-32b6-45c6-ae65-95fcb3118360)

3. 부하 명령어 종료 후 

![image](https://github.com/jaeseonNamgung/HIT-Paper-SourceCode/assets/84066249/80add75c-65e3-404b-9cb1-f891a3bf3bce)
![Uploading image.png…]()

