# HIT-Paper-SourceCode

논문에서 사용한 기술은 Kubernetes 환경에서 ROS와 Spring Boot가 운영되며 Ros Bridge 를 이용한 Web Socket 통신으로 ROS와 Web 인터페이스가 연결된다. 사용자는 웹 인터페이스에서 로봇을 실시간으로 제어할 수 있다. 이 논문에서는 Turtlebot3 SLAM과 Navigation을 통해 로봇에 움직임을 실험하고 Prometheus와 Grafana를 통해 Kubernetes에 성능 관찰과 HPA 기술을 이용한 Kubernetes 환경에서 자원 관리 방법을 소개한다.

## 실험 결과

### 웹 인터페이스와 SLAM에서 실시간 로봇 움직임 과정

![image-20240603133348996](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133348996.png)

![image-20240603133356105](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133356105.png)

![image-20240603133402449](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133402449.png)

### 웹 인터페이스와 Navigation에서 실시간 로봇 움직임 과정

![image-20240603133507203](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133507203.png)

![image-20240603133511851](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133511851.png)

![image-20240603133516106](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133516106.png)

### Prometheus를 이용해 메트릭 수집 후 CPU, 메모리 네트워크 상태 확인

![image-20240603133556557](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133556557.png)

![image-20240603133602187](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133602187.png)

![image-20240603133608134](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133608134.png)

### Prometheus를 이용해 HPA 부하 과정 실험

1. 부하 전 

![image-20240603133613353](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133613353.png)

![image-20240603133617652](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133617652.png)

2. 부하 후

![image-20240603133628317](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133628317.png)

![image-20240603133632380](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133632380.png)

![image-20240603133636910](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133636910.png)

![image-20240603133642423](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133642423.png)

![image-20240603133647268](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133647268.png)

3. 부하 명령어 종료 후 

![image-20240603133651881](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133651881.png)

![image-20240603133656203](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20240603133656203.png)
