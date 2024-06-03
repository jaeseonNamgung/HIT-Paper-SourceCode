FROM amazoncorretto:17
LABEL authors="namgung"
COPY build/libs/web-0.0.1-SNAPSHOT.jar ros-web.jar
EXPOSE 8080
ENTRYPOINT ["java", "-jar", "ros-web.jar"]