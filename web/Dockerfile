FROM openjdk:17-jdk
LABEL authors="sunnamgung"
ARG JAR_FILE=build/libs/web-0.0.1-SNAPSHOT.jar
ADD ${JAR_FILE} docker-springboot.jar
ENTRYPOINT ["java","-Djava.security.egd=file:/dev/./urandom","-jar","/docker-springboot.jar"]