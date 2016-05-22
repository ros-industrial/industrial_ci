FROM ubuntu:16.04

MAINTAINER "ROS Industrial" "https://github.com/ros-industrial"

# Install packages
RUN apt-get update -qq
RUN apt-get -qq install -y --force-yes git sudo lsb-release python-pip wget
RUN apt-get clean
ENV IN_DOCKER 1
ENV TERM xterm
