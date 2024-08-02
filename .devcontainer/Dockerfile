ARG VARIANT=dev
ARG ROS_DISTRO=humble

FROM jncfa/ros2:${ROS_DISTRO}-${VARIANT}

# ** [Optional] Uncomment this section to install additional packages. **
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   pre-commit \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/.env ]; then source ${WORKSPACE}/.env; fi" >> /home/ros/.bashrc
