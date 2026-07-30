FROM osrf/ros:jazzy-desktop-full

WORKDIR /src

COPY . .

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade --break-system-packages -r requirements.txt

EXPOSE 8000

CMD ["ros2", "launch", "start_gazebo.launch.py"]