FROM python:3.12.3-slim

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

WORKDIR /app

RUN apt-get update && apt-get install -y \
    build-essential git \
    libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev \
    libportmidi-dev libfreetype6-dev pkg-config \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade pip setuptools wheel --root-user-action=ignore

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt --root-user-action=ignore

RUN git clone --depth 1 https://github.com/shmul95/pyvesc.git /tmp/pyvesc \
 && mv /tmp/pyvesc /usr/local/lib/python3.12/site-packages/pyvesc \
 && rm -rf /tmp/pyvesc

COPY action_config.json .
COPY autopilot.py .
COPY car.py .

CMD ["python", "car.py"]
