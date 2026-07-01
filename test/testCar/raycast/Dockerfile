FROM nvcr.io/nvidia/l4t-pytorch:r32.7.1-pth1.9-py3

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    LD_LIBRARY_PATH=/usr/local/cuda/targets/aarch64-linux/lib:$LD_LIBRARY_PATH

WORKDIR /app

RUN rm -f /etc/apt/sources.list.d/kitware* \
 && sed -i '/kitware/d' /etc/apt/sources.list

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-opencv \
    && rm -rf /var/lib/apt/lists/*

COPY raycast.py .

CMD ["python3", "raycast.py"]
