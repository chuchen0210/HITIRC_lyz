# SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# PROJECT_DIR=$(realpath "$SCRIPT_DIR")

# xhost +
# docker run --name tongverselite-release -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
#   --ulimit rtprio=99 \
#   -e "PRIVACY_CONSENT=Y" \
#   -v $HOME/.Xauthority:/root/.Xauthority \
#   -e DISPLAY \
#   -v ~/docker/isaac-sim_2023.1.0/cache/kit:/isaac-sim/kit/cache:rw \
#   -v ~/docker/isaac-sim_2023.1.0/cache/ov:/root/.cache/ov:rw \
#   -v ~/docker/isaac-sim_2023.1.0/cache/pip:/root/.cache/pip:rw \
#   -v ~/docker/isaac-sim_2023.1.0/cache/glcache:/root/.cache/nvidia/GLCache:rw \
#   -v ~/docker/isaac-sim_2023.1.0/cache/computecache:/root/.nv/ComputeCache:rw \
#   -v ~/docker/isaac-sim_2023.1.0/logs:/root/.nvidia-omniverse/logs:rw \
#   -v ~/docker/isaac-sim_2023.1.0/data:/root/.local/share/ov/data:rw \
#   -v ~/docker/isaac-sim_2023.1.0/documents:/root/Documents:rw \
#   -v $PROJECT_DIR:/BipedChallenge:rw \
#   tongverselite-release:v1.0 bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$(realpath "$SCRIPT_DIR")

xhost +

docker run --name tongverselite-release -it --gpus all --rm --network=host \
  --ulimit rtprio=99 \
  --env="ACCEPT_EULA=Y" \
  --env="PRIVACY_CONSENT=Y" \
  --env="DISPLAY=$DISPLAY" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env http_proxy=http://127.0.0.1:7890 \
  --env https_proxy=http://127.0.0.1:7890 \
  --env HTTP_PROXY=http://127.0.0.1:7890 \
  --env HTTPS_PROXY=http://127.0.0.1:7890 \
  --ipc=host \
  --device=/dev/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v ~/docker/isaac-sim_2023.1.0/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim_2023.1.0/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim_2023.1.0/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim_2023.1.0/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim_2023.1.0/documents:/root/Documents:rw \
  -v $PROJECT_DIR:/BipedChallenge:rw \
  tongverselite-release:v1.0 bash
