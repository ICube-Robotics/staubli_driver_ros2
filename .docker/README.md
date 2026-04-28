# Demo docker image

## Build image locally

```bash
cd staubli_driver_ros2

export STAUBLI_DRIVER_VERSION="$(xmllint --xpath 'string(//version)' staubli_driver_ros2/package.xml)"
export IMG_NAME=staubli_driver_ros2
export FULL_IMG_NAME="$IMG_NAME:$STAUBLI_DRIVER_VERSION"

echo "Building $FULL_IMG_NAME..."
docker build -t $FULL_IMG_NAME -f .docker/Dockerfile .

# Save to archive
docker save  $FULL_IMG_NAME | gzip > ${IMG_NAME}-${STAUBLI_DRIVER_VERSION}.tar.gz

# If you prefer to have a progress bar:

sudo apt-get install pv

docker save $FULL_IMG_NAME | pv -N "Docker Save" -s $(docker image inspect $FULL_IMG_NAME --format='{{.Size}}') -p -t -e -b | gzip > ${IMG_NAME}-${STAUBLI_DRIVER_VERSION}.tar.gz
```

## Load from TAR

```bash
docker load < ${IMG_NAME}-${STAUBLI_DRIVER_VERSION}.tar.gz
```

## Launch demo app (Moveit2)

```bash
cd staubli_driver_ros2/.docker

docker compose up
```
