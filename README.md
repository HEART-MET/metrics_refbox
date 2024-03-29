metrics_refbox
-------------

UI for refbox for metrics benchmark runs, and can be used to:

1. Generate, edit and delete trial configurations for each benchmark
2. Send start and stop commands to the refbox client
3. Save results received from the refbox client

## Dependencies
Most dependencies are listed in [package.xml](package.xml)

## Usage Instructions
Usage instructions can be found in this [PDF file](refbox_instructions.pdf).


## Run using docker

### Build
```
sudo docker build -t metrics_refbox docker
```

### Execute
```
sudo docker run -it --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --name refbox_container metrics_refbox:latest
```

