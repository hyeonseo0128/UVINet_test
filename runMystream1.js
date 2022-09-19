const { exec, spawn } = require("child_process");

let runRtspServer = exec("gst-launch-1.0 -v -e v4l2src device=/dev/video1 ! 'video/x-raw, width=640, height=480, framerate=30/1, format=YUY2' ! nvvidconv ! 'video/x-raw(memory:NVMM), format=(string)NV12' ! omxh264enc bitrate=1000000 ! flvmux ! rtmpsink location='rtmp://localhost:1935/mystream1 live=1'");

runRtspServer.stdout.on("data", function(data) {
  console.log(data.toString());
});

runRtspServer.stderr.on("data", function(data) {
  console.error(data.toString());
});

