const { exec, spawn } = require("child_process");

let runRtspServer = exec("gst-launch-1.0 -e nvcompositor name=comp sink_0::alpha=1 sink_0::xpos=0 sink_0::ypos=0 sink_1::alpha=1 sink_1::xpos=0 sink_1::ypos=1080 ! nvoverlaysink display-id=1 rtspsrc location=rtsp://172.30.82.109:8554/mystream ! matroskademux  name=demux0 demux0.video_0! h264parse ! omxh264dec ! nvvidconv ! comp.sink_0 rtspsrc location=rtsp://172.30.82.109:8554/mystream1 ! matroskademux  name=demux1 demux1.video_0! h264parse ! omxh264dec ! nvvidconv ! comp.sink_1 ");

runRtspServer.stdout.on("data", function(data) {
  console.log(data.toString());
});

runRtspServer.stderr.on("data", function(data) {
  console.error(data.toString());
});
