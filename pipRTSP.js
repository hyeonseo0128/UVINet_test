const { exec, spawn } = require("child_process");

let runRtspServer = exec("gst-launch-1.0 v4l2src device=/dev/video0 ! queue2 ! c.sink_0 v4l2src device=/dev/video1 ! queue2 ! c.sink_1 imxg2dcompositor name=c background-color=0x223344 sink_0::xpos=0 sink_0::ypos=0 sink_0::width=400 sink_0::height=300 sink_0::fill_color=0x00000000 sink_1::xpos=400 sink_1::ypos=0 sink_1::width=400 sink_1::height=300 sink_1::fill_color=0x11111111 queue2 ! video/x-raw, width=800, height=600 ! rtmpsink location='rtmp://localhost:1935/mystream live=1'");

runRtspServer.stdout.on("data", function(data) {
  console.log(data.toString());
});

runRtspServer.stderr.on("data", function(data) {
  console.error(data.toString());
});
