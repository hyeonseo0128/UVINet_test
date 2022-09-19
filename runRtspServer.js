const { exec, spawn } = require("child_process");

let runRtspServer = exec("./rtsp-simple-server");

runRtspServer.stdout.on("data", function(data) {
  console.log(data.toString());
});

runRtspServer.stderr.on("data", function(data) {
  console.error(data.toString());
});

