const { exec, spawn } = require("child_process");

let runRtspServer = exec("mavlink-routerd -e 172.30.82.150:14550");

runRtspServer.stdout.on("data", function(data) {
  console.log(data.toString());
});

runRtspServer.stderr.on("data", function(data) {
  console.error(data.toString());
});
