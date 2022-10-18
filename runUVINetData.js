const { exec, spawn } = require("child_process");

let runRtspServer = exec("python3 Kuvinet_message_tacker_net.py");

runRtspServer.stdout.on("data", function(data) {
  console.log(data.toString());
});

runRtspServer.stderr.on("data", function(data) {
  console.error(data.toString());
});
