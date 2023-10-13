module.exports = function (RED) {
  "use strict";
  let WS = require("ws");
  let bciLib = require("./bci-lib");
  let config = require("./bci-config");

  function WebSocketInNode(n) {
    RED.nodes.createNode(this, n);

    let node = this;
    let path = config.path;
    let clientid = config.clientid;
    let clientsecret = config.clientsecret;
    let errorCode = config.errorCode;
    let sessionID = null;
    let auth = null;
    let headsetID = null;
    let loggedInOSUsername = "";
    let loggedInUsername = "";

    function startconn() {
      // Connect to remote endpoint
      node.tout = null;
      node.socket = new WS(path, { rejectUnauthorized: false });
      node.socket.setMaxListeners(config.maxSocketListener);
      node.context().global.set("socket", node.socket);
      handleConnection();
    }

    function handleConnection() {
      node.socket.on("open", function () {
        node.closing = false;
        bciLib.getUserLogin(node);

        node.status({
          fill: "white",
          shape: "ring",
          text: "Connecting ..."
        });
      });

      node.socket.on("close", function () {
        node.status({
          fill: "red",
          shape: "ring",
          text:
            "Please ensure that you have Cortex installed and running in the background."
        });
        if (!node.closing) {
          clearTimeout(node.tout);
          node.tout = setTimeout(function () {
            startconn();
          }, 3000);
        }
      });

      node.socket.on("message", function (data) {
        let msg = JSON.parse(data);
        console.log("Message")
        console.log(msg)


        if (!!msg.warning) {
          switch (msg.warning.code) {
            case 0:
              // Stop session
              sessionID = null;
              setTimeout(() => {
                bciLib.queryHeadsets(node);
              }, 3500);
              break;

            case 1:
              // Close session
              break;

            case 2:
              // Already logged ininin
              bciLib.authorizeWithOutLicense(node, clientid, clientsecret);
              break;

            case 3:
              // Log out
              node.status({
                fill: "red",
                shape: "ring",
                text: "Please login via Emotiv Apps"
              });
              break;

            default:
              // node.status({fill: 'red', shape: 'ring', text: msg.warning.message});
              break;
          }
        } else if (!!msg.id && !!msg.error) {
          switch (msg.error.code) {
            case errorCode.ERR_NO_HEADSET:
              node.status({
                fill: "red",
                shape: "ring",
                text: "No headset is connected!"
              });
              headsetID = null;
              sessionID = null;
              setTimeout(() => {
                bciLib.queryHeadsets(node);
              }, 3500);
              break;

            case errorCode.ERR_SESSION_CONFLICT:
              throw new Error("ERR_SESSION_CONFLICT");

            case errorCode.ERR_REQUEST_TIME_OUT:
              node.status({
                fill: "red",
                shape: "ring",
                text: "Request time out, please re-deploy flow!"
              });
              break;

            case errorCode.ERR_INVALID_PROFILE:
              node.status({
                fill: "red",
                shape: "ring",
                text: "Request time out, please re-deploy flow!"
              });

            default:
              // node.status({fill: 'red', shape: 'ring', text: msg.error.message});
              break;
          }
        } else {
          switch (msg.id) {
            case "getUserLogin":
              if (msg.result.length != 0) {
                loggedInUsername = msg.result[0].username;
                loggedInOSUsername = msg.result[0].loggedInOSUsername;
                bciLib.hasAccessRight(node, clientid, clientsecret);
              } else {
                node.status({
                  fill: "red",
                  shape: "ring",
                  text: "Please login via other Emotiv Cortex Apps"
                });
              }
              break;

            case "hasAccessRight":
              if (msg.result.accessGranted) {
                bciLib.authorizeWithOutLicense(node, clientid, clientsecret);
              } else {
                setTimeout(() => {
                  bciLib.requestAccess(node, clientid, clientsecret);
                }, 3000);
              }
              break;

            case "requestAccess":
              bciLib.hasAccessRight(node, clientid, clientsecret);
              break;

            case "login":
              node.status({ fill: "green", shape: "ring", text: "Logged in" });
              bciLib.authorizeWithOutLicense(node, clientid, clientsecret);
              break;

            case "authorizeWithOutLicense":
              sessionID = null;
              node.status({ fill: "green", shape: "ring", text: "Authorized" });
              auth = msg.result.cortexToken;
              node.context().global.set("auth", auth);
              bciLib.queryHeadsets(node);
              break;

            case "createSession":
              sessionID = msg.result.id;
              node.context().global.set("sessionID", sessionID);
              let stream = config.stream;
              bciLib.subscribe(node, auth, stream, sessionID);
              storeSocketToFlow(auth);

              let dataStreamArr = node.context().global.get("streamArr");

              if (!!dataStreamArr) {
                node.context().global.set("streamArr", dataStreamArr);
              }
              if (!!headsetID) {
                node.context().global.set("headsetID", headsetID);
              }
              break;

            case "getCurrentProfile":
              if (msg.result != null && msg.result != "") {
                // Use the loading profile without loading again
                const loadedProfile = msg.result;
                bciLib.unloadProfile(node, auth, headsetID, loadedProfile);
              }
              break;
            case "connectHeadset":
              bciLib.createSession(node, auth, headsetID);
              break;

            case "queryHeadsets":
              if (!!msg.result[0]) {
                msg.result.forEach((item, idx) => {
                  console.log(`[DEV-INFO] Headset ${idx} Info `, item)
                })
                bciLib.queryProfile(node, auth);

                // Note: If there are multiple headsets connected to Emotiv Cortex
                // pick first headset as data headset for the sake of simplicity
                const HEADSET_INDEX = 0
                console.log(`[DEV-INFO] Data headset: `, msg.result[HEADSET_INDEX])
                node.context().global.set("headsetInfo", msg.result[HEADSET_INDEX]);
                headsetID = msg.result[0].id || null;
                bciLib.connectHeadset(node, headsetID);
              } else {
                if (node.socket.readyState === WS.OPEN) {
                  node.status({
                    fill: "red",
                    shape: "ring",
                    text: "No headset is connected!"
                  });
                  setTimeout(() => {
                    bciLib.queryHeadsets(node);
                  }, 3000);
                }
              }
              break;
            default:
              node.status({ fill: "green", shape: "ring", text: `Logged in as ${loggedInUsername}` });
              break;
          }
        }
      });

      node.socket.on("error", function (err) {
        if (!node.closing) {
          clearTimeout(node.tout);
          node.tout = setTimeout(function () {
            startconn();
          }, 3000);
        }
      });
    }

    function storeSocketToFlow(auth) {
      node.status({ fill: "green", shape: "ring", text: "Connected!" });
      node.send({ payload: [auth] });
    }

    node.closing = false;
    startconn(); // start outbound connection

    node.on("close", function () {
      node.closing = true;
      sessionID = null;
      node.socket.close();
      if (node.tout) {
        clearTimeout(node.tout);
      }
    });
  }
  RED.nodes.registerType("EMOTIV", WebSocketInNode);
};
