// Requiered to connect to gamepad
// USAGE ON FLASK PYTHON
    // socketio = SocketIO(app)
    // controller = None
    // @socketio.on("submit gamepad")
    // def gamepad(data):
    //     # Array: 0-3 axis 4-n buttons
    //     controller = data


var socket;
document.addEventListener('DOMContentLoaded', () => {

    // Connect to websocket
    socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port);

    // When connected, configure buttons
    socket.on('connect', updateStatus);

});

var haveEvents = 'GamepadEvent' in window;
var haveWebkitEvents = 'WebKitGamepadEvent' in window;
var controllers = {};
var rAF = window.mozRequestAnimationFrame ||
  window.webkitRequestAnimationFrame ||
  window.requestAnimationFrame;

function connecthandler(e) {
  addgamepad(e.gamepad);
}
function addgamepad(gamepad) {
  controllers[gamepad.index] = gamepad; var d = document.createElement("div");
  d.setAttribute("id", "controller" + gamepad.index);
  var t = document.createElement("h1");
  t.appendChild(document.createTextNode("gamepad: " + gamepad.id));
  d.appendChild(t);
  var b = document.createElement("div");
  b.className = "buttons";
  for (var i=0; i<gamepad.buttons.length; i++) {
    var e = document.createElement("span");
    e.className = "button";
    //e.id = "b" + i;
    e.innerHTML = i;
    b.appendChild(e);
  }
  d.appendChild(b);
  var a = document.createElement("div");
  a.className = "axes";
  for (i=0; i<gamepad.axes.length; i++) {
    // e = document.createElement("meter");
    // e.className = "axis";
    // //e.id = "a" + i;
    // e.setAttribute("min", "-1");
    // e.setAttribute("max", "1");
    // e.setAttribute("value", "0");
    // e.innerHTML = i;
    // a.appendChild(e);
    f = document.createElement("h2");
    f.className = "axis";
    f.innerHTML = "Value = 0";
    a.appendChild(f);
    //f.appendChild(document.createTextNode("Value = "+"0"))
  }
  d.appendChild(a);
  // document.getElementById("start").style.display = "none";
  document.body.appendChild(d);
  rAF(updateStatus);
}

function disconnecthandler(e) {
  removegamepad(e.gamepad);
}

function removegamepad(gamepad) {
  var d = document.getElementById("controller" + gamepad.index);
  document.body.removeChild(d);
  delete controllers[gamepad.index];
}

function updateStatus() {
  scangamepads();
  if (!(controllers[0]==undefined)) {
    // axis0 = controllers[0]
    var buttons = []
    for (var i=0; i<controllers[0].buttons.length; i++) {
      buttons.push(controllers[0].buttons[i].value);
    }
    data0 = controllers[0].axes.concat(buttons)
    socket.emit('submit gamepad',data0);
  }

  
  rAF(updateStatus);
}

function scangamepads() {
  var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
  for (var i = 0; i < gamepads.length; i++) {
    if (gamepads[i]) {
      if (!(gamepads[i].index in controllers)) {
        addgamepad(gamepads[i]);
      } else {
        controllers[gamepads[i].index] = gamepads[i];
      }
    }
  }
}

if (haveEvents) {
  window.addEventListener("gamepadconnected", connecthandler);
  window.addEventListener("gamepaddisconnected", disconnecthandler);
} else if (haveWebkitEvents) {
  window.addEventListener("webkitgamepadconnected", connecthandler);
  window.addEventListener("webkitgamepaddisconnected", disconnecthandler);
} else {
  setInterval(scangamepads, 50);
}

console.log(controllers)