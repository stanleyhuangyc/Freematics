var source;
var origin;

window.addEventListener("message", receiveMessage, false);

function receiveMessage(event) {
	//event.source.postMessage(event.data,event.origin);
    source = event.source;
    origin = event.origin;
	processInput(event.data);
}

function sendMessage(data) {
    if (source) source.postMessage(data, origin);
}

function checkData(data, key)
{
	var i;
	if ((i = data.lastIndexOf(key)) >= 0) {
		var j = data.indexOf("\r", i);
		return j >= 0 ? data.substr(i + key.length, j - i - key.length) : data.substr(i + key.length);
	}
	return null;
}

var imgTick = "<img src='tick.png'/>";
var imgCross = "<img src='cross.png'/>";

var con = "";
var inited = false;

function processInput(data)
{
	var i;
	var ret;
	if (con.length > 1024) con = con.substr(512);
	con += data;
	if (!inited) {
		if (ret = checkData(con, "Flash:")) {
			document.getElementById("flash_size").innerText = ret;
		}
		if (ret = checkData(con, "PSRAM:")) {
			document.getElementById("psram_size").innerText = ret.substr(0, 2) == "E " ? "N/A" : ret;
		}
		if (ret = checkData(con, "Firmware:")) {
			document.getElementById("firmware").innerText = ret;
		}
		if (ret = checkData(con, "DEVICE ID:")) {
			document.getElementById("devid").value = ret;
		}
		if (ret = checkData(con, "RTC:")) {
			document.getElementById("rtc").innerText = ret;
		}
		if (ret = checkData(con, "SD:")) {
			document.getElementById("sd_size").innerHTML = ret;
		}
		if (ret = checkData(con, "NO SD CARD") != null) {
			document.getElementById("sd_size").innerHTML = "NO CARD";
		}
		if (ret = checkData(con, "GNSS:")) {
			document.getElementById("gps").innerHTML = ret.indexOf("NO") >= 0 ? imgCross : imgTick;
		}
		if (ret = checkData(con, "OBD:")) {
			document.getElementById("obd").innerHTML = ret.indexOf("NO") >= 0 ? imgCross : imgTick;
		}
		if (ret = checkData(con, "MEMS:")) {
			document.getElementById("mems").innerHTML = ret.indexOf("NO") >= 0 ? imgCross : (imgTick + " " + ret);
		}
		if (ret = checkData(con, "HTTPD:")) {
			document.getElementById("wifi").innerHTML = ret.indexOf("NO") >= 0 ? imgCross : imgTick;
		}
		if (ret = checkData(con, "WiFi IP:")) {
			document.getElementById("wifi").innerHTML = imgTick + " IP:" + ret;
		}
		if (ret = checkData(con, "IMEI:")) {
			document.getElementById("sim_card").innerHTML = imgTick;
			document.getElementById("imei").innerText = "IMEI:" + ret;
		}
		if (ret = checkData(con, "RSSI:")) {
			document.getElementById("rssi").innerText = "RSSI:" + ret;
		}
		if (ret = checkData(con, "CELL:")) {
			document.getElementById("cell").innerHTML = ret == "NO" ? imgCross : (imgTick + " " + ret);
		}
		if ((ret = checkData(con, "NO SIM CARD")) != null) {
			document.getElementById("sim_card").innerHTML = imgCross;
		}
		if (ret = checkData(con, "Operator:")) {
			document.getElementById("sim_card").innerHTML = imgTick + " " + ret
		}
		if (ret = checkData(con, "Unable to connect") != null) {
			document.getElementById("server").innerHTML = imgCross;
		}
		if (ret = checkData(con, "LOGIN")) {
			document.getElementById("server").innerText = "Connecting to server" + ret;
		}
	}
	if (ret = checkData(con, "[NET]")) {
		document.getElementById("server").innerText = ret;
		inited = true;
	}
	if (ret = checkData(con, "[BUF]")) {
		document.getElementById("buffer").innerText = ret;
	}
	if (ret = checkData(con, "[FILE]")) {
		document.getElementById("file").innerText = ret;
	}
	if (ret = checkData(con, "[GPS]")) {
		document.getElementById("gps").innerText = ret;
	}
}
