/******************************************************************************
* Web based vehicle tracker based on Freematics Hub
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit http://freematics.com/hub/api for Freematics Hub API reference
* To obtain your Freematics Hub server key, contact support@freematics.com.au
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

var TRIP_TIMEOUT = 300000;
var REQUEST_INTERVAL = 1000;
var ROLLBACK_TIME = 300000;
var MAX_CACHE_SIZE = 10000;
var DATA_FETCH_INTERVAL = 500;

var PID_GPS_LATITUDE = 0xA;
var PID_GPS_LONGITUDE = 0xB;
var PID_GPS_ALTITUDE = 0xC;
var PID_GPS_SPEED = 0xD;
var PID_GPS_HEADING = 0xE;
var PID_GPS_SAT_COUNT = 0xF;
var PID_GPS_TIME = 0x10;
var PID_GPS_DATE = 0x11;
var PID_ACC = 0x20;
var PID_GYRO = 0x21;
var PID_COMPASS = 0x22;
var PID_MEMS_TEMP = 0x23;
var PID_BATTERY_VOLTAGE = 0x24;
var PID_TRIP_DISTANCE = 0x30;

var FLAG_DRIVING = 0x1;

var chart = new Array;
var gauge = new Array;
var deviceStartTick;
var lastDataTick = 0;
var cache = new Array;
var stats = null;
var lat, lng;

var serverURL = "freematics.php?api=";

var chartData = [
    [0x10d, "Vehicle Speed", "#808080", "km/h", "Speed (km/h)", 100],
    [0x10c, "Engine RPM", "#404040", "rpm", "RPM", 100],
    [0x104, "Engine Load", "#808080", "%", "Engine (%)", 100],
    [PID_ACC, "G-force", "#404040", "g", "G-force", 100],
    [PID_BATTERY_VOLTAGE, "Battery Voltage", "#808080", "V", "Voltage (V)", 100]
];

var gaugeData = [
    [0x10d, "Speed (km/h)", 0, 250],
    [0x10c, "RPM", 0, 8000],
    [0x111, "Throttle (%)", 0, 100],
    [PID_BATTERY_VOLTAGE, "Battery (V)", 0, 15]
];

$(function () {
    $(document).ready(function () {
        Highcharts.setOptions({
            global: {
                useUTC: false
            }
        });
        for (var n = 0; n < gaugeData.length; n++) {
            gauge[n] = CreateGauge("gauge" + n, gaugeData[n][1], gaugeData[n][2], gaugeData[n][3]);

        }
    });
});

var channel = getUrlArg("channel");
var profile;
if (!channel) channel = 1;

function init()
{
    // init UI
    var tab = "";
    for (var n = 0; n < chartData.length; n++) {
        tab += "<label><input type='checkbox' id='chartCheck" + n + "' onclick='toggleChart(" + n + ")'/>" + chartData[n][1] + "</label> ";
    }
    document.getElementById("chartTabs").innerHTML = tab;

    // load channel data
    if (!loadChannels()) {
        return;
    }
	if (profile.tick) {
        lastDataTick = deviceStartTick = profile.tick;
        requestDataHistory();
	} else {
		document.getElementById("stats").innerHTML = "Invalid profile";
	}
}

var map;
var marker;

function initMap() {
	map = new google.maps.Map(document.getElementById('map'), {
		zoom: 12,
		mapTypeId: google.maps.MapTypeId.ROADMAP,
		center: {lat: -33.890, lng: 151.274}
	});
  	var image = 'car.png';
	marker = new google.maps.Marker( {map: map, icon: image} );
	marker.addListener('click', showMapInfo);
}

function setMapPos(latitude, longitude)
{
	marker.setPosition({lat:latitude, lng:longitude});
	map.panTo({lat:latitude, lng:longitude});	
}

function showMapInfo()
{
	var contentString = getCarInfo();
	var infowindow = new google.maps.InfoWindow({
		content: contentString
	  });
	infowindow.open(map, marker);
}

function getCarInfo()
{
	var str ="";
    if (cache[PID_GPS_LATITUDE]) {
        str += "LAT:" + cache[PID_GPS_LATITUDE];
        str += " LNG:" + cache[PID_GPS_LONGITUDE];
        str += " ALT:" + cache[PID_GPS_ALTITUDE] + "m<br/>"
    }
	str += "Speed:" + cache[0x10d] + "kph";
	str += " RPM:" + cache[0x10c];
	str += " Engine: " + cache[0x104] + "%";
	return str;	
}

function getHHMMSS(sec)
{
    var hours   = Math.floor(sec / 3600);
    var minutes = Math.floor((sec - (hours * 3600)) / 60);
    var seconds = sec - (hours * 3600) - (minutes * 60);
    if (hours   < 10) {hours   = "0"+hours;}
    if (minutes < 10) {minutes = "0"+minutes;}
    if (seconds < 10) {seconds = "0"+seconds;}
    return hours+':'+minutes+':'+seconds;
}

function getTickCount()
{
    var d = new Date();
    return d.getTime(); 
}

function updateStats()
{
	// update stats
    if (!stats) return;
    
	var s = "";
    if (!(stats.flags & FLAG_DRIVING) || stats.age > TRIP_TIMEOUT) { 
		s = "PARKING " + getHHMMSS(Math.floor(stats.age / 1000));
    } else {
        s = "DRIVING " + getHHMMSS(stats.elapsed);
    }
    document.getElementById("state").innerText = s;
    
    s = "";
    s += "<table>"
    if (profile.csq > 0) s += "<tr><td>CSQ</td><td>" + profile.csq / 10 + "dB</td>";
    s += "<tr><td width='50%'>Data</td><td>" + Math.floor(stats.data / 1024) + "KB</td>";
    //s += "<li>Device Tick: " + stats.tick + "</li>";
    //s += "<li>Local Tick: " + lastDataTick + "</li>";
    if (stats.age < TRIP_TIMEOUT) {
        if (cache[PID_TRIP_DISTANCE]) {
            s += "<tr><td>Distance</td><td>" + (parseInt(cache[PID_TRIP_DISTANCE]) / 1000).toFixed(1) + "km</td>";
        }
        s += "<tr><td>Top Speed</td><td>" + stats.topspeed + "km/h</td>";
        s += "<tr><td>Delay</td><td>" + stats.age + "ms</td>";
    }
    s += "</table>";

	document.getElementById("stats").innerHTML = s;
    stats = null;
}

var chartUpdateCount = 0;
var activeChart = [0,1];

function addChartData(ts, pid, value)
{
    // update charts data
    if (value == "") return;
    for (var n = 0; n < chartData.length; n++) {
        if (pid == chartData[n][0]) {
            var x = (new Date()).getTime() - (lastDataTick - ts);
            var y = normalizeData(pid, value);
            chart[n].series[0].addPoint([x, y], false, true);
            return;
        }
    }
}

function normalizeData(pid, value)
{
    switch (pid) {
        case PID_BATTERY_VOLTAGE:
            return parseInt(value) / 100;
        case PID_ACC:
            // data in x,y,z, get y axis
            var i = value.indexOf("/");
            if (i > 0) {
                return parseInt(value.substr(i + 1)) / 98;
            }
            break;
    }
    return parseInt(value);
}

var utc, alt, speed, sat;
var lastUTC;

function updateDataGrid()
{
    // update gauges
    for (var n = 0; n < gaugeData.length; n++) {
        var pid = gaugeData[n][0];
        if (cache[pid])
            gauge[n].series[0].points[0].update(normalizeData(pid, cache[pid]));
    }

    lat = cache[PID_GPS_LATITUDE];
    lng = cache[PID_GPS_LONGITUDE];
    alt = cache[PID_GPS_ALTITUDE];
    speed = cache[PID_GPS_SPEED];
    sat = cache[PID_GPS_SAT_COUNT];
    utc = cache[PID_GPS_TIME];

    // update OBD data
    var s = "<table class='grid'><tr><th>Parameter</th><th>Value</th></tr>";
    for (var n = 0; n < chartData.length; n++) {
        var value = cache[chartData[n][0]];
        if (value)
            s += "<tr><td>" + chartData[n][1] + "</td><td>" + value + "</td></tr>"   
    }
    if (utc) {
        // add data to grid
        s += "<tr><td>UTC</td><td>" + utc + "</td></tr>";
        s += "<tr><td>Latitude</td><td>" + lat + "</td></tr>";
        s += "<tr><td>Longitude</td><td>" + lng + "</td></tr>";
        if (alt) s += "<tr><td>Altitude</td><td>" + alt + "</td></tr>";
        if (speed) s += "<tr><td>GPS Speed</td><td>" + speed + "</td></tr>";
        if (sat) s += "<tr><td>Satellites</td><td>" + sat + "</td></tr>";
    }

    if (utc != lastUTC && lat && lng) {
        // has GPS data
        // update map
        setMapPos(parseFloat(lat), parseFloat(lng));
        lastUTC = utc;
    }

    document.getElementById("grid").innerHTML = s;
 }

function updateCharts()
{
    // refresh chart    
    chart[activeChart[0]].series[0].setVisible(true, true);
    chart[activeChart[1]].series[0].setVisible(true, true);
}

function toggleChart(index)
{
    if (index != activeChart[0] && index != activeChart[1]) {
        activeChart[0] = activeChart[1];
        activeChart[1] = index;
    }

    for (var n = 0; n < chartData.length; n++) {
        var enabled = n == activeChart[0] || n == activeChart[1];
        document.getElementById("container" + n).style.display = (enabled ? "block" : "none");
        document.getElementById("chartCheck" + n).checked = enabled;
    }
    chart[activeChart[0]].series[0].setVisible(true, true);
    chart[activeChart[1]].series[0].setVisible(true, true);
}

var xhr = newHttpRequest();

function requestData()
{
	xhr.onreadystatechange = function() {
        if (this.readyState != 4) return;
        if (this.status != 200) {
            if (this.status) {
                alert("Server under maintenance (status: " + this.status + ")");
            }
            return;
        }
        var pull = JSON.parse(this.responseText);
        if (pull.stats) {
            stats = pull.stats;
            if (pull.stats.tick < deviceStartTick) {
                // device reset
                alert("Device reset");
                location.reload();
                return;
            }
            updateStats();
        }
        if (pull.data.length > 0) {
            lastDataTick = pull.data[pull.data.length - 1][0];
            for (var i = 0; i < pull.data.length; i++) {
                var ts = pull.data[i][0];
                var pid = pull.data[i][1];
                var value = pull.data[i][2];
                if (pid <= 65535) cache[pid] = value;
                addChartData(ts, pid, value);
            }
            updateDataGrid();
            updateCharts();
        }
        self.setTimeout(requestData, DATA_FETCH_INTERVAL);
    };
	var url = serverURL + "/pull/" + channel + "?ts=" + (lastDataTick + 1);
	xhr.open('GET', url, true);    
	xhr.send(null);
}

function requestDataHistory()
{
	xhr.onreadystatechange = function() {
        if (this.readyState != 4 || this.status != 200) {
            return;
        }
        
        var pull = JSON.parse(this.responseText);
    
        if (pull.error) {
            alert(pull.error);
            return;
        } else if (pull.stats) {
            if (pull.data.length == 0) {
                if (pull.stats.flags & FLAG_DRIVING)
                    alert("No data in the past " + ROLLBACK_TIME / 60000 + " minutes");
                if (!stats) {
                    // no data, roll further back for once
                    stats = pull.stats;
                    self.setTimeout(requestDataHistory, 0);
                    return;
                }
        
            }
            stats = pull.stats;
            updateStats();
        }
    
        /*
        var data = [],
            time = (new Date()).getTime(),
            i;
    
        for (i = -119; i <= 0; i += 1) {
            data.push({
                x: time + i * 5000,
                y: 0
            });
        }
        */
        for (var i = pull.data.length - 1; i >= 0 && (!lat || !lng); i--) {
            switch (pull.data[i][1]) {
            case PID_GPS_LATITUDE: 
                lat = parseFloat(pull.data[i][2]);
                break;
            case PID_GPS_LONGITUDE:
                lng = parseFloat(pull.data[i][2]);
                break;
            }
        }
        if (lat && lng) setMapPos(lat, lng);

        for (var i = 0; i < pull.data.length; i++) {
            var pid = pull.data[i][1];
            if (pid <= 65535) cache[pid] = pull.data[i][2];
        }
        updateDataGrid();
        
        for (var n = 0; n < chartData.length; n++) {
            var mydata = [];
            // load history data;
            var pid = chartData[n][0];
            var curTime = (new Date()).getTime();
            for (var i = 0; i < pull.data.length; i++) {
                var value = pull.data[i][2];
                if (pid == pull.data[i][1]) {
                    var time = curTime - ROLLBACK_TIME - pull.stats.age + pull.data[i][0] - pull.data[0][0]; // current time minus elapsed in ms
                    mydata.push({
                        x: time,
                        y: normalizeData(pid, value)
                    });
                }
            }
            // create chart with loaded data
            chart[n] = CreateChart("container" + n, chartData[n][1], chartData[n][2], chartData[n][3], chartData[n][4], chartData[n][5], mydata);
        }
        toggleChart(0);
        toggleChart(1);
        // start receiving
        requestData();
    }
    
    if (!stats) {
        // relative rollback
	   xhr.open('GET', serverURL + "/pull/" + channel + "&rollback=" + ROLLBACK_TIME, true);
    } else {
        // rollback with timestamp
       xhr.open('GET', serverURL + "/pull/" + channel + "&ts=" + (stats.tick - ROLLBACK_TIME), true);
    }
	xhr.send(null);
}

function openChannel(id)
{
    if (id) {
		location.href = "#?channel=" + id;
        location.reload();
	}
}

function loadChannels()
{
    var chdata = loadJSON(serverURL + "/channels");
    if (!chdata) {
        document.getElementById("state").innerText = "Service unavailable";
        return false;
    }
    if (chdata.channels.length == 0) {
        document.getElementById("state").innerText = "No registered vehicle";
        return false;
    }
	var s = "<select onchange='openChannel(value)'>";
	for (var i = 0; i < chdata.channels.length; i++) {
	    if (chdata.channels[i].id) {
	        s += "<option value='" + chdata.channels[i].id + "'"
	        if (channel == chdata.channels[i].id) {
                profile = chdata.channels[i];
                s += " selected";
            }
            s += ">";
            s += chdata.channels[i].vin + "</option>";
		}
	}
    s += "</select>";
    document.getElementById("channels").innerHTML = s;
    self.setTimeout(loadChannels, 60000);
    return true;
}
