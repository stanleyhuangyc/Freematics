/******************************************************************************
* Web based vehicle tracker based on Freematics Hub
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit https://freematics.com/hub/api for Freematics Hub API reference
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


onResize();
USER.load("updateUI()");

var chart;

$(function () {
    $(document).ready(function () {
		var options = {
            enableHighAccuracy: true,
            timeout: 10000,
            maximumAge: 10000,
        };
        navigator.geolocation.getCurrentPosition(
            function (pos) {
                OSMAP.init("map", pos.coords.latitude, pos.coords.longitude, 15);
                //OSMAP.setCenter([pos.coords.latitude, pos.coords.longitude]);
            }, null, options);

    });
});

var myStyle = {
    "color": "#0000ff",
    "weight": 5,
    "opacity": 0.65
};

function getDateFromGPSFormat(date, time)
{
    var d = new Date();
	if (date.length == 6) {
		d.setUTCFullYear(2000 + (date % 100));
		d.setUTCMonth(Math.floor(date / 100) % 100 - 1);
		d.setUTCDate(Math.floor(date / 10000));
	} else {
		d.setUTCFullYear(Math.floor(date / 10000));
		d.setUTCMonth(Math.floor(date / 100) % 100 - 1);
		d.setUTCDate(date % 100);
	}
    if (time) {
        d.setUTCHours(Math.floor(time / 1000000))
        d.setUTCMinutes(Math.floor((time % 1000000) / 10000));
        d.setUTCSeconds(Math.floor((time % 10000) / 100));
        d.setUTCMilliseconds((time % 100) * 10);
    }
    return d;
}

function getISOFromLocal(date, time) {
    var d = new Date(Math.floor(date / 10000),
		Math.floor(date / 100) % 100 - 1,
		date % 100,
		Math.floor(time / 10000),
		Math.floor((time % 10000) / 100),
		Math.floor(time % 100)
	);
	return d.toISOString();
	/*
    var utc = d.getUTCFullYear();
    if (d.getUTCMonth() < 9) utc += "0";
    utc += (d.getUTCMonth() + 1);
    if (d.getUTCDate() < 10) utc += "0";
    utc += d.getUTCDate();
    utc += "-";
    if (d.getHours() < 10) utc += "0";
    utc += d.getHours();
    if (d.getMinutes() < 10) utc += "0";
    utc += d.getMinutes();
    if (d.getSeconds() < 10) utc += "0";
    utc += d.getSeconds();
    return utc;
	*/
}

function goDash(devid)
{
	var s = "?";
	if (USER.user) s += "user=" + USER.user + "&";
	s += "devid=" + devid;
	location.href = s;
}

function goNextDash(devid)
{
	if (USER.info.devid) {
		for (var i = 0; i < USER.info.devid.length; i++) {
			if (USER.info.devid[i] == devid) {
				 goDash(USER.info.devid[(i + 1) % USER.info.devid.length]);
				 return;
			}
		}
	}
}

function updateUI()
{
	var opts = document.getElementById("opts_range");
	setRange(opts.value);
	opts.style.display = "block";
	TRIPS.list();

	if (!USER.info) {
		document.getElementById("info").innerHTML = "DEVICE: " + USER.devid;
		return;
	}
	var s = "<select onchange='goDash(this.value)'>";
	for (var i = 0; i < USER.info.devid.length; i++) {
        s += "<option value=\"" + USER.info.devid[i] + "\"";
        if (USER.info.devid[i] == USER.devid) {
            s += " selected";
            selected = true;
        }
        s += ">" + USER.info.devid[i] + "</option>";
    }
    s += "</select><input type='button' onclick='goNextDash(previousSibling.value)' value='Switch'></input>";
    document.getElementById("info").innerHTML = s;
}

var TRIPS = {
	xhr: new XMLHttpRequest(),
	startTimeEpoc: null,
	startDeviceTick: null,
	histroy: null,
	series: [],
	tripID: null,
	loadTrip: function(tripIndex)
	{
		this.xhr.onreadystatechange = function () {
			if (this.readyState != 4) return;
			if (this.status != 200) {
				if (this.status >= 500) {
					alert("Server under maintenance (status: " + this.status + ")");
				}
				return;
			}
			var data = JSON.parse(this.responseText);
			if (!data || !data.trip) {
				document.getElementById("trips").innerHTML = "No location data";
				document.getElementById("chart").style.display = "none";
				OSMAP.clear();
				return;
			}
			var length = data.trip.coordinates.length;
			var features = new Array;
			var tripDate = TRIPS.tripID.substr(0, 8);
			var startTime = getDateFromGPSFormat(data.stats.start.date ? data.stats.start.date : tripDate, data.stats.start.time);
			var finishTime = getDateFromGPSFormat(data.stats.end.date ? data.stats.end.date : tripDate, data.stats.end.time);
			TRIPS.startTimeEpoc = Date.parse(startTime.toISOString()) - startTime.getTimezoneOffset() * 60000;
			TRIPS.startDeviceTick = data.stats.start.ts;

			features.push({
				type: "Feature", properties: { name: "Start", info: startTime }, geometry: { type: "Point", coordinates: data.trip.coordinates[0] }
			});
			features.push({
				type: "Feature", properties: { name: "Finish", info: finishTime }, geometry: { type: "Point", coordinates: data.trip.coordinates[length - 1] }
			});

			var index;
			var indexTopSpeed;
			var topSpeed = 0;
			var longestStopping = 0;
			var totalStopping = 0;
			for (var i = 0; i < length; i++) {
				var speed = data.trip.speeds[i];
				if (index == null && speed == 0) {
					index = i;
				} else if (index != null && speed > 1) {
					var duration = Math.floor((data.trip.timestamps[i] - data.trip.timestamps[index]) / 1000);
					if (duration > longestStopping) longestStopping = duration;
					if (duration >= STOP_TIME_MIN) {
						// stopped
						features.push({
							type: "Feature", properties: { name: "Stopping", info: duration + " seconds" }, geometry: { type: "Point", coordinates: data.trip.coordinates[index] }
						});
					}
					totalStopping += duration;
					index = null;
				}
				if (speed > topSpeed) {
					topSpeed = speed;
					indexTopSpeed = i;
				}
			}
			/*
			features.push({
				type: "Feature", properties: { name: "Top Speed", info: topSpeed + " km/h" }, geometry: { type: "Point", coordinates: data.trip.coordinates[indexTopSpeed] }
			});
			*/
        
			// generate popup info
			var info = "Distance: " + (data.stats.distance / 1000).toFixed(1) + " km";
			var duration = Math.floor((data.stats.end.ts - data.stats.start.ts) / 1000);
			var hr = Math.floor(duration / 3600);
			var min = Math.floor((duration % 3600) / 60);
			var sec = duration % 60;
			info += "<br/>Duration: ";
			if (hr) info += hr + " hours ";
			if (hr || min) info += min + " minutes ";
			if (hr == 0 && sec) info += sec + " seconds";

			if (topSpeed >= 1) info += "<br/><br/>Top Speed: " + topSpeed + " km/h";
			info += "<br/>Average Speed: " + (data.stats.distance / duration * 3600 / 1000).toFixed(1) + " km/h";
			if (totalStopping) info += "<br/>Total Stopping Time: " + (totalStopping / 60).toFixed(0) + " minutes";
			info += "<br/>Max Stopping Time: " + longestStopping + " seconds";

			// generate summary
			var summ = "<hr/>";
			summ += "<a onclick=\"TRIPS.list()\"><strong>" + startTime.toDateString() + "</strong></a>";
			var tm = startTime.toTimeString();
			summ += "<br/>" + tm.substr(0, tm.indexOf(" ")) + "~";
			tm = finishTime.toTimeString();
			summ += tm.substr(0, tm.indexOf(" "));
			summ += "<br/><br/>Distance: " + (data.stats.distance / 1000).toFixed(1) + " km";
			summ += "<br/>Duration: ";
			if (hr) summ += hr + ":";
			summ += (min < 10 ? "0" : "") + min + ":";
			summ += (sec < 10 ? "0" : "") + sec;

			if (totalStopping) summ += "<br/>Stopping: " + (totalStopping / 60).toFixed(0) + " minutes";
			summ += "<br/>Average Speed: " + (data.stats.distance / duration * 3600 / 1000).toFixed(1) + " km/h";
			if (topSpeed >= 1) summ += "<br/>Top Speed: " + topSpeed + " km/h";

			// PID list for chart data
			summ += "<br/><br/>Chart Data<br/>";
			for (var j = 0; j < 2; j++) {
				summ += "<select id='chartPID" + j + "'>";
				for (var i = 0; i < data.pids.length; i++) {
					if (PID.illustratable(data.pids[i])) {
						summ += "<option value=\"" + data.pids[i] + "\">" + PID.getName(data.pids[i]) + "</option>";
					}
				}
				summ += "</select>";
			}
			summ += "<input type='button' value='Update' onclick='TRIPS.loadChart([parseInt(previousSibling.previousSibling.value),parseInt(previousSibling.value)])'/>";
			summ += "<hr/>"

			document.getElementById("trips").innerHTML = summ;
			document.getElementById("chartPID1").selectedIndex = 1;

			if (OSMAP.map) {
				OSMAP.clear();
			} else {
				OSMAP.init("map", data.bounds[0].lat, data.bounds[0].lng, 15);
			}
			OSMAP.line(data.trip, myStyle, info);
			OSMAP.map.fitBounds(data.bounds);
			OSMAP.features(features, myStyle);

			//TRIPS.series = [TRIPS.getTimestampedArray(data.trip.speeds, data.trip.timestamps),
			//	TRIPS.getTimestampedArray(data.trip.altitudes, data.trip.timestamps)];

			TRIPS.loadChart(
				[parseInt(document.getElementById("chartPID0").value), parseInt(document.getElementById("chartPID1").value)]
			);
		}

		this.tripID = this.history[tripIndex].id;

		var html = "";
		html += "<a href='" + serverURL + "trip?devid=" + USER.devid + "&tripid=" + this.tripID + "' target='_blank'>JSON</a> | " 
		html += "<a href='" + serverURL + "trip/kml?devid=" + USER.devid + "&tripid=" + this.tripID + "' target='_blank'>KML</a> | " 
		html += "<a href='" + serverURL + "trip/raw?devid=" + USER.devid + "&tripid=" + this.tripID + "' target='_blank'>RAW</a><hr/>";

		if (tripIndex < TRIPS.history.length - 1) {
			html += "<input type='button' value='Previous' onclick='TRIPS.loadTrip(" + (tripIndex + 1) + ")'></input>";
		} else {
			html += "<input type='button' value='Previous' disabled></input>";
		}
		if (tripIndex > 0) {
			html += "<input type='button' value='Next' onclick='TRIPS.loadTrip(" + (tripIndex - 1) + ")'></input>";
		} else {
			html += "<input type='button' value='Next' disabled></input>";
		}
		html += "<br/><br/><input type='button' onclick='TRIPS.showList()' value='Back'></input>";
		document.getElementById("toolbar").innerHTML = html;

		document.getElementById("chart").style.display = "block";
		var url = serverURL + "trip?devid=" + USER.devid + "&tripid=" + this.tripID;
		this.xhr.open('GET', url, true);
		this.xhr.send(null);
	},
	loadChart: function(pids)
	{
		var offset = this.startTimeEpoc - this.startDeviceTick;
		var names = [];
		this.series = [];
		for (var i = 0; i < pids.length; i++) {
			var series = transport.getJSON(serverURL + "data?devid=" + USER.devid + "&tripid=" + this.tripID + "&offset=" + offset + "&pid=" + pids[i]);
			if (series) {
				this.series.push(series);
				names.push(PID.getName(pids[i]));
			}
		}		
		chart = ShowChart("chart", names, this.series);	
	},
	list: function ()
	{
		this.xhr.onreadystatechange = function () {
			if (this.readyState != 4) return;
			if (this.status != 200) {
				if (this.status >= 500) {
					alert("Server under maintenance (status: " + this.status + ")");
				}
				return;
			}
			TRIPS.history = JSON.parse(this.responseText);
			TRIPS.history.sort(function(a, b) { return b.key - a.key; }	);
			TRIPS.showList();
		}
		// local date and time
		var begin = document.getElementById("begin").value;
		var end = document.getElementById("end").value;
		var url = serverURL + "history?devid=" + USER.devid + "&begin=" + getISOFromLocal(parseInt(begin), 0) + "&end=" + getISOFromLocal(parseInt(end), 235959);
		//alert(url);
		this.xhr.open('GET', url, true);
		this.xhr.send(null);
	},
	showList: function ()
	{
		var html = "";
		var dateStr;
		document.getElementById("toolbar").innerHTML = "";
		for (var i = 0; i < this.history.length; i++) {
			var d = new Date(this.history[i].utc);
			if (dateStr != d.toDateString()) {
				dateStr = d.toDateString();
				html += "<br/><div><strong>" + dateStr + "</strong></div>";
			}
			var duration = Math.floor(this.history[i].duration / 1000);
			// strip out very short trips
			if (duration < 30) continue;
			var str;
			if (duration > 3600) {
				str = Math.floor(duration / 3600) + "h " + Math.floor((duration / 60) % 60) + "m";
			} else {
				str = Math.floor(duration / 60) + "m " + Math.floor(duration % 60) + "s";
			}
			var tm = d.toTimeString();
			html += "<div class='link'><a onclick=\"TRIPS.loadTrip(" + i + ")\">" + tm.substr(0, tm.indexOf(" ")) + " ("  + str +")</a></div>";
		}
		document.getElementById("trips").innerHTML = html;
	},
	getTimestampedArray: function(data, ts)
	{
		var result = new Array;
		var element = [0, 0];
		for (var i = 0; i < data.length; i++) {
			result.push([ts[i] + this.startTimeEpoc, data[i]]);
		}
		return result;
	},
	showMarkerFromTime: function(t)
	{
		var ts = t - this.startTimeEpoc;
		for (var i = 0; i < data.trip.timestamps.length; i++) {
			if (data.trip.timestamps[i] >= ts) {
				OSMAP.setMarker(0, [data.trip.coordinates[i][1], data.trip.coordinates[i][0]]);
				var info = Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', t) + "<br/>" + 
					data.trip.speeds[i] + " " + data.trip.altitudes[i];
				OSMAP.popupMarker(0, info);
				break;
			}
		}
	}
}

function getShortDate(d)
{
	var year = d.getFullYear();
	var month = d.getMonth() + 1;
	var day = d.getDate();
	return year + (month < 10 ? "0" : "") + month + (day < 10 ? "0" : "") + day;	
}

function setRange(range)
{
	var d = new Date();
	var begin;
	var end = getShortDate(d);
	switch (range) {
	case "today":
		begin = end;
		break;
	case "yesterday":
		d.setTime(Date.now() - 86400000);
		end = begin = getShortDate(d);
		break;
	case "3days":
		d.setTime(Date.now() - 86400000 * 2);
		begin = getShortDate(d);
		break;
	case "7days":
		d.setTime(Date.now() - 86400000 * 6);
		begin = getShortDate(d);
		break;
	case "30days":
		d.setTime(Date.now() - 86400000 * 29);
		begin = getShortDate(d);
		break;
	case "this_week":
		d.setTime(Date.now() - 86400000 * d.getDay());
		begin = getShortDate(d);
		break;		
	case "last_week":
		d.setTime(Date.now() - 86400000 * (d.getDay() + 7));
		begin = getShortDate(d);
		d.setTime(Date.now() - 86400000 * (d.getDay() + 1));
		end = getShortDate(d);
		break;		
	case "this_month":
		d.setTime(Date.now() - 86400000 * (d.getDate() - 1));
		begin = getShortDate(d);
		break;
	case "custom":
		document.getElementById("range").style.display = "block";
	default:
		return;
	}
	document.getElementById("range").style.display = "none";
	document.getElementById("begin").value = begin;
	document.getElementById("end").value = end;
}

function onResize()
{
    var height = window.innerHeight - 8;
    var width = window.innerWidth - document.getElementById("sidebar").offsetWidth - 2;
    //if (mapHeight < 300) mapHeight = 300;
    document.getElementById("container").style.height = height + "px";
    document.getElementById("map").style.height = height * 2 / 3 + "px";
    document.getElementById("map").style.width = width + "px";
    document.getElementById("chart").style.height = height / 3 + "px";
    document.getElementById("chart").style.width = width+ "px";
}

