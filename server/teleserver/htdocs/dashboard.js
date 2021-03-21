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

onResize();

$(function () {
  $(document).ready(function () {
    USER.load('DASH.load()');

    self.setInterval(function () {
      if (DASH.curLocation && OSMAP.map) OSMAP.setCenter(DASH.curLocation);
    }, MAP_CENTERING_INTERVAL);

    /*
		var options = {
            enableHighAccuracy: true,
            timeout: 10000,
            maximumAge: 10000,
        };
        navigator.geolocation.getCurrentPosition(
            function (pos) {
                if (!DASH.curLocation) OSMAP.setCenter([pos.coords.latitude, pos.coords.longitude]);
            }, null, options);
        */
  });
});

if (window.require) {
  const shell = require('electron').shell;

  function openLink(url) {
    shell.openExternal(url);
  }
}

function onResize() {
  var height =
    window.innerHeight - document.getElementById('list').offsetHeight - 8;
  var width =
    window.innerWidth - document.getElementById('sidebar').offsetWidth - 2;
  //if (mapHeight < 300) mapHeight = 300;
  document.getElementById('container').style.height = height + 'px';
  document.getElementById('map').style.height = (height * 2) / 3 + 'px';
  document.getElementById('map').style.width = width + 'px';
  document.getElementById('chart').style.height = height / 3 + 'px';
  document.getElementById('chart').style.width = width + 'px';
}

var DASH = {
  xhr: new XMLHttpRequest(),
  dataSlideIndex: [0, 1],
  deviceID: null,
  curLocation: null,
  data: null,
  chart: null,
  chartPID: null,
  chartDataTick: 0,
  selectedPID: 269,
  lastDataCount: null,
  parked: null,
  selectPID: function (pid) {
    this.selectedPID = pid;
  },
  setText: function (name, text) {
    document.getElementById('data_' + name).innerText = text;
  },
  setClass: function (name, className) {
    document.getElementById('data_' + name).className = className;
  },
  setHTML: function (name, html) {
    document.getElementById('data_' + name).innerHTML = html;
  },
  setTempBar: function (name, temp) {
    if (temp < 0) temp = 0;
    if (temp > 80) temp = 80;
    var i = Math.floor((temp / 80) * 45) * 34;
    var img = document.getElementById('data_' + name);
    img.style.marginLeft = -i + 'px';
    img.title = temp + 'C';
  },
  togglePID: function (num) {
    if (this.data.length > num) {
      if (++this.dataSlideIndex[num] >= this.data.length)
        this.dataSlideIndex[num] = 0;
      if (this.dataSlideIndex[num] == this.dataSlideIndex[1 - num]) {
        if (++this.dataSlideIndex[num] >= this.data.length)
          this.dataSlideIndex[num] = 0;
      }
      this.updatePID(num);
    }
  },
  getPIDValue: function (pid) {
    for (var i = 0; i < this.data.length; i++) {
      if (this.data[i][0] == pid) {
        return PID.normalize(pid, this.data[i][1]);
      }
    }
    return null;
  },
  updatePID: function (num) {
    var dataIndex = this.dataSlideIndex[num];
    if (dataIndex < this.data.length) {
      var pid = this.data[dataIndex][0];
      var value = this.data[dataIndex][1];
      this.setText('pid_value' + num.toString(), PID.normalize(pid, value));
      this.setText('pid_name' + num.toString(), PID.getNameUnit(pid));
    }
  },
  updateUserInfo: function (info, devid) {
    if (!USER.info) {
      document.getElementById('info').innerHTML = !devid
        ? ''
        : "DEVICE: <input type='text' size='7' readonly value='" +
          devid +
          "'/>";
      return;
    }
    var s = "<select onchange='USER.goDash(this.value)'>";
    var found = false;
    for (var i = 0; i < info.devid.length; i++) {
      s += '<option value="' + info.devid[i] + '"';
      if (info.devid[i] == devid) {
        s += ' selected';
        found = true;
      }
      s += '>' + info.devid[i] + '</option>';
    }
    if (!found) {
      s += '<option value="' + devid + '" selected>' + devid + '</option>';
    }
    s +=
      "</select><input type='button' onclick='USER.goNextDash(previousSibling.value)' value='Switch'></input>";
    document.getElementById('info').innerHTML = s;
  },
  pickNewestData: function (data) {
    var index = [-1, -1];
    var ts = 0;
    for (var i = 0; i < data.length; i++) {
      if (ts == 0 || data[i][2] < ts) {
        index[0] = i;
        ts = data[i][2];
      }
    }
    ts = 0;
    for (var i = 0; i < data.length; i++) {
      if (i == index[0]) continue;
      if (ts == 0 || data[i][2] < ts) {
        index[1] = i;
        ts = data[i][2];
      }
    }
    if (index[0] >= 0) this.dataSlideIndex[0] = index[0];
    if (index[1] >= 0) this.dataSlideIndex[1] = index[1];
  },
  update: function (ch) {
    this.parked = ch.stats.parked || ch.stats.age.data > TRIP_END_TIMEOUT;
    if (this.parked) {
      this.setText('elapsed', getHHMM(Math.floor(ch.stats.age.data / 1000)));
      var offline = ch.stats.age.ping > DEVICE_OFFLINE_TIMEOUT;
      if (offline) {
        this.setText('state', 'OFFLINE');
        this.setClass('state', 'state_offline');
      } else {
        this.setText('state', 'PARKED');
        this.setClass('state', 'state_parked');
      }
      this.setText('rate', '-');
      this.setText('delay', '-');
      if (!offline) {
        this.pickNewestData(ch.data);
      }
    } else {
      this.setText('elapsed', getHHMMSS(ch.stats.elapsed));
      this.setText('state', 'RUNNING');
      this.setClass('state', 'state_running');
      this.setText('rate', ch.stats.rate);
      this.setText('delay', ch.stats.age.data);
      this.setText('recv', Math.floor(ch.stats.recv / 1024));
    }

    this.data = ch.live;

    var deviceTemp = this.getPIDValue(PID.DEVICE_TEMP);
    if (deviceTemp != null) {
      this.setTempBar('temp', deviceTemp);
    }

    this.updatePID(0);
    this.updatePID(1);

    // update data grid
    var s = '<hr/>';
    if (this.deviceFlags && (this.deviceFlags & 0xf000) == 0x1000) {
      if (this.deviceFlags & 0x1) s += '[OBD]';
      if (this.deviceFlags & (0x2 | 0x4)) s += '[GNSS]';
      if (this.deviceFlags & 0x8) s += '[MEMS] ';
      s += '<br/>';
    }

    s += "<span class='smaller_text'>Timestamp </span>" + ch.stats.devtick;

    if (this.deviceRSSI) {
      s +=
        "<br/><span class='smaller_text'>RSSI </span>" +
        this.deviceRSSI +
        'dBm';
    }

    for (var n = 0; n < this.data.length; n++) {
      var pid = this.data[n][0];
      var value = this.data[n][1];
      s +=
        "<br/><span class='smaller_text'>" +
        PID.getName(pid) +
        ' </span>' +
        PID.normalize(pid, value);
      var unit = PID.getUnit(pid);
      if (unit) s += "<span class='small_text'> " + unit + '</span>';
    }
    document.getElementById('grid').innerHTML = s;

    if (this.lastDataCount != this.data.length) {
      s =
        "<hr/>Chart Data<br/><select id='chartPIDselect' onchange='DASH.selectPID(parseInt(value))'>";
      for (var n = 0; n < this.data.length; n++) {
        var pid = this.data[n][0];
        if (!PID.illustratable(pid)) continue;
        s += "<option value='" + pid + "'";
        if (pid == this.selectedPID) s += ' selected';
        s += '>' + PID.getName(pid) + '</option>';
      }
      s += '</select>';
      document.getElementById('tools').innerHTML = s;
      this.lastDataCount = this.data.length;
      this.selectedPID = parseInt(
        document.getElementById('chartPIDselect').value
      );
    }

    // update map
    var lat = this.getPIDValue(PID.GPS.LATITUDE);
    var lng = this.getPIDValue(PID.GPS.LONGITUDE);
    if (lat != null && lng != null && lat != 0 && lng != 0) {
      if (!OSMAP.map) OSMAP.init('map', lat, lng, 15);
      //if (devid) OSMAP.setTooltip(0, devid);
      if (
        !this.curLocation ||
        this.curLocation[0] != lat ||
        this.curLocation[1] != lng
      ) {
        this.curLocation = [lat, lng];
        OSMAP.setMarker(0, this.curLocation);
        OSMAP.setCenter(this.curLocation);
        if (OPENCAGE_API_KEY) {
          // console.log('here I can reverse geocode', lat, lng);
          var url =
            'https://api.opencagedata.com/geocode/v1/json?q=' +
            lat +
            ',' +
            lng +
            '&no_annotations=1&key=' +
            OPENCAGE_API_KEY;

          // geocodeResult success or error ?
          var geocodeResult = transport.getJSON(url);
          if (geocodeResult && Array.isArray(geocodeResult.results)) {
            var address = geocodeResult.results[0].formatted;
            OSMAP.popupMarker(0, address);
          }
        }
      }
    }
  },
  load: function () {
    this.updateUserInfo(USER.info, USER.devid);
    this.lastDataCount = null;
    // load channel data
    this.xhr.onreadystatechange = function () {
      if (this.readyState != 4) return;
      if (this.status != 200) {
        if (this.status) {
          alert('Server under maintenance (status: ' + this.status + ')');
        }
        return;
      }
      var chdata = JSON.parse(this.responseText);
      if (chdata && chdata.id) {
        DASH.deviceID = chdata.devid;
        DASH.deviceFlags = chdata.flags;
        DASH.deviceRSSI = chdata.rssi;
        self.setTimeout('DASH.showChart()', 0);
      } else {
        alert(
          'Not an active device. Please check if your device is working or the device ID is correct.'
        );
      }
    };
    var url = serverURL + 'channels/' + USER.devid;
    this.xhr.open('GET', url, true);
    this.xhr.send(null);
  },
  showChart: function () {
    var pid = this.selectedPID;
    this.xhr.onreadystatechange = function () {
      if (this.readyState != 4 || this.status != 200) {
        return;
      }

      var pull = JSON.parse(this.responseText);

      if (pull.error) {
        alert(pull.error);
        return;
      }

      var mydata = [];
      // load history data;
      var d = new Date();
      var tm = d.getTime() - d.getTimezoneOffset() * 60000;
      var lastDataTick = pull.stats.devtick;
      for (var i = 0; i < pull.data.length; i++) {
        var value = pull.data[i][2];
        var ts = pull.data[i][0];
        mydata.push({
          x: tm - (lastDataTick - ts),
          y: PID.toNumber(pid, value),
        });
      }
      // create chart with loaded data
      var chart = CreateChart(
        'chart',
        PID.getName(pid),
        '#808080',
        PID.getUnit(pid),
        PID.getNameUnit(pid),
        100,
        mydata
      );
      // start receiving
      chart.series[0].setVisible(true, true);
      DASH.chart = chart;
      DASH.chartPID = pid;
      DASH.chartDataTick = lastDataTick;
      DASH.update(pull);
      self.setTimeout('DASH.updateData()', DATA_FETCH_INTERVAL);
      //requestData();
    };

    this.chart = null;
    document.getElementById('chart').innerHTML = '';
    var rollback = this.parked ? ROLLBACK_TIME_PARKED : ROLLBACK_TIME;
    this.xhr.open(
      'GET',
      serverURL +
        'pull/' +
        this.deviceID +
        '?pid=' +
        pid +
        '&rollback=' +
        rollback,
      true
    );
    this.xhr.send(null);
  },
  updateData: function () {
    this.xhr.onreadystatechange = function () {
      if (this.readyState != 4) return;
      if (this.status != 200) {
        if (this.status) {
          alert('Server under maintenance (status: ' + this.status + ')');
        }
        return;
      }
      var pull = JSON.parse(this.responseText);
      if (pull.data.length > 0) {
        var lastDataTick = pull.stats.devtick;
        var d = new Date();
        var tm = d.getTime() - d.getTimezoneOffset() * 60000;
        for (var i = 0; i < pull.data.length; i++) {
          var ts = pull.data[i][0];
          var pid = pull.data[i][1];
          var value = pull.data[i][2];
          var x = tm - (lastDataTick - ts);
          var y = PID.toNumber(pid, value);
          DASH.chart.series[0].addPoint([x, y], false, true);
        }
        DASH.chartDataTick = lastDataTick;
        DASH.chart.series[0].setVisible(true, true);
      }
      DASH.update(pull);
      self.setTimeout('DASH.updateData()', DATA_FETCH_INTERVAL);
    };
    if (this.selectedPID != this.chartPID) {
      this.showChart();
      return;
    }
    if (!this.chartPID || !this.chart) return;
    var url =
      serverURL +
      'pull/' +
      this.deviceID +
      '?pid=' +
      DASH.chartPID +
      '&ts=' +
      (this.chartDataTick + 1);
    this.xhr.open('GET', url, true);
    this.xhr.send(null);
  },
};
