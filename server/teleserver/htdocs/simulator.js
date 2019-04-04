/******************************************************************************
* Data feed simulator for Freematics Hub
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

const interval = 500; /* ms */

var battery = navigator.battery || navigator.webkitBattery || navigator.mozBattery;

var SIM = {
    acc: null,
    rotation: null,
	orientation: null,
    loc: null,
    on: false,
    manual: false,
    init: function()
    {
        var options = {
            enableHighAccuracy: false,
            timeout: 10000,
            maximumAge: 10000,
        };

        navigator.geolocation.getCurrentPosition(function (pos) { SIM.loc = pos.coords; }, null, options);

        if (window.DeviceMotionEvent) {
            window.addEventListener('devicemotion', function (eventData) {
                SIM.acc = eventData.acceleration;
                SIM.rotation = eventData.rotationRate;
            }, false);
        }
		
		if (window.DeviceOrientationEvent) {
		  window.addEventListener('deviceorientation', function(eventData) { SIM.orientation = eventData; });
		}

        self.setInterval("SIM.refresh()", interval);
    },
    refresh: function()
    {
        var tick = (new Date()).getTime() % 0xffffffff;
        document.getElementById("ts").value = tick;
        if (this.manual) return;
        if (this.loc) {
            document.getElementById("lat").value = this.loc.latitude.toFixed(6);
            document.getElementById("lng").value = this.loc.longitude.toFixed(6);
        }
        if (this.acc && this.acc.x) {
            document.getElementById("acc_x").value = Math.floor(this.acc.x * 10);
            document.getElementById("acc_y").value = Math.floor(this.acc.y * 10);
            document.getElementById("acc_z").value = Math.floor(this.acc.z * 10);
        }
        if (this.orientation) {
            document.getElementById("ori_alpha").value = Math.floor(this.orientation.alpha * 10);
            document.getElementById("ori_beta").value = Math.floor(this.orientation.beta * 10);
            document.getElementById("ori_gamma").value = Math.floor(this.orientation.gamma * 10);
        }
        if (battery) {
            document.getElementById("battery").value = battery.level;
        }
        if (FEED.connected() && this.on) {
            this.feed();
        }
    },
    login: function()
    {
        var devid = document.getElementById("devid").value;
        var ts = document.getElementById("ts").value;
        if (FEED.login(devid, ts)) {
            this.print("Login successfully. Feed ID: " + FEED.id);
        } else {
            this.print("Login failed.");
        }
    },
    logout: function()
    {
        if (FEED.id) {
            if (FEED.logout()) {
                this.print("Logout successful.");
            } else {
                this.print("Logout failed.");
            }
        }
    },
    start: function ()
    {
        if (!FEED.connected()) {
            this.print("Please login first.");
        } else {
            this.on = true;
        }
    },
    stop: function ()
    {
        this.print("");
        this.on = false;
    },
    feed: function()
    {
        if (!FEED.connected()) {
            this.print("Please login first.");
            return false;
        }
        FEED.purge();
        FEED.add(PID.TS, document.getElementById("ts").value);
        FEED.add(PID.GPS.LATITUDE, document.getElementById("lat").value);
        FEED.add(PID.GPS.LONGITUDE, document.getElementById("lng").value);
        var acc_x = document.getElementById("acc_x").value;
        var acc_y = document.getElementById("acc_y").value;
        var acc_z = document.getElementById("acc_z").value;
        if (acc_x != "" && acc_y != "" && acc_z != "") {
            FEED.add(PID.ACC, acc_x + ";" + acc_y + ";" + acc_z);
        }
        var ori_alpha = document.getElementById("ori_alpha").value;
        var ori_beta = document.getElementById("ori_beta").value;
        var ori_gamma = document.getElementById("ori_gamma").value;
        if (ori_alpha != "" && ori_beta != "" && ori_gamma != "") {
            FEED.add(PID.ORIENTATION, ori_alpha + ";" + ori_beta + ";" + ori_gamma);
        }
        var volt = document.getElementById("battery").value;
        if (volt != "") {
            FEED.add(PID.BATTERY_VOLTAGE, parseFloat(volt) * 100);
        }
        FEED.add(PID.DEVICE_TEMP, document.getElementById("device_temp").value);
        if (!FEED.post()) {
            this.print("Error feeding data to server. Network down?");
            return false;
        } else {
            this.print(FEED.payload);
            return true;
        }
    },
    print: function(msg)
    {
        document.getElementById("info").innerText = msg;
    }
};
