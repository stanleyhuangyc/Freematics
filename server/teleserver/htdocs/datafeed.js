const EVENT_LOGIN = 1;
const EVENT_LOGOUT = 2;
const EVENT_SYNC = 3;
const EVENT_CLEAR_COMMAND = 4;

var FEED = {
    xhr: new XMLHttpRequest(),
    id: null,
    payload: "",
    login: function(devid, ts)
    {
        var url = serverURL + "notify/" + devid + "?EV=" + EVENT_LOGIN + "&TS=" + ts;
        this.xhr.open("GET", url, false);
        this.xhr.send(null);
        if (this.xhr.status == 200) {
			this.id = devid;
			return true;
        }
        return false;
    },
    logout: function()
    {
        var url = serverURL + "notify/" + this.id + "?EV=" + EVENT_LOGOUT;
        this.xhr.open("GET", url, false);
        this.xhr.send(null);
        if (this.xhr.status == 200) {
			this.id = null;
			return true;
        }
        return false;
    },
    post: function()
    {
        var url = serverURL + "post/" + this.id;
        this.xhr.open("POST", url, false);
        this.xhr.send(this.payload);
        return this.xhr.status == 200;
    },
    purge: function()
    {
        this.payload = "";
    },
    add: function(pid, value)
    {
        if (value != null && value != "") {
            if (this.payload != "") this.payload += ",";
            this.payload += pid.toString(16) + ":" + value;
        }
    },
    connected: function()
    {
        return this.id != null;
    }
}
