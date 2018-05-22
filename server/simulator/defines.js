var FLAG_DRIVING = 0x1;

var PID = {
    TS: 0,
	SPEED: 0x10d,
	RPM: 0x10c,
	ENGINE_LOAD: 0x104,
	THROTTLE: 0x111,
	COOLANT_TEMP: 0x105,
	INTAKE_TEMP: 0x10F,
	GPS : {
		LATITUDE: 0xA,
		LONGITUDE: 0xB,
		ALTITUDE: 0xC,
		SPEED: 0xD,
		HEADING: 0xE,
		SAT_COUNT: 0xF,
		TIME: 0x10,
		DATE: 0x11
	},
	ACC: 0x20,
	GYRO: 0x21,
	COMPASS: 0x22,
	MEMS_TEMP: 0x23,
	BATTERY_VOLTAGE: 0x24,
	ORIENTATION: 0x25,
	TRIP_DISTANCE: 0x30,
	DEVICE_TEMP: 0x82,
	getNameUnit: function(pid)
	{
		switch (pid) {
		case this.SPEED:
			return "Speed (km/h)";
		case this.RPM:
			return "Engine RPM";
		case this.ENGINE_LOAD:
			return "Engine Load (%)";
		case this.THROTTLE:
			return "Throttle Position (%)";
		case this.COOLANT_TEMP:
			return "Coolant Temp. (°C)";
		case this.INTAKE_TEMP:
			return "Intake Temp. (°C)";
		case this.BATTERY_VOLTAGE:
			return "Battery Voltage (V)";
		case this.TRIP_DISTANCE:
			return "Trip Distance (km)";
		case this.DEVICE_TEMP:
			return "Device Temp. (°C)";
		case this.ACC:
			return "Acceleration (g)";
		case this.ORIENTATION:
			return "Orientation (yaw/pitch/roll)";
		case this.GPS.LATITUDE:
			return "Latitude";
		case this.GPS.LONGITUDE:
			return "Longitude";
		case this.GPS.ALTITUDE:
			return "Altitude (m)";
		case this.GPS.SPEED:
			return "GPS Speed (km/h)";
		case this.GPS.HEADING:
			return "Course (°)";
		case this.GPS.SAT_COUNT:
			return "Satellites in Use";
		case this.GPS.TIME:
			return "UTC Time";
		case this.GPS.DATE:
			return "UTC Date";
		}
		return "PID 0x" + (pid.toString(16));
	},
	getUTCTime: function (t)
	{
		var hours   = Math.floor(t / 1000000);
		var minutes = Math.floor((t % 1000000) / 10000);
		var seconds = Math.floor((t % 10000) / 100);
		var ms = t % 100;
		if (hours   < 10) {hours   = "0"+hours;}
		if (minutes < 10) {minutes = "0"+minutes;}
		if (seconds < 10) {seconds = "0"+seconds;}
		if (ms < 10) {ms = "0"+ms;}
		return hours+':'+minutes+':'+seconds+'.'+ms;
	},
	getUTCDate: function (d)
	{
		var day = Math.floor(d / 10000);
		var mon = Math.floor((d % 10000) / 100);
		var year = d % 100; 
		return "20" + year + "-" + mon + "-" + day;
	},
	getXYZ: function(s)
	{
		var axis = s.split(";");
		if (axis.length == 3) {
		    return "X:" + (axis[0] / 100).toFixed(1) + " Y:" + (axis[1] / 100).toFixed(1) + " Z:" + (axis[2] / 100).toFixed(1);
		} else {
			return s;
		}
	},
	getXYZComposed: function(s)
	{
		var axis = s.split(";");
		if (axis.length == 3) {
			return (axis[0] / 100) ^ 2  + (axis[1] / 100) ^ 2 + (axis[2] / 100) ^ 2;
		} else {
			return 0;
		}
	},
	normalize: function (pid, value)
	{
		switch (pid) {
			case this.BATTERY_VOLTAGE:
				return value / 100;
			case this.TRIP_DISTANCE:
				return (value / 1000).toFixed(1);
			case this.GPS.ALTITUDE:
				return (value / 100).toFixed(1);
			case this.GPS.TIME:
				return this.getUTCTime(value);
			case this.GPS.DATE:
				return this.getUTCDate(value);
			case this.ACC:
			case this.GYRO:
			case this.COMPASS:
			case this.ORIENTATION:
				return this.getXYZ(value);
			case this.GPS.LATITUDE:
			case this.GPS.LONGITUDE:
				return parseFloat(value);
		}
		return value;
	},
	toNumber: function (pid, value)
	{
		switch (pid) {
			case this.BATTERY_VOLTAGE:
				return value / 100;
			case this.TRIP_DISTANCE:
				return value / 1000;
			case this.GPS.ALTITUDE:
				return value / 100;
			case this.ACC:
			case this.GYRO:
				return this.getXYZComposed(value);
		}
		return typeof value != "number" ? parseFloat(value) : value;
	}
};