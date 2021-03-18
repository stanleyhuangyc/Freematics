var PID = {
  TS: 0,
  FUEL_PRESSURE: 0x10a,
  SPEED: 0x10d,
  RPM: 0x10c,
  ENGINE_LOAD: 0x104,
  THROTTLE: 0x111,
  COOLANT_TEMP: 0x105,
  TIMING_ADVANCE: 0x10e,
  INTAKE_TEMP: 0x10f,
  PID_DISTANCE: 0x131,
  PID_BAROMETRIC: 0x133,
  PID_AMBIENT_TEMP: 0x146,
  PID_FUEL_RATE: 0x15e,
  GPS: {
    LATITUDE: 0xa,
    LONGITUDE: 0xb,
    ALTITUDE: 0xc,
    SPEED: 0xd,
    HEADING: 0xe,
    SAT_COUNT: 0xf,
    TIME: 0x10,
    DATE: 0x11,
    HDOP: 0x12,
  },
  ACC: 0x20,
  GYRO: 0x21,
  COMPASS: 0x22,
  MEMS_TEMP: 0x23,
  BATTERY_VOLTAGE: 0x24,
  ORIENTATION: 0x25,
  TRIP_DISTANCE: 0x30,
  DEVICE_TEMP: 0x82,
  HALL_SENSOR: 0x83,
  getName: function (pid) {
    switch (pid) {
      case this.SPEED:
        return 'Vehicle Speed';
      case this.RPM:
        return 'Engine RPM';
      case this.FUEL_PRESSURE:
        return 'Fuel Pressure';
      case this.TIMING_ADVANCE:
        return 'Timing Advance';
      case this.ENGINE_LOAD:
        return 'Engine Load';
      case this.THROTTLE:
        return 'Throttle Position';
      case this.COOLANT_TEMP:
        return 'Coolant Temp.';
      case this.INTAKE_TEMP:
        return 'Intake Temp.';
      case this.PID_AMBIENT_TEMP:
        return 'Ambient Temp.';
      case this.PID_BAROMETRIC:
        return 'Barometric';
      case this.PID_DISTANCE:
        return 'Distance Since Service';
      case this.PID_FUEL_RATE:
        return 'Fuel Rate';
      case this.BATTERY_VOLTAGE:
        return 'Battery Voltage';
      case this.TRIP_DISTANCE:
        return 'Trip Distance';
      case this.DEVICE_TEMP:
        return 'Device Temp.';
      case this.ACC:
        return 'Acceleration';
      case this.ORIENTATION:
        return 'Orientation';
      case this.GPS.LATITUDE:
        return 'Latitude';
      case this.GPS.LONGITUDE:
        return 'Longitude';
      case this.GPS.ALTITUDE:
        return 'Altitude';
      case this.GPS.SPEED:
        return 'Speed';
      case this.GPS.HEADING:
        return 'Course';
      case this.GPS.SAT_COUNT:
        return 'Satellites in Use';
      case this.GPS.TIME:
        return 'UTC Time';
      case this.GPS.DATE:
        return 'UTC Date';
      case this.GPS.HDOP:
        return 'HDOP';
      case this.HALL_SENSOR:
        return 'Hall Sensor';
      default:
        return 'PID ' + pid.toString(16);
    }
  },
  getUnit: function (pid) {
    switch (pid) {
      case this.SPEED:
        return 'km/h';
      case this.RPM:
        return 'rpm';
      case this.FUEL_PRESSURE:
        return 'kPa';
      case this.ENGINE_LOAD:
      case this.THROTTLE:
        return '%';
      case this.COOLANT_TEMP:
      case this.INTAKE_TEMP:
      case this.PID_AMBIENT_TEMP:
      case this.DEVICE_TEMP:
        return '°C';
      case this.PID_BAROMETRIC:
        return 'bar';
      case this.PID_DISTANCE:
        return 'km';
      case this.PID_FUEL_RATE:
        return 'L/hr';
      case this.BATTERY_VOLTAGE:
        return 'V';
      case this.TRIP_DISTANCE:
        return 'km';
      case this.ACC:
        return 'g';
      case this.ORIENTATION:
        return 'yaw/pitch/roll';
      case this.GPS.ALTITUDE:
        return 'm';
      case this.GPS.SPEED:
        return 'km/h';
      case this.GPS.HEADING:
      case this.TIMING_ADVANCE:
        return '°';
      default:
        return null;
    }
  },
  getNameUnit: function (pid) {
    var s = this.getName(pid);
    var unit = this.getUnit(pid);
    if (unit) s += ' (' + unit + ')';
    return s;
  },
  getUTCTime: function (t) {
    var hours = Math.floor(t / 1000000);
    var minutes = Math.floor((t % 1000000) / 10000);
    var seconds = Math.floor((t % 10000) / 100);
    var ms = t % 100;
    if (hours < 10) {
      hours = '0' + hours;
    }
    if (minutes < 10) {
      minutes = '0' + minutes;
    }
    if (seconds < 10) {
      seconds = '0' + seconds;
    }
    if (ms < 10) {
      ms = '0' + ms;
    }
    return hours + ':' + minutes + ':' + seconds + '.' + ms;
  },
  getUTCDate: function (d) {
    var day = Math.floor(d / 10000);
    var mon = Math.floor((d % 10000) / 100);
    var year = d % 100;
    return '20' + year + '-' + mon + '-' + day;
  },
  getXYZ: function (d) {
    if (d.length == 3) {
      return (
        (d[0] / 100).toFixed(1) +
        '/' +
        (d[1] / 100).toFixed(1) +
        '/' +
        (d[2] / 100).toFixed(1)
      );
    } else {
      return d;
    }
  },
  getXYZComposed: function (d) {
    if (d.length == 3) {
      return (d[0] / 100) ^ (2 + d[1] / 100) ^ (2 + d[2] / 100) ^ 2;
    } else {
      return 0;
    }
  },
  normalize: function (pid, value) {
    switch (pid) {
      case this.BATTERY_VOLTAGE:
        return value / 100;
      case this.TRIP_DISTANCE:
        return (value / 1000).toFixed(1);
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
      case this.GPS.ALTITUDE:
        return value; /* m */
      case this.GPS.SPEED:
        return parseFloat(value); /* kph */
    }
    return value;
  },
  toNumber: function (pid, value) {
    switch (pid) {
      case this.BATTERY_VOLTAGE:
        return value / 100;
      case this.TRIP_DISTANCE:
        return value / 1000;
      case this.GPS.ALTITUDE:
        return value;
      case this.ACC:
      case this.GYRO:
        return this.getXYZComposed(value);
    }
    return typeof value != 'number' ? parseFloat(value) : value;
  },
  illustratable: function (pid) {
    switch (pid) {
      case 0:
      case PID.GYRO:
      case PID.COMPASS:
      case PID.ORIENTATION:
      case PID.GPS.LATITUDE:
      case PID.GPS.LONGITUDE:
      case PID.GPS.HEADING:
      case PID.GPS.TIME:
      case PID.GPS.DATE:
        return false;
    }
    return true;
  },
};
