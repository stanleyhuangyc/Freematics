const ROLLBACK_TIME = 120000;
const LIST_REFRESH_INTERVAL = 500;
const PARKED_REFRESH_INTERVAL = 5000;
const DATA_FETCH_INTERVAL = 500;
const DATA_FETCH_INTERVAL_PARKED = 10000;
const DATA_SLIDE_INTERVAL = 5000;
const TRIP_END_TIMEOUT = 180000;
const DEVICE_OFFLINE_TIMEOUT = 910000;
const MAP_CENTERING_INTERVAL = 5000;
const STOP_TIME_MIN = 30; /* seconds */

const serverURL = window.require ? "http://localhost:8080/api/" : (window.location.href.indexOf("localhost") > 0 ? "/api/" : "/hub/api/");
