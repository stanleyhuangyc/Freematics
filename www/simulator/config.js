const REQUEST_INTERVAL = 1000;
const ROLLBACK_TIME = 300000;
const MAX_CACHE_SIZE = 10000;
const DATA_FETCH_INTERVAL = 500;
const DATA_SLIDE_INTERVAL = 5000;
const TRIP_TIMEOUT = 60000;

const serverURL = window.require ? "http://localhost:8080/api/" : "api/";
