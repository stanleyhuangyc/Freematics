var OSMAP = {
    map: null,
    marker: new Array,
    popup: new Array,
    loc: null,
    setMarker: function (index, latlng) {
        if (!this.marker[index]) {
            this.marker[index] = L.marker(latlng)
				.addTo(this.map);
        } else {
            this.marker[index].setLatLng(latlng);
        }
    },
    popupMarker: function (index, text) {
        if (!this.popup[index]) {
            this.popup[index] = this.marker[index].bindPopup(text)
				.openPopup();
            this.popup[index].autoPan = true;
        }
    },
    setTooltip: function (index, text) {
        if (this.marker[index]) {
            this.marker[index].bindTooltip(text).openTooltip();
        }
    },
    setCenter: function(latlng)
    {
        this.map.setView(latlng);
    },
    init: function (div, lat, lng, zoom)
    {
        if (this.map) return;
        this.map = L.map('map').setView([lat, lng], zoom);
        L.tileLayer('https://{s}.tile.osm.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a>'
        }).addTo(this.map);
    },
    getCurrentLocation: function()
    {
        var options = {
            enableHighAccuracy: true,
            timeout: 10000,
            maximumAge: 10000,
        };
        navigator.geolocation.getCurrentPosition(function (pos) { OSMAP.loc = pos.coords; }, null, options);
    }
};
