var username = getCookie("user");
if (username) {
	document.getElementById("username").value = atob(username);
	document.getElementById("save").checked = true;
} else {
	var devid = getCookie("devid");
	if (devid) {
		document.getElementById("username").value = devid;
		document.getElementById("save").checked = true;
	}
}

function apply(callback)
{
	var input = document.getElementById("username").value;
	if (input == "") {
		alert("Please enter email address or device ID");
		return;
	}
	if (!document.getElementById("save").checked) {
		setCookie("devid", "", 0);
		setCookie("user", "", 0);
	}
	if (input.length <= 8 && input.indexOf("@") < 0 && input.indexOf("#") < 0) {
		// treat as device ID
		USER.devid = input;
		if (document.getElementById("save").checked) {
			setCookie("devid", input, 365);
		}
	} else {
		// treat as user name
		USER.user = input;
		if (document.getElementById("save").checked) {
			setCookie("user", btoa(input), 365);
		}
	}
	USER.check(callback);
}

function goPage(page)
{
	var s = page;
	if (!document.getElementById("save").checked) {
		s += "?";
		if (USER.user)
			s += "user=" + USER.user;
		if (USER.devid)
			s += "&devid=" + USER.devid;
	}
	location.href = s;
}

function goDash()
{
	goPage("dashboard.html");
}

function goTrips()
{
	goPage("trips.html");
}

function goParking()
{
	goPage("parking.html");
}

function goTraccar()
{
	var expire = document.getElementById("save").checked ? 365 : 1;
	setCookie("user", USER.userb64, expire);
	setCookie("password", USER.info.traccar, expire);
	location.href = "traccar/";
}
