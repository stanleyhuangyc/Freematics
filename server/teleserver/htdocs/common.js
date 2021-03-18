var transport = {
  xhr: new XMLHttpRequest(),
  get: function (url) {
    this.xhr.open('GET', xmlFile, false);
    this.xhr.send(null);
    return this.xhr.status == 200 ? this.xhr.responseText : null;
  },
  getXML: function (url) {
    this.xhr.open('GET', url, false);
    this.xhr.overrideMimeType('text/xml');
    this.xhr.send(null);
    return this.xhr.status == 200 ? this.xhr.responseXML : null;
  },
  getJSON: function (url) {
    this.xhr.open('GET', url, false);
    this.xhr.overrideMimeType('application/json');
    this.xhr.send(null);
    return this.xhr.status == 200 ? JSON.parse(this.xhr.responseText) : null;
  },
};

function transformXML(xmlDoc, xslDoc, element) {
  var XSLT = new XSLTProcessor();
  XSLT.importStylesheet(xslDoc);
  var e = document.getElementById(element);
  if (e) {
    e.innerHTML = '';
    e.appendChild(XSLT.transformToFragment(xmlDoc, document));
  }
}

function getNodeValue(xml, tagname) {
  var tags = xml.getElementsByTagName(tagname);
  if (tags.length > 0)
    return tags[0].firstChild ? tags[0].firstChild.nodeValue : null;
  else return null;
}

function showCtrl(id) {
  document.getElementById(id).style.display = 'block';
}

function hideCtrl(id) {
  document.getElementById(id).style.display = 'none';
}

function enableCtrl(id) {
  document.getElementById(id).disabled = false;
}

function disableCtrl(id) {
  document.getElementById(id).disabled = true;
}

function setValue(id, value) {
  document.getElementById(id).value = value;
}

function setContent(id, content) {
  document.getElementById(id).innerHTML = content;
}

function getUrlArg(name) {
  var idx = document.location.href.indexOf(name + '=');
  if (idx <= 0) return null;
  var argstr = document.location.href.substring(idx + name.length + 1);
  idx = argstr.indexOf('&');
  var token = idx >= 0 ? argstr.substring(0, idx) : argstr;
  return token.replace('+', ' ', 'g');
}

function getTickCount() {
  var d = new Date();
  return d.getTime();
}

function getHHMM(sec) {
  var hours = Math.floor(sec / 3600);
  var minutes = Math.floor((sec - hours * 3600) / 60);
  return hours > 0 ? hours + 'h ' + minutes + 'm' : minutes + ' mins';
}

function getHHMMSS(sec) {
  var hours = Math.floor(sec / 3600);
  var minutes = Math.floor((sec - hours * 3600) / 60);
  var seconds = sec - hours * 3600 - minutes * 60;
  if (hours < 10) {
    hours = '0' + hours;
  }
  if (minutes < 10) {
    minutes = '0' + minutes;
  }
  if (seconds < 10) {
    seconds = '0' + seconds;
  }
  return hours + ':' + minutes + ':' + seconds;
}

function getCookie(cname) {
  var name = cname + '=';
  var decodedCookie = decodeURIComponent(document.cookie);
  var ca = decodedCookie.split(';');
  for (var i = 0; i < ca.length; i++) {
    var c = ca[i];
    while (c.charAt(0) == ' ') {
      c = c.substring(1);
    }
    if (c.indexOf(name) == 0) {
      return c.substring(name.length, c.length);
    }
  }
  return null;
}

function setCookie(cname, cvalue, exdays) {
  var d = new Date();
  d.setTime(d.getTime() + exdays * 24 * 60 * 60 * 1000);
  var expires = 'expires=' + d.toUTCString();
  document.cookie = cname + '=' + cvalue + ';' + expires + ';path=/';
}
