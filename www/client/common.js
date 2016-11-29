var isIE = navigator.userAgent.toLowerCase().indexOf("msie") > -1;
var isMoz = document.implementation && document.implementation.createDocument;
var xmlhttp = newHttpRequest();

function newHttpRequest()
{
	var _xmlhttp;
	if (window.XMLHttpRequest)
	  {// code for all new browsers
	  _xmlhttp=new XMLHttpRequest();
	  }
	else if (window.ActiveXObject)
	  {// code for IE5 and IE6
	  _xmlhttp=new ActiveXObject("Microsoft.XMLHTTP");
	  }
	 else
	  {
	  alert("Your browser does not support XMLHTTP.");
	  }
	  return _xmlhttp;
}

function transformXML(xmlDoc, xslDoc, element)
{
	if (isIE) {
		var e = document.getElementById(element);
		if (e) {
			var content = xmlDoc.transformNode(xslDoc);
			e.innerHTML = content;
		}
	} else {
		var XSLT = new XSLTProcessor;
		XSLT.importStylesheet(xslDoc);
		var e = document.getElementById(element);
		if (e) {
			e.innerHTML = "";
			e.appendChild(XSLT.transformToFragment(xmlDoc, document));
		}
	}
}

function loadXML(url)
{
	xmlhttp.open('GET', url, false);
	xmlhttp.send(null);
	return xmlhttp.responseXML;
}

function loadJSON(url)
{
	xmlhttp.open('GET', url, false);
	xmlhttp.send(null);
	if (xmlhttp.status == 200)
	    return JSON.parse(xmlhttp.responseText);
	else
	    return null;
}

function httpGet(url)
{
	xmlhttp.open("GET", url, false);
	xmlhttp.send();
	if (xmlhttp.status == 200)
		return xmlhttp.responseText;
	else
		return null;
}

function getNodeValue(xml, tagname)
{
	var tags = xml.getElementsByTagName(tagname);
	if (tags.length > 0)
		return tags[0].firstChild ? tags[0].firstChild.nodeValue : null;
	else
		return null;
}

function showCtrl(id)
{
	document.getElementById(id).style.display = "block";
}

function hideCtrl(id)
{
	document.getElementById(id).style.display = "none";
}

function enableCtrl(id)
{
	document.getElementById(id).disabled = false;
}

function disableCtrl(id)
{
	document.getElementById(id).disabled = true;
}

function setValue(id, value)
{
	document.getElementById(id).value = value;
}

function setContent(id, content)
{
	document.getElementById(id).innerHTML = content;
}

function getUrlArg(name)
{
	var idx=document.location.href.indexOf(name+'=');
	if (idx<=0) return null;
	var argstr=document.location.href.substring(idx+name.length+1);
	idx = argstr.indexOf('&');
	var token = idx>=0?argstr.substring(0, idx):argstr;
	return token.replace('+', ' ', 'g');
}