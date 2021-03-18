var USER = {
  xhr: new XMLHttpRequest(),
  devid: getUrlArg('devid'),
  user: getUrlArg('user'),
  userb64: null,
  info: null,
  save: function () {
    setCookie('user', this.userb64, 365);
  },
  load: function (callback) {
    this.xhr.onreadystatechange = function () {
      if (this.readyState != 4) return;
      if (this.status == 404) {
        if (!USER.devid) {
          USER.devid = window.prompt('Please enter your device ID');
        }
        self.setTimeout(callback, 0);
        return;
      } else if (this.status != 200) {
        if (this.status) {
          alert('Server under maintenance (status: ' + this.status + ')');
        }
        return;
      }
      if (!USER.devid) USER.devid = getCookie('devid');
      USER.info = JSON.parse(this.responseText);
      if (USER.info && USER.info.devid) {
        if (USER.devid) {
          var found = false;
          for (var i = 0; i < USER.info.devid.length; i++) {
            if (USER.info.devid[i] == USER.devid) {
              found = true;
              break;
            }
          }
          //if (!found) USER.devid = USER.info.devid[0];
        } else {
          USER.devid = USER.info.devid[0];
        }
        self.setTimeout(callback, 0);
      } else {
        alert('No activated device for the specified user');
        self.setTimeout(callback, 0);
        return;
      }
    };

    this.userb64 = this.user ? btoa(this.user) : getCookie('user');
    if (!this.devid) this.devid = getCookie('devid');
    if (!this.devid) {
      var tosave = false;
      if (!this.userb64) {
        var input = window.prompt('Please enter your email or device ID');
        if (!input) return;
        if (input.indexOf('@') <= 0 && input.indexOf('#') <= 0) {
          this.devid = input;
        } else {
          this.userb64 = btoa(input);
          tosave = true;
        }
      }
      if (this.userb64 && tosave) {
        this.save();
      }
    }
    if (!this.userb64) {
      if (this.devid) {
        self.setTimeout(callback, 0);
      } else {
        alert('Please specify a device ID');
      }
    } else {
      var url = serverURL + 'query?user=' + this.userb64;
      this.xhr.open('GET', url, true);
      this.xhr.send(null);
    }
  },
  check: function (callback) {
    this.xhr.onreadystatechange = function () {
      if (this.readyState != 4) return;
      if (this.status != 200) {
        if (this.status) {
          alert('Server under maintenance (status: ' + this.status + ')');
        }
        return;
      }
      USER.info = JSON.parse(this.responseText);
      if (callback) self.setTimeout(callback, 0);
    };

    this.userb64 = this.user ? btoa(this.user) : getCookie('user');
    if (this.userb64) {
      var url = serverURL + 'query?user=' + this.userb64;
      this.xhr.open('GET', url, true);
      this.xhr.send(null);
    } else {
      if (callback) self.setTimeout(callback, 0);
    }
  },
  goDash: function (devid) {
    setCookie('devid', devid, 7);
    var s = '?';
    if (this.user) s += 'user=' + this.user + '&';
    s += 'devid=' + devid;
    location.href = s;
  },
  goNextDash: function (devid) {
    if (this.info.devid) {
      for (var i = 0; i < this.info.devid.length; i++) {
        if (this.info.devid[i] == devid) {
          this.goDash(this.info.devid[(i + 1) % this.info.devid.length]);
          return;
        }
      }
    }
  },
};
