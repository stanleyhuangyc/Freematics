var mychart;

var opts = {
  chart: {
    renderTo: 'chart1',
    zoomType: 'x',
    backgroundColor: '#111',
    events: {
      click: function (e) {
        console.log(
          Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', e.xAxis[0].value),
          e.yAxis[0].value,
          e.yAxis[1].value
        );
        showMarkerFromTime(e.xAxis[0].value);
      },
      selection: function (event) {
        // log the min and max of the primary, datetime x-axis
        if (event.xAxis) {
          console.log(
            Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', event.xAxis[0].min),
            Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', event.xAxis[0].max)
          );
        }
        // log the min and max of the y axis
        //console.log(event.xAxis[0].min, event.xAxis[0].max);
      },
    },
  },
  title: {
    text: '',
  },
  legend: {
    enabled: false,
  },
  xAxis: [
    {
      type: 'datetime',
      minRange: 100,
      title: { text: 'Time' },
    },
  ],
  yAxis: [
    {
      // Primary yAxis
      labels: {
        formatter: function () {
          return this.value;
        },
        style: {
          color: '#BB0000',
        },
        align: 'left',
        x: 0,
        y: -2,
      },
      showFirstLabel: false,
      title: {
        text: '',
        style: {
          color: '#BB0000',
        },
      },
    },
    {
      // Secondary yAxis
      title: {
        text: '',
        style: {
          color: '#4572A7',
        },
      },
      labels: {
        align: 'right',
        x: 0,
        y: -2,
        formatter: function () {
          return this.value;
        },
        style: {
          color: '#4572A7',
        },
      },
      showFirstLabel: false,
      opposite: true,
    },
  ],
  tooltip: {
    formatter: function () {
      return this.series.name + ': <strong>' + this.y + '</strong>';
    },
  },

  plotOptions: {
    series: {
      marker: {
        enabled: false,
      },
    },
  },
  time: {
    useUTC: true,
  },
  series: [],
};

function ShowChart(divName, names, dataset) {
  if (mychart) mychart.destroy();
  var linecolor = ['#BB0000', '#4572A7'];
  opts.series = [];
  for (i = 0; i < dataset.length; i++) {
    var series = new Object();
    series.type = 'spline';
    series.name = names[i];
    series.color = linecolor[i];
    series.yAxis = i;
    series.data = dataset[i];
    //if (info[i].unit != '°C') opts.yAxis[i].min = 0;
    //if (opts.title.text != "") opts.title.text += " vs ";
    //opts.title.text += info[i].name;
    opts.yAxis[i].title.text = names[i];
    opts.yAxis[i].title.style.color = linecolor[i];
    opts.series.push(series);
  }
  opts.chart.renderTo = divName;
  mychart = new Highcharts.Chart(opts);
  return mychart;
}

function SetChartRange(startpos, endpos) {
  mychart.xAxis[0].setExtremes(startpos, endpos);
}

function GetTimeByLocation(lat, lon) {
  xmlhttp.open(
    'GET',
    '/obdchart/query?lat=' + lat + '&lon=' + lon + '&id=' + GetUrlArg('id'),
    false
  );
  xmlhttp.send(null);
  if (xmlhttp.status == 200) {
    return parseInt(xmlhttp.responseText);
  }
  return -1;
}
