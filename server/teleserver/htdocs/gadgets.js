var chartData = [
    [PID.SPEED, "Vehicle Speed", "#808080", "km/h", "Speed (km/h)", 100],
    [PID.RPM, "Engine RPM", "#404040", "rpm", "RPM", 100],
    [PID.ENGINE_LOAD, "Engine Load", "#808080", "%", "Engine (%)", 100],
    [PID.THROTTLE, "Throttle Pos.", "#808080", "%", "Throttle (%)", 100],
    [PID.COOLANT_TEMP, "Coolant Temp.", "#808080", "°„C", "Coolant (°„C)", 120],
    [PID.INTAKE_TEMP, "Intake Air Temp.", "#808080", "°„C", "Intake Air (°„C)", 100],
    [PID.ACC, "G-force", "#404040", "g", "G-force", 100],
    [PID.BATTERY_VOLTAGE, "Battery Voltage", "#808080", "V", "Voltage (V)", 100]
];

var gaugeData = [
    [PID.SPEED, "Speed (km/h)", 0, 250],
    [PID.RPM, "RPM", 0, 8000],
    [PID.THROTTLE, "Throttle (%)", 0, 100],
    [PID.BATTERY_VOLTAGE, "Battery (V)", 0, 15]
];

function CreateChart(container, title, color, val1, val2, interval, samples)
{
	var chart = new Highcharts.Chart({
            chart: {
                renderTo: container,
                type: 'area',
                animation: Highcharts.svg, // don't animate in old IE
				backgroundColor: "#111",
                marginRight: 10,
            },
            title: {
				style: { "color": "#fff" },
                text: title
            },
            xAxis: {
                type: 'datetime',
                tickPixelInterval: interval,
            },
            yAxis: {
                title: {
                    text: val1
                },
                plotLines: [{
                    value: 0,
                    width: 1,
                    color: "#808080"
                }]
            },
            tooltip: {
                formatter: function () {
                    return '<b>' + this.series.name + '</b><br/>' +
                        Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', this.x) + '<br/>' +
                        Highcharts.numberFormat(this.y, 2);
                }
            },
            legend: {
                enabled: false
            },
            exporting: {
                enabled: false
            },
            series: [{
                name: val2,
                data: samples
            }]
	});
	return chart;	
}

function CreateChartTiny(container, color, val1, val2, interval, samples)
{
	var chart = new Highcharts.Chart({
            chart: {
                renderTo: container,
                type: 'area',
                animation: Highcharts.svg, // don't animate in old IE
				backgroundColor: "#111",
                marginRight: 0,
				marginLeft: -10,
            },
            title: {
				text: ''
            },
            xAxis: {
                type: 'datetime',
                tickPixelInterval: interval,
            },
            yAxis: {
                title: {
                    text: val1
                },
                plotLines: [{
                    value: 0,
                    width: 1,
                    color: "#808080"
                }]
            },
            tooltip: {
                formatter: function () {
                    return '<b>' + this.series.name + '</b><br/>' +
                        Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', this.x) + '<br/>' +
                        Highcharts.numberFormat(this.y, 2);
                }
            },
            legend: {
                enabled: false
            },
            exporting: {
                enabled: false
            },
            series: [{
                name: val2,
                data: samples
            }]
	});
	return chart;	
}

function CreateGauge(container, title, minval, maxval)
{
	var chart = new Highcharts.Chart({
        chart: {
            renderTo: container,
			backgroundColor: "#111",
            type: 'solidgauge'
        },

        title: null,

        pane: {
            center: ['50%', '85%'],
            size: '140%',
            startAngle: -90,
            endAngle: 90,
            background: {
                backgroundColor: (Highcharts.theme && Highcharts.theme.background2) || '#333',
                innerRadius: '60%',
                outerRadius: '100%',
                shape: 'arc'
            }
        },

        tooltip: {
            enabled: false
        },

        // the value axis
        yAxis: {
            stops: [
                [0.1, '#0000FF'], // blue
                [0.5, '#00FFFF'], // yellow
                [0.9, '#FF0000'] // red
            ],
            lineWidth: 0,
            minorTickInterval: null,
			tickAmount: 2,
            min: minval,
            max: maxval,
            title: {
                text: title,
				style: { "color": "#bbb" },
                y: -55
            }
        },

        credits: {
            enabled: false
        },

        plotOptions: {
            solidgauge: {
                dataLabels: {
                    y: 10,
                    borderWidth: 0,
                    useHTML: true
                }
            }
        },
		
		exporting: {
			enabled: false
		},
		
        series: [{
            name: title,
            data: [0],
            dataLabels: {
                format: '<div style="text-align:center"><span style="font-size:xx-large;font-weight:bold;color:#fff">{y}</span>'
            },
            tooltip: {
                //valueSuffix: ' ' + unit
            }
        }]

	});
	return chart;	

}
