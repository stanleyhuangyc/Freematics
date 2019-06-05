<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
<Document>
<name>Freematics Telemetry</name>
<Style id="brakepoint">
<BalloonStyle>
<text><strong>Brakepoint</strong><br/>
<![CDATA[
Speed: $[Speed] km/h<br/>
RPM: $[RPM]<br/>
ACC: $[ACC]
]]>
</text>
</BalloonStyle>
</Style>
<Style id="track_n">
<IconStyle>
<scale>.5</scale>
<Icon>
<href>http://earth.google.com/images/kml-icons/track-directional/track-none.png</href>
</Icon>
</IconStyle>
<LabelStyle>
<scale>0</scale>
</LabelStyle>
</Style>
<Style id="track_h">
<IconStyle>
<scale>1.2</scale>
<Icon>
<href>http://earth.google.com/images/kml-icons/track-directional/track-none.png</href>
</Icon>
</IconStyle>
</Style>
<StyleMap id="track">
<Pair>
<key>normal</key>
<styleUrl>#track_n</styleUrl>
</Pair>
<Pair>
<key>highlight</key>
<styleUrl>#track_h</styleUrl>
</Pair>
</StyleMap>
<Style id="multiTrack_n">
<IconStyle>
<Icon>
<href>http://earth.google.com/images/kml-icons/track-directional/track-0.png</href>
</Icon>
</IconStyle>
<LineStyle>
<color>eeffac7f</color>
<width>8</width>
</LineStyle>
</Style>
<Style id="multiTrack_h">
<IconStyle>
<scale>1.2</scale>
<Icon>
<href>http://earth.google.com/images/kml-icons/track-directional/track-0.png</href>
</Icon>
</IconStyle>
<LineStyle>
<color>eeffac7f</color>
<width>8</width>
</LineStyle>
</Style>
<StyleMap id="multiTrack">
<Pair>
<key>normal</key>
<styleUrl>#multiTrack_n</styleUrl>
</Pair>
<Pair>
<key>highlight</key>
<styleUrl>#multiTrack_h</styleUrl>
</Pair>
</StyleMap>
<Style id="waypoint_n">
<IconStyle>
<Icon>
<href>http://maps.google.com/mapfiles/kml/pal4/icon61.png</href>
</Icon>
</IconStyle>
</Style>
<Style id="waypoint_h">
<IconStyle>
<scale>1.2</scale>
<Icon>
<href>http://maps.google.com/mapfiles/kml/pal4/icon61.png</href>
</Icon>
</IconStyle>
</Style>
<StyleMap id="waypoint">
<Pair>
<key>normal</key>
<styleUrl>#waypoint_n</styleUrl>
</Pair>
<Pair>
<key>highlight</key>
<styleUrl>#waypoint_h</styleUrl>
</Pair>
</StyleMap>
<Style id="lineStyle">
<LineStyle>
<color>eeffac7f</color>
<width>8</width>
</LineStyle>
</Style>
<Schema id="schema">
<gx:SimpleArrayField name="10D" type="int">
<displayName>Speed (km/h)</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="10C" type="int">
<displayName>Engine RPM</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="20" type="string">
<displayName>Acceleration</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="105" type="int">
<displayName>Coolant Temperature (C)</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="104" type="int">
<displayName>Engine Load (%)</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="111" type="int">
<displayName>Throttle Position (%)</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="24" type="float">
<displayName>Battery Voltage (V)</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="C" type="float">
<displayName>Altitude (m)</displayName>
</gx:SimpleArrayField>
<gx:SimpleArrayField name="0" type="int">
<displayName>Timestamp (ms)</displayName>
</gx:SimpleArrayField>
</Schema>
<Folder>
<name>Tracks</name>
<Placemark>
<name>Trip</name>
<styleUrl>#multiTrack</styleUrl>
