<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.2.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="NodeMCU-Esp32-s">
<packages>
<package name="ESP32_DEVKIT">
<description>ESP32_DEVKIT</description>
<wire x1="-15.24" y1="-26.67" x2="-15.24" y2="24.13" width="0.2032" layer="51"/>
<wire x1="-15.24" y1="-26.67" x2="-5.08" y2="-26.67" width="0.2032" layer="51"/>
<wire x1="-5.08" y1="-26.67" x2="2.54" y2="-26.67" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-26.67" x2="12.7" y2="-26.67" width="0.2032" layer="51"/>
<wire x1="-15.24" y1="24.13" x2="-10.16" y2="24.13" width="0.2032" layer="51"/>
<wire x1="-10.16" y1="24.13" x2="7.62" y2="24.13" width="0.2032" layer="51"/>
<wire x1="7.62" y1="24.13" x2="12.7" y2="24.13" width="0.2032" layer="51"/>
<wire x1="12.7" y1="24.13" x2="12.7" y2="-26.67" width="0.2032" layer="51"/>
<pad name="EN" x="-13.97" y="17.78" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="VP" x="-13.97" y="15.24" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="VN" x="-13.97" y="12.7" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D34" x="-13.97" y="10.16" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D35" x="-13.97" y="7.62" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D32" x="-13.97" y="5.08" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D33" x="-13.97" y="2.54" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D25" x="-13.97" y="0" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D26" x="-13.97" y="-2.54" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D27" x="-13.97" y="-5.08" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D14" x="-13.97" y="-7.62" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D12" x="-13.97" y="-10.16" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D13" x="-13.97" y="-12.7" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="GND@1" x="-13.97" y="-15.24" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="VIN" x="-13.97" y="-17.78" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D23" x="11.43" y="17.78" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D22" x="11.43" y="15.24" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="TX0" x="11.43" y="12.7" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="RX0" x="11.43" y="10.16" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D21" x="11.43" y="7.62" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D19" x="11.43" y="5.08" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D18" x="11.43" y="2.54" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D5" x="11.43" y="0" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="TX2" x="11.43" y="-2.54" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="RX2" x="11.43" y="-5.08" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D4" x="11.43" y="-7.62" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D2" x="11.43" y="-10.16" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="D15" x="11.43" y="-12.7" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="GND@2" x="11.43" y="-15.24" drill="0.9" diameter="1.4224" shape="octagon"/>
<pad name="3V3" x="11.43" y="-17.78" drill="0.9" diameter="1.4224" shape="octagon"/>
<wire x1="-10.16" y1="24.13" x2="-10.16" y2="1.27" width="0.2032" layer="51"/>
<wire x1="-10.16" y1="1.27" x2="7.62" y2="1.27" width="0.2032" layer="51"/>
<wire x1="7.62" y1="1.27" x2="7.62" y2="24.13" width="0.2032" layer="51"/>
<wire x1="-5.08" y1="-26.67" x2="-5.08" y2="-22.86" width="0.2032" layer="51"/>
<wire x1="-5.08" y1="-22.86" x2="2.54" y2="-22.86" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-22.86" x2="2.54" y2="-26.67" width="0.2032" layer="51"/>
<wire x1="-6.35" y1="19.05" x2="-6.35" y2="22.86" width="0.2032" layer="51"/>
<wire x1="-6.35" y1="22.86" x2="-5.08" y2="22.86" width="0.2032" layer="51"/>
<wire x1="-5.08" y1="22.86" x2="-5.08" y2="20.32" width="0.2032" layer="51"/>
<wire x1="-5.08" y1="20.32" x2="-3.81" y2="20.32" width="0.2032" layer="51"/>
<wire x1="-3.81" y1="20.32" x2="-3.81" y2="22.86" width="0.2032" layer="51"/>
<wire x1="-3.81" y1="22.86" x2="-2.54" y2="22.86" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="22.86" x2="-2.54" y2="20.32" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="20.32" x2="-1.27" y2="20.32" width="0.2032" layer="51"/>
<wire x1="-1.27" y1="20.32" x2="-1.27" y2="22.86" width="0.2032" layer="51"/>
<wire x1="-1.27" y1="22.86" x2="0" y2="22.86" width="0.2032" layer="51"/>
<wire x1="0" y1="22.86" x2="0" y2="20.32" width="0.2032" layer="51"/>
<wire x1="0" y1="20.32" x2="1.27" y2="20.32" width="0.2032" layer="51"/>
<wire x1="1.27" y1="20.32" x2="1.27" y2="22.86" width="0.2032" layer="51"/>
<wire x1="1.27" y1="22.86" x2="3.81" y2="22.86" width="0.2032" layer="51"/>
<wire x1="-8.89" y1="-22.86" x2="-6.35" y2="-22.86" width="0.2032" layer="51"/>
<wire x1="-6.35" y1="-22.86" x2="-6.35" y2="-25.4" width="0.2032" layer="51"/>
<wire x1="-6.35" y1="-25.4" x2="-8.89" y2="-25.4" width="0.2032" layer="51"/>
<wire x1="-8.89" y1="-25.4" x2="-8.89" y2="-22.86" width="0.2032" layer="51"/>
<wire x1="3.81" y1="-22.86" x2="3.81" y2="-25.4" width="0.2032" layer="51"/>
<wire x1="3.81" y1="-25.4" x2="6.35" y2="-25.4" width="0.2032" layer="51"/>
<wire x1="6.35" y1="-25.4" x2="6.35" y2="-22.86" width="0.2032" layer="51"/>
<wire x1="6.35" y1="-22.86" x2="3.81" y2="-22.86" width="0.2032" layer="51"/>
<text x="-12.7" y="17.78" size="0.8128" layer="25">EN</text>
<text x="-12.7" y="15.24" size="0.8128" layer="25">VP</text>
<text x="-12.7" y="12.7" size="0.8128" layer="25">VN</text>
<text x="-12.7" y="10.16" size="0.8128" layer="25">D34</text>
<text x="-12.7" y="7.62" size="0.8128" layer="25">D35</text>
<text x="-12.7" y="5.08" size="0.8128" layer="25">D32</text>
<text x="-12.7" y="2.54" size="0.8128" layer="25">D33</text>
<text x="-12.7" y="0" size="0.8128" layer="25">D25</text>
<text x="-12.7" y="-2.54" size="0.8128" layer="25">D26</text>
<text x="-12.7" y="-5.08" size="0.8128" layer="25">D27</text>
<text x="-12.7" y="-7.62" size="0.8128" layer="25">D14</text>
<text x="-12.7" y="-10.16" size="0.8128" layer="25">D12</text>
<text x="-12.7" y="-12.7" size="0.8128" layer="25">D13</text>
<text x="-12.7" y="-15.24" size="0.8128" layer="25">GND</text>
<text x="-12.7" y="-17.78" size="0.8128" layer="25">VIN</text>
<text x="10.16" y="-17.78" size="0.8128" layer="25" align="bottom-right">3V3</text>
<text x="10.16" y="-15.24" size="0.8128" layer="25" align="bottom-right">GND</text>
<text x="10.16" y="-12.7" size="0.8128" layer="25" align="bottom-right">D15</text>
<text x="10.16" y="-10.16" size="0.8128" layer="25" align="bottom-right">D2</text>
<text x="10.16" y="-7.62" size="0.8128" layer="25" align="bottom-right">D4</text>
<text x="10.16" y="-5.08" size="0.8128" layer="25" align="bottom-right">RX2</text>
<text x="10.16" y="-2.54" size="0.8128" layer="25" align="bottom-right">TX2</text>
<text x="10.16" y="0" size="0.8128" layer="25" align="bottom-right">D5</text>
<text x="10.16" y="2.54" size="0.8128" layer="25" align="bottom-right">D18</text>
<text x="10.16" y="5.08" size="0.8128" layer="25" align="bottom-right">D19</text>
<text x="10.16" y="7.62" size="0.8128" layer="25" align="bottom-right">D21</text>
<text x="10.16" y="10.16" size="0.8128" layer="25" align="bottom-right">RX0</text>
<text x="10.16" y="12.7" size="0.8128" layer="25" align="bottom-right">TX0</text>
<text x="10.16" y="15.24" size="0.8128" layer="25" align="bottom-right">D22</text>
<text x="10.16" y="17.78" size="0.8128" layer="25" align="bottom-right">D23</text>
<text x="-5.08" y="-21.59" size="1.778" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-21.59" size="1.778" layer="27" rot="R90">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="ESP32_DEVKIT">
<description>ESP32_DEVKIT</description>
<pin name="EN" x="-17.78" y="17.78" visible="pin" length="middle"/>
<pin name="VP" x="-17.78" y="15.24" visible="pin" length="middle"/>
<pin name="VN" x="-17.78" y="12.7" visible="pin" length="middle"/>
<pin name="D34" x="-17.78" y="10.16" visible="pin" length="middle"/>
<pin name="D35" x="-17.78" y="7.62" visible="pin" length="middle"/>
<pin name="D32" x="-17.78" y="5.08" visible="pin" length="middle"/>
<pin name="D33" x="-17.78" y="2.54" visible="pin" length="middle"/>
<pin name="D25" x="-17.78" y="0" visible="pin" length="middle"/>
<pin name="D26" x="-17.78" y="-2.54" visible="pin" length="middle"/>
<pin name="D27" x="-17.78" y="-5.08" visible="pin" length="middle"/>
<pin name="D14" x="-17.78" y="-7.62" visible="pin" length="middle"/>
<pin name="D12" x="-17.78" y="-10.16" visible="pin" length="middle"/>
<pin name="D13" x="-17.78" y="-12.7" visible="pin" length="middle"/>
<pin name="GND@1" x="-17.78" y="-15.24" visible="pin" length="middle"/>
<pin name="VIN" x="-17.78" y="-17.78" visible="pin" length="middle"/>
<pin name="D23" x="17.78" y="17.78" visible="pin" length="middle" rot="R180"/>
<pin name="D22" x="17.78" y="15.24" visible="pin" length="middle" rot="R180"/>
<pin name="TX0" x="17.78" y="12.7" visible="pin" length="middle" rot="R180"/>
<pin name="RX0" x="17.78" y="10.16" visible="pin" length="middle" rot="R180"/>
<pin name="D21" x="17.78" y="7.62" visible="pin" length="middle" rot="R180"/>
<pin name="D19" x="17.78" y="5.08" visible="pin" length="middle" rot="R180"/>
<pin name="D18" x="17.78" y="2.54" visible="pin" length="middle" rot="R180"/>
<pin name="D5" x="17.78" y="0" visible="pin" length="middle" rot="R180"/>
<pin name="TX2" x="17.78" y="-2.54" visible="pin" length="middle" rot="R180"/>
<pin name="RX2" x="17.78" y="-5.08" visible="pin" length="middle" rot="R180"/>
<pin name="D4" x="17.78" y="-7.62" visible="pin" length="middle" rot="R180"/>
<pin name="D2" x="17.78" y="-10.16" visible="pin" length="middle" rot="R180"/>
<pin name="D15" x="17.78" y="-12.7" visible="pin" length="middle" rot="R180"/>
<pin name="GND@2" x="17.78" y="-15.24" visible="pin" length="middle" rot="R180"/>
<pin name="3V3" x="17.78" y="-17.78" visible="pin" length="middle" rot="R180"/>
<wire x1="-12.7" y1="20.32" x2="-12.7" y2="-20.32" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-20.32" x2="12.7" y2="-20.32" width="0.254" layer="94"/>
<wire x1="12.7" y1="-20.32" x2="12.7" y2="20.32" width="0.254" layer="94"/>
<wire x1="12.7" y1="20.32" x2="-12.7" y2="20.32" width="0.254" layer="94"/>
<text x="-12.7" y="20.32" size="1.778" layer="95">&gt;NAME</text>
<text x="-12.7" y="-22.86" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ESP32_DEVKIT">
<description>ESP32 DEVKIT BOARD</description>
<gates>
<gate name="G$1" symbol="ESP32_DEVKIT" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ESP32_DEVKIT">
<connects>
<connect gate="G$1" pin="3V3" pad="3V3"/>
<connect gate="G$1" pin="D12" pad="D12"/>
<connect gate="G$1" pin="D13" pad="D13"/>
<connect gate="G$1" pin="D14" pad="D14"/>
<connect gate="G$1" pin="D15" pad="D15"/>
<connect gate="G$1" pin="D18" pad="D18"/>
<connect gate="G$1" pin="D19" pad="D19"/>
<connect gate="G$1" pin="D2" pad="D2"/>
<connect gate="G$1" pin="D21" pad="D21"/>
<connect gate="G$1" pin="D22" pad="D22"/>
<connect gate="G$1" pin="D23" pad="D23"/>
<connect gate="G$1" pin="D25" pad="D25"/>
<connect gate="G$1" pin="D26" pad="D26"/>
<connect gate="G$1" pin="D27" pad="D27"/>
<connect gate="G$1" pin="D32" pad="D32"/>
<connect gate="G$1" pin="D33" pad="D33"/>
<connect gate="G$1" pin="D34" pad="D34"/>
<connect gate="G$1" pin="D35" pad="D35"/>
<connect gate="G$1" pin="D4" pad="D4"/>
<connect gate="G$1" pin="D5" pad="D5"/>
<connect gate="G$1" pin="EN" pad="EN"/>
<connect gate="G$1" pin="GND@1" pad="GND@1"/>
<connect gate="G$1" pin="GND@2" pad="GND@2"/>
<connect gate="G$1" pin="RX0" pad="RX0"/>
<connect gate="G$1" pin="RX2" pad="RX2"/>
<connect gate="G$1" pin="TX0" pad="TX0"/>
<connect gate="G$1" pin="TX2" pad="TX2"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
<connect gate="G$1" pin="VN" pad="VN"/>
<connect gate="G$1" pin="VP" pad="VP"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="hcsr04">
<description>Ultrasonic Sensor HC-SR04</description>
<packages>
<package name="HC-SR04_FOOTPRINT">
<description>Ultrasonic Sensor HC-SR04 Footprint</description>
<pad name="ECHO" x="0" y="1.27" drill="0.9" diameter="1.778"/>
<pad name="TRIG" x="0" y="-1.27" drill="0.9" diameter="1.778"/>
<pad name="VCC" x="0" y="-3.81" drill="0.9" diameter="1.778"/>
<pad name="GND" x="0" y="3.81" drill="0.9" diameter="1.778" shape="square"/>
<wire x1="-1.27" y1="5.08" x2="-1.27" y2="-5.08" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-5.08" x2="1.27" y2="-5.08" width="0.127" layer="21"/>
<wire x1="1.27" y1="-5.08" x2="1.27" y2="5.08" width="0.127" layer="21"/>
<wire x1="1.27" y1="5.08" x2="-1.27" y2="5.08" width="0.127" layer="21"/>
<text x="1.397" y="3.556" size="0.6096" layer="21">GND</text>
<text x="1.397" y="1.016" size="0.6096" layer="21">Echo</text>
<text x="1.397" y="-1.524" size="0.6096" layer="21">Trig</text>
<text x="1.397" y="-4.064" size="0.6096" layer="21">VCC</text>
<text x="-2.032" y="-5.08" size="1.016" layer="25" ratio="12" rot="R90">&gt;NAME</text>
</package>
</packages>
<symbols>
<symbol name="HC-SR04_SYMBOL">
<description>Ultrasonic Sensor HC-SR04 Schematic</description>
<pin name="GND" x="-12.7" y="7.62" length="middle"/>
<pin name="ECHO" x="-12.7" y="2.54" length="middle"/>
<pin name="TRIG" x="-12.7" y="-2.54" length="middle"/>
<pin name="5V" x="-12.7" y="-7.62" length="middle"/>
<wire x1="-7.62" y1="10.16" x2="-7.62" y2="-10.16" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-10.16" x2="2.54" y2="-10.16" width="0.254" layer="94"/>
<wire x1="2.54" y1="-10.16" x2="2.54" y2="10.16" width="0.254" layer="94"/>
<wire x1="2.54" y1="10.16" x2="-7.62" y2="10.16" width="0.254" layer="94"/>
<text x="-7.62" y="10.922" size="1.778" layer="95" ratio="12">&gt;NAME</text>
<text x="-7.62" y="-12.954" size="1.778" layer="96" ratio="12">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="HC-SR04" prefix="J_ULTRASONIC_">
<description>Ultrasonic Sensor HC-SR04</description>
<gates>
<gate name="J_ULTRASONIC" symbol="HC-SR04_SYMBOL" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HC-SR04_FOOTPRINT">
<connects>
<connect gate="J_ULTRASONIC" pin="5V" pad="VCC"/>
<connect gate="J_ULTRASONIC" pin="ECHO" pad="ECHO"/>
<connect gate="J_ULTRASONIC" pin="GND" pad="GND"/>
<connect gate="J_ULTRASONIC" pin="TRIG" pad="TRIG"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="NodeMCU-Esp32-s" deviceset="ESP32_DEVKIT" device=""/>
<part name="J_ULTRASONIC_1" library="hcsr04" deviceset="HC-SR04" device=""/>
<part name="J_ULTRASONIC_2" library="hcsr04" deviceset="HC-SR04" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="71.12" y="48.26" smashed="yes">
<attribute name="NAME" x="58.42" y="68.58" size="1.778" layer="95"/>
<attribute name="VALUE" x="58.42" y="25.4" size="1.778" layer="96"/>
</instance>
<instance part="J_ULTRASONIC_1" gate="J_ULTRASONIC" x="147.32" y="83.82" smashed="yes">
<attribute name="NAME" x="139.7" y="94.742" size="1.778" layer="95" ratio="12"/>
<attribute name="VALUE" x="139.7" y="70.866" size="1.778" layer="96" ratio="12"/>
</instance>
<instance part="J_ULTRASONIC_2" gate="J_ULTRASONIC" x="147.32" y="53.34" smashed="yes">
<attribute name="NAME" x="139.7" y="64.262" size="1.778" layer="95" ratio="12"/>
<attribute name="VALUE" x="139.7" y="40.386" size="1.778" layer="96" ratio="12"/>
</instance>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
