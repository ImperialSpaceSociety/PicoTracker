# uBlox M8 GPS

We decided to consider GPS modules based on the uBlox M8 because of their functionality, low cost, and support for high-altitude operation.

## M8N Capabilities

These GPS units, based on the UBX-G8030 IC, support multiple GNSS constellations (GPS, GLONASS, QZSS, and SBAS). With external flash, they can also support Galileo and BeiDou.

In airborne mode, the M8 modules are capable of operating up to a maximum altitude of 50 km.

## GPS Modules based on M8

Our early development was based on uBlox MAX-M8 modules, but these required PCB mounting and were expensive in small quantities. The uBlox NEO-M8 adds a low-noise amplifier (LNA) and SAW filter, and clones are available at low cost, but again they require PCB mounting.

We found the following UBX-G8030-based boards with LNA and SAW filters to be low cost and easy to connect without an additional PCB. They all have flash memory and battery backup.

* Beitian BN-280: The module is available with and without a compass. We do not use the compass and do not have an I2C interface available on the HC12, so either is suitable. The interface has six pins, including two unused I2C pins for the compass. The unmodified module weighs 13.5 g; removing the ceramic antenna and screening can gives a weight of 2.5 g.

* Beitian [BN-220](https://surehobby.com/desc/HR/HR5512/BN_220_GPS_Antenna_datasheet.pdf): This is the smaller sibling of the BN-280 at 22 mm × 20 mm × 6.5 mm. It has an integrated antenna and a four-pin interface. The unmodified module weighs 5.6 g.

* TOPGNSS [GG-1802](http://www.stotoncn.com/gnssmodule/showproduct.php?lang=en&id=63)/[GB-1803](http://www.stotoncn.com/gnssmodule/product.php?lang=en&class3=110): These are the smallest GPS modules we used, at 18 mm × 18 mm × 6.2 mm. They have an integrated antenna and a five-pin interface including a 1PPS output. The GG-1802 uses GPS and GLONASS by default, while the GB-1803 uses GPS and BeiDou by default. The unmodified module weighs 7.25 g; removing the ceramic antenna gives a weight of 1.6 g. We estimated that removing the screening can and connector could reduce this to around 1 g.

At the time of the original project, the modules above were available for around £6 each. They worked for our prototypes and appeared to use genuine uBlox chips.

## GPS Power Consumption
