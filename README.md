<h1>Skysense - Skydiving Altimeter (in progress)</h1>


<h2>Description</h2>
Project consists of a system with peripherals connected to a microprocessor in order to calculate altitude based on air pressure and temperature. The data is then displayed on a simple screen for a skydiver to read current altitude at any given moment. The project is being done from a very low level (C language). Speed and efficiency are the highest priorities when writing the code. I will be designing and buying a PCB, 3D printing a case, and testing the prototype so stick around!
<br />

<h2>Features</h2>
- Automatic Zeroing at power on.
- STM32 ultra-low-power mode toggling (for when not in use but on)
- 60 Samples Per second (matching frame rate)
- estimated 200 jumps of use per charge.

<h2>MCU</h2>

- <b>STM32C011F6P6</b> 

<h2>Environments and Languages Used </h2>

- <b>Windows 11</b>
- <b>STM32 Cube IDE (Eclipse Based)</b>
- <b>C</b>

<h2>Peripherals:</h2>

- [BMP390](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp390.html)
- [2KBIT EEPROM](https://www.digikey.com/en/products/detail/microchip-technology/AT24CSW020-STUM-T/15219653)
- [0.91" IIC I2C Serial OLED LCD Display](https://www.amazon.com/HiLetgo-Serial-Display-SSD1306-Arduino/dp/B01N0KIVUX)
- [3.3V Power supply](https://www.digikey.com/en/products/detail/texas-instruments/TPS62203DBVT/461344)
- [Buzzer](https://www.sameskydevices.com/product/audio/buzzers/audio-transducers/cmt-322-65-smt-tr?srsltid=AfmBOooERc3Dd-YpNRm_QXyVyu42uBkd9ORBP6C3XmQU_50bZidL7A-O)

<!--
 ```diff
- text in red
+ text in green
! text in orange
# text in gray
@@ text in purple (and bold)@@
```
--!>
