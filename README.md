# OAK
A fully open source Asterisk FXO/X module which based on Raspberry Pi 

# OAK Manual
### Compile the new Kernel dtbs files to support tdm bus
Follow this link to download Raspberry Pi Kernel https://www.raspberrypi.org/documentation/linux/kernel/building.md
```shell
cd ~/linux/arch/arm/boot/dts
vim bcm283x.dtsi
```
Add below tdm bus defination after the i2s section
```shell
                i2s: i2s@7e203000 {
                        compatible = "brcm,bcm2835-i2s";
                        reg = <0x7e203000 0x24>;
                        clocks = <&clocks BCM2835_CLOCK_PCM>;
                        dmas = <&dma 2>,
                               <&dma 3>;
                        dma-names = "tx", "rx";
                        status = "disabled";
                };
                //new section to add
                tdm: tdm@7e203000 {
                        compatible = "brcm,pi-tdm";
                        reg = <0x7e203000 0x20>,
                              <0x7e101098 0x02>;
                        dmas = <&dma 2>,
                               <&dma 3>;
                        dma-names = "tx", "rx";
                        interrupts = <1 25>, <1 26>;
                        interrupt-names = "dma9", "dma10";
                        status = "okay";
                };

```
```shell
cd ~/linux
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs
```
when done
```shell
mkdir pidtbs
cp arch/arm/boot/dts/*.dtb pidtbs/
```
then copy the new dtb files to the RaspberryPi /boot folder, make sure you have a backup before override them

### Disable the spi and i2s bus in /boot/config.txt file like below shows
```shell
dtparam=i2c_arm=on
#dtparam=i2s=on
#dtparam=spi=on
```

### Compile the dahdi
Get the source code from https://github.com/lixinswitchpi/oak.git
```shell
cd /usr/src/dahdi-linux
make
make install
```

### Compile the dahdi-tools

### Recompile the Asterisk to add dahdi supports
install required packages
```shell
apt install libncurses5-dev uuid-dev libjansson-dev libxml2-dev libsqlite3-dev openssl libssl-dev build-essential
```
Download the asterisk-13.20.0 and recompile it, remember to check up the chan-dahdi


