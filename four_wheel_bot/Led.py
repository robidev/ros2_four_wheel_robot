# -*-coding: utf-8 -*-
import spidev
import threading
import numpy
import time
import os
from numpy import sin, cos, pi

class SPI_LedPixel(object):
    def __init__(self, count = 8, bright = 255, sequence='GRB', bus = 0, device = 0):
        self.set_led_type(sequence)
        self.set_led_count(count)
        self.set_led_brightness(bright)
        self.led_begin(bus, device)
        self.set_all_led_color(0,0,0)
       
    def led_begin(self, bus = 0, device = 0):
        self.bus = bus
        self.device = device
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)
            self.spi.mode = 0
            self.led_init_state = 1
        except OSError:
            print("Please check the configuration in /boot/firmware/config.txt.")
            if self.bus == 0:
                print("You can turn on the 'SPI' in 'Interface Options' by using 'sudo raspi-config'.")
                print("Or make sure that 'dtparam=spi=on' is not commented, then reboot the Raspberry Pi. Otherwise spi0 will not be available.")
            else:
                print("Please add 'dtoverlay=spi{}-2cs' at the bottom of the /boot/firmware/config.txt, then reboot the Raspberry Pi. otherwise spi{} will not be available.".format(self.bus, self.bus))
            self.led_init_state = 0
            
    def check_spi_state(self):
        return self.led_init_state
        
    def spi_gpio_info(self):
        if self.bus == 0:
            print("SPI0-MOSI: GPIO10(WS2812-PIN)  SPI0-MISO: GPIO9  SPI0-SCLK: GPIO11  SPI0-CE0: GPIO8  SPI0-CE1: GPIO7")
        elif self.bus == 1:
            print("SPI1-MOSI: GPIO20(WS2812-PIN)   SPI1-MISO: GPIO19  SPI1-SCLK: GPIO21  SPI1-CE0: GPIO18  SPI1-CE1: GPIO17  SPI0-CE1: GPIO16")
        elif self.bus == 2:
            print("SPI2-MOSI: GPIO41(WS2812-PIN)   SPI2-MISO: GPIO40  SPI2-SCLK: GPIO42  SPI2-CE0: GPIO43  SPI2-CE1: GPIO44  SPI2-CE1: GPIO45")
        elif self.bus == 3:
            print("SPI3-MOSI: GPIO2(WS2812-PIN)  SPI3-MISO: GPIO1  SPI3-SCLK: GPIO3  SPI3-CE0: GPIO0  SPI3-CE1: GPIO24")
        elif self.bus == 4:
            print("SPI4-MOSI: GPIO6(WS2812-PIN)  SPI4-MISO: GPIO5  SPI4-SCLK: GPIO7  SPI4-CE0: GPIO4  SPI4-CE1: GPIO25")
        elif self.bus == 5:
            print("SPI5-MOSI: GPIO14(WS2812-PIN)  SPI5-MISO: GPIO13  SPI5-SCLK: GPIO15  SPI5-CE0: GPIO12  SPI5-CE1: GPIO26")
        elif self.bus == 6:
            print("SPI6-MOSI: GPIO20(WS2812-PIN)  SPI6-MISO: GPIO19  SPI6-SCLK: GPIO21  SPI6-CE0: GPIO18  SPI6-CE1: GPIO27")
    
    def led_close(self):
        self.set_all_led_rgb([0,0,0])
        self.spi.close()
    
    def set_led_count(self, count):
        self.led_count = count
        self.led_color = [0,0,0] * self.led_count
        self.led_original_color = [0,0,0] * self.led_count
    
    def set_led_type(self, rgb_type):
        try:
            led_type = ['RGB','RBG','GRB','GBR','BRG','BGR']
            led_type_offset = [0x06,0x09,0x12,0x21,0x18,0x24]
            index = led_type.index(rgb_type)
            self.led_red_offset = (led_type_offset[index]>>4) & 0x03
            self.led_green_offset = (led_type_offset[index]>>2) & 0x03
            self.led_blue_offset = (led_type_offset[index]>>0) & 0x03
            return index
        except ValueError:
            self.led_red_offset = 1
            self.led_green_offset = 0
            self.led_blue_offset = 2
            return -1
    
    def set_led_brightness(self, brightness):
        self.led_brightness = brightness
        for i in range(self.led_count):
            self.set_led_rgb_data(i, self.led_original_color)
            
    def set_ledpixel(self, index, r, g, b):
        p = [0,0,0]
        p[self.led_red_offset] = round(r * self.led_brightness / 255)
        p[self.led_green_offset] = round(g * self.led_brightness / 255)
        p[self.led_blue_offset] = round(b * self.led_brightness / 255)
        self.led_original_color[index*3+self.led_red_offset] = r
        self.led_original_color[index*3+self.led_green_offset] = g
        self.led_original_color[index*3+self.led_blue_offset] = b
        for i in range(3):
            self.led_color[index*3+i] = p[i]

    def set_led_color_data(self, index, r, g, b):
        self.set_ledpixel(index, r, g, b)  
        
    def set_led_rgb_data(self, index, color):
        self.set_ledpixel(index, color[0], color[1], color[2])   
        
    def set_led_color(self, index, r, g, b):
        self.set_ledpixel(index, r, g, b)
        self.show() 
        
    def set_led_rgb(self, index, color):
        self.set_led_rgb_data(index, color)   
        self.show() 
    
    def set_all_led_color_data(self, r, g, b):
        for i in range(self.led_count):
            self.set_led_color_data(i, r, g, b)
            
    def set_all_led_rgb_data(self, color):
        for i in range(self.led_count):
            self.set_led_rgb_data(i, color)   
        
    def set_all_led_color(self, r, g, b):
        for i in range(self.led_count):
            self.set_led_color_data(i, r, g, b)
        self.show()
        
    def set_all_led_rgb(self, color):
        for i in range(self.led_count):
            self.set_led_rgb_data(i, color) 
        self.show()
    
    def write_ws2812_numpy8(self):
        d = numpy.array(self.led_color).ravel()        
        tx = numpy.zeros(len(d)*8, dtype=numpy.uint8)  
        for ibit in range(8):                          
            tx[7-ibit::8]=((d>>ibit)&1)*0x78 + 0x80    #T0H=1,T0L=7, T1H=5,T1L=3
        if self.led_init_state != 0:
            if self.bus == 0:
                self.spi.xfer(tx.tolist(), int(8/1.25e-6))         #6.4MHz
            else:
                self.spi.xfer(tx.tolist(), int(8/1.0e-6))          #8MHz
        
    def write_ws2812_numpy4(self):
        d=numpy.array(self.led_color).ravel()
        tx=numpy.zeros(len(d)*4, dtype=numpy.uint8)
        for ibit in range(4):
            tx[3-ibit::4]=((d>>(2*ibit+1))&1)*0x60 + ((d>>(2*ibit+0))&1)*0x06 + 0x88  
        if self.led_init_state != 0:
            if self.bus == 0:
                self.spi.xfer(tx.tolist(), int(4/1.25e-6))         
            else:
                self.spi.xfer(tx.tolist(), int(4/1.0e-6))       
        
    def show(self, mode = 1):
        if mode == 1:
            write_ws2812 = self.write_ws2812_numpy8
        else:
            write_ws2812 = self.write_ws2812_numpy4
        write_ws2812()
        
    def wheel(self, pos):
        if pos < 85:
            return [(255 - pos * 3), (pos * 3), 0]
        elif pos < 170:
            pos = pos - 85
            return [0, (255 - pos * 3), (pos * 3)]
        else:
            pos = pos - 170
            return [(pos * 3), 0, (255 - pos * 3)]
    
    def hsv2rgb(self, h, s, v):
        h = h % 360
        rgb_max = round(v * 2.55)
        rgb_min = round(rgb_max * (100 - s) / 100)
        i = round(h / 60)
        diff = round(h % 60)
        rgb_adj = round((rgb_max - rgb_min) * diff / 60)
        if i == 0:
            r = rgb_max
            g = rgb_min + rgb_adj
            b = rgb_min
        elif i == 1:
            r = rgb_max - rgb_adj
            g = rgb_max
            b = rgb_min
        elif i == 2:
            r = rgb_min
            g = rgb_max
            b = rgb_min + rgb_adj
        elif i == 3:
            r = rgb_min
            g = rgb_max - rgb_adj
            b = rgb_max
        elif i == 4:
            r = rgb_min + rgb_adj
            g = rgb_min
            b = rgb_max
        else:
            r = rgb_max
            g = rgb_min
            b = rgb_max - rgb_adj
        return [r, g, b]

class Led:
    def __init__(self):
        self.result = os.popen('cat /proc/device-tree/model')
        self.s = self.result.read()
        if "Raspberry Pi" in self.s:  # 支持所有树莓派型号
            print("Hardware supported")
            self.Ledsupported = 1
            # 使用SPI_LedPixel替代Pi5Neo
            # count=8: LED数量
            # bright=255: 最大亮度
            # sequence='GRB': WS2812的颜色序列
            # bus=0: 使用SPI0
            self.strip = SPI_LedPixel(count=8, bright=255, sequence='GRB', bus=0)
            if self.strip.check_spi_state() == 0:
                print("SPI initialization failed")
                self.Ledsupported = 0
        else:
            print("Hardware not supported")
            self.Ledsupported = 0

    def colorWipe(self, color, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.led_count):
            # 适配SPI_LedPixel的颜色设置方式
            self.strip.set_led_color(i, color[0], color[1], color[2])
            time.sleep(wait_ms / 1000.0)

    def theaterChase(self, color, wait_ms=50, iterations=10):
        """Movie theater light style chaser animation."""
        for j in range(iterations):
            for q in range(3):
                for i in range(0, self.strip.led_count, 3):
                    if i + q < self.strip.led_count:
                        self.strip.set_led_color_data(i + q, color[0], color[1], color[2])
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.led_count, 3):
                    if i + q < self.strip.led_count:
                        self.strip.set_led_color_data(i + q, 0, 0, 0)

    def wheel(self, pos):
        """Generate rainbow colors across 0-255 positions."""
        # 直接使用SPI_LedPixel的wheel函数
        return self.strip.wheel(pos)

    def rainbow(self, wait_ms=20, iterations=1):
        """Draw rainbow that fades across all pixels at once."""
        for j in range(256 * iterations):
            for i in range(self.strip.led_count):
                color = self.wheel((i + j) & 255)
                self.strip.set_led_rgb_data(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def rainbowCycle(self, wait_ms=20, iterations=5):
        """Draw rainbow that uniformly distributes itself across all pixels."""
        for j in range(256 * iterations):
            for i in range(self.strip.led_count):
                color = self.wheel(
                    (int(i * 256 / self.strip.led_count) + j) & 255)
                self.strip.set_led_rgb_data(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def theaterChaseRainbow(self, wait_ms=50):
        """Rainbow movie theater light style chaser animation."""
        for j in range(256):
            for q in range(3):
                for i in range(0, self.strip.led_count, 3):
                    if i + q < self.strip.led_count:
                        color = self.wheel((i + j) % 255)
                        self.strip.set_led_rgb_data(i + q, color)
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.led_count, 3):
                    if i + q < self.strip.led_count:
                        self.strip.set_led_color_data(i + q, 0, 0, 0)

    def ledIndex(self, index, R, G, B):
        """Set a specific LED or all LEDs based on the index."""
        print(f"Setting LED: index={index}, R={R}, G={G}, B={B}")
        if index == 255:
            self.strip.set_all_led_color(R, G, B)
        else:
            self.strip.set_led_color(index, R, G, B)

    def ledMode(self, n):
        """Run pre-defined LED modes."""
        self.mode = n
        try:
            while True:
                if self.mode == '2':
                    self.colorWipe((255, 0, 0))  # Red wipe
                    self.colorWipe((0, 255, 0))  # Green wipe
                    self.colorWipe((0, 0, 255))  # Blue wipe
                    self.colorWipe((0, 0, 0), 10)
                elif self.mode == '3':
                    self.theaterChaseRainbow()
                    self.colorWipe((0, 0, 0), 10)
                elif self.mode == '4':
                    self.rainbow()
                    self.colorWipe((0, 0, 0), 10)
                elif self.mode == '5':
                    self.rainbowCycle()
                    self.colorWipe((0, 0, 0), 10)
                else:
                    self.colorWipe((0, 0, 0), 10)
                    break
        except KeyboardInterrupt:
            self.strip.set_all_led_color(0, 0, 0)

    def __del__(self):
        """清理函数"""
        if hasattr(self, 'strip') and self.Ledsupported:
            self.strip.set_all_led_color(0, 0, 0)

led = Led()

if __name__ == '__main__':
    print('Program is starting ... ')
    if led.Ledsupported == 1:
        try:
            while True:
                print("Chaser animation")
                led.colorWipe((255, 0, 0))  # Red wipe
                led.colorWipe((0, 255, 0))  # Green wipe
                led.colorWipe((0, 0, 255))  # Blue wipe
                led.theaterChaseRainbow()
                print("Rainbow animation")
                led.rainbow()
                led.rainbowCycle()
                led.colorWipe((0, 0, 0), 10)
        except KeyboardInterrupt:
            led.strip.set_all_led_color(0, 0, 0)
    else:
        print("\nThis hardware is not supported for LED control.")