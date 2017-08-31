package main

import (
	"fmt"
	rpi "github.com/dpawsbear/bear_rpi_go"
	"time"
)

func main(){
	fmt.Println("starting...")
	if 0 != rpi.Bcm2837_init() {
		rpi.Bcm2837_close()
		panic(55)
		return
	}



	//test for pwm (GPIO.12 and GND)
	rpi.Bcm2837_gpio_fsel(rpi.RPI_3B_GPIO_J8_12,uint8(rpi.BCM2837_GPIO_FSEL_ALT5))

	rpi.Bcm2837_pwm_set_clock(rpi.BCM2837_PWM_CLOCK_DIVIDER_16)

	rpi.Bcm2837_pwm_set_mode(0, 1, 1 )

	rpi.Bcm2837_pwm_set_range(0,1024 )

	rpi.Bcm2837_pwm_set_data(0,10 )

	var data uint32 = 1
	for{
		for {
			time.Sleep(time.Millisecond*2)
			data ++
			if data > 768{
				break
			}
			rpi.Bcm2837_pwm_set_data(0,data )
		}
		for {
			time.Sleep(time.Millisecond*2)
			data --
			if data < 2 {
				data = 1
				break
			}
			rpi.Bcm2837_pwm_set_data(0,data )
		}

	}
}