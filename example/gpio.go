package main

import (
	"fmt"
	"time"
	rpi "github.com/dpawsbear/bear_rpi_go"
)


func main(){

	fmt.Println("test for gpio 12 ")

	fmt.Println("starting...")
	if 0 != rpi.Bcm2837_init() {
		rpi.Bcm2837_close()
		panic(55)
		return
	}

	rpi.Bcm2837_gpio_fsel(rpi.RPI_3B_GPIO_J8_12,uint8(rpi.BCM2837_GPIO_FSEL_OUTP))
	rpi.Bcm2837_gpio_set_pud(rpi.RPI_3B_GPIO_J8_12, rpi.BCM2837_GPIO_PUD_UP)
	rpi.Bcm2837_gpio_set(rpi.RPI_3B_GPIO_J8_12)
	time.Sleep(time.Second * 3)
	rpi.Bcm2837_gpio_clr(rpi.RPI_3B_GPIO_J8_12)

	fmt.Println("test finish")

}