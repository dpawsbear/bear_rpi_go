# bear_rpi_go(raspberry 3B library)

　　this is the raspberry 3b  base library built by golang

on the go!


## How to use?

run first demo as gpio ( GPIO.12(vcc) and GND )

    go get github.com/dpawsbear/bear_rpi_go

    cd $gopath/src/github.com/dpawsbear/bear_rpi_go/example

    go build -ldflags "-w -s" gpio.go

    sudo ./gpio

then you can see led connect GPIO.12 and GND light up 2 seconds and close .




## changelog:

1. 2017.08.30 dpawsbear : first add and test pwm and gpio
2. 2017.08.31 dpawsbear : try to change this as library
3. 2017.08.31 dpawsbear : add the first demo(gpio)
3. 2017.08.31 dpawsbear : add new branch
