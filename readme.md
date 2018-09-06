# ADS1115
C++ I2C driver for the Texas Instruments ADS1115. Currently only supports Linux in single shot read.

## Prerequisites
* [cmake](https://cmake.org/)

## Build
```
mkdir build
cd build
cmake ..
make
```
## Example
The repository comes with an example application *bin/ADS1115_example* to run on a Raspberry Pi with the address configured to 0x48 simply do:
```
./build/bin/ADS1115_example "/dev/i2c-1" 0x48
```
## TODOs ##
* Add support for continous read mode. 

## Acknowledgments
See the list of [contributors](https://github.com/lokraszewski/ADS1115/contributors) who participated in this project.
