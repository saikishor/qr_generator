# qr_generator - Encoding data to QR Code through ROS Interface

A simple ROS package for generating [QR codes](https://en.wikipedia.org/wiki/QR_code). This package is useful to encode data into QR Codes.

## What is Version, Error Correction and Mode?
A QR code is composed of many little squares, called **modules**, which represent encoded data, with additional error correction (allowing partially damaged QR codes to still be read).

The **version** of a QR code is a number between 1 and 40 (inclusive), which indicates the size of the QR code. The width and height of a standard QR code are always equal (it is square) and are equal to `4 * version + 17`. Later, this generated QR is resized to the desired dimensions.

The level of **error correction** is a number between 0 and 3  (inclusive), or can be one of the symbolic names ECC_LOW, ECC_MEDIUM, ECC_QUARTILE and ECC_HIGH. Higher levels of error correction sacrifice data capacity, but allow a larger portion of the QR code to be damaged or unreadable.

## Functionality
**Launching the node**
`roslaunch qr_generator qr_generator.launch`

**Requesting for a QR Code**
```
rosservice call /qr_generator_ros/generate_qr "data: 'I will display ♠'
error_correction_type: 2
qr_version: 6" 
```
**Generated QR Code**
![Image of generated QR Code(QR Version: 6)](/qr.png)
