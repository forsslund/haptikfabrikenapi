#sudo echo SUBSYSTEMS==\"usb\", ATTR{idVendor}==\"1234\", ATTRS{idProduct}==\"0006\", GROUP=\"users\", MODE=\"0666\" > /etc/udev/rules.d/50-usb-fsdevice.rules
#sudo echo KERNEL==\"hidraw*\", ATTRS{idVendor}==\"1234\", MODE=\"0666\" >> /etc/udev/rules.d/50-usb-fsdevice.rules
#sudo echo KERNEL==\"hidraw*\", ATTRS{busnum}==\"3\", ATTRS{idVendor}==\"1234\", MODE=\"0666\" >> /etc/udev/rules.d/50-usb-fsdevice.rules

# For newer teensy-based devices
sudo cp 49-teensy.rules /etc/udev/rules.d/49-teensy.rules

