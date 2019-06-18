sudo echo SUBSYSTEM==\"usb\", ATTR{idVendor}==\"1234\", MODE=\"0777\" > /etc/udev/rules.d/50-usb-fsdevice.rules
sudo echo KERNEL==\"hidraw*\", ATTRS{busnum}==\"1\", ATTRS{idVendor}==\"1234\", MODE=\"0666\" >> /etc/udev/rules.d/50-usb-fsdevice.rules

