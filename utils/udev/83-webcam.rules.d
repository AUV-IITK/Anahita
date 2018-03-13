ACTION=="add", KERNEL=="video0", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0843", SYMLINK+="front_cam"
ACTION=="add", KERNEL=="video1", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0843", SYMLINK+="bottom_cam"
