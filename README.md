# Argus Camera ROS

## Notes
For timestamp mode to work with `TIME_FROM_PTP`, one must do the following:

```bash
sudo vim /etc/udev/rules.d/99-nvpps.rules

# add the following inside
KERNEL=="nvpps0", MODE="0666"

# Then, run the following commands, or reboot
sudo udevadm control --reload
sudo udevadm trigger
```
