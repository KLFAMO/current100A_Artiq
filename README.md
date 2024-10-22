# current100A_Artiq
100A current driver compatible with Artiq. At the moment, device is compatible with Sinara package size and includes all Sinara connectors. There is no software compatibility yet.

# User Interface

User interface is available via TCP/IP on port 10. Depending on configuration IP address is static or taken from DHCP. After connecting to the device user is able to change setting by sending commands.

## Commands

`MODE 0` - switch off current

`MODE 1` - current control via voltage input (SMA input on front panel)

`MODE 2` - current control via ethernet user interface (`CUR` command)

`CUR 10.2` - set current 10.2 A (only in mode 2)

Please add your comments in `Issues` Github section.