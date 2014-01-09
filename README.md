Quick (and dirty) demo with nrf24l01 radio device.

Hardware:
- stellaris launchpad (tiva now);
- nrf24l01 module

The device can be either receiver of transmitter.

The transmitter transmits any data received by uart or
if there is no data dummy packet. If the packet is acked
blue LED is on, otherwise red led is on.

The receiver transmits to uart any data received by nrf24l01
If data received by nrf24l01 the green led is on, if no data
received during timeout the led is red.
