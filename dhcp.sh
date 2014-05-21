echo "killall udhcpd && ifconfig ath0 down && iwconfig ath0 mode managed essid hier ap any channel auto && ifconfig ath0 192.168.2.187 netmask 255.255.255.0 up && sleep 1" | telnet 192.168.1.1

