cp can0 /etc/network/interfaces.d/can0
cp isotp@.service /etc/systemd/system/isotp@.service
cp config/* /etc/default/

echo 'Do not forget to enable the services after this script (see the README) and restart.'
