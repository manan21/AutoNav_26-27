DEV="/dev/serial/by-id/usb-RoboteQ_RoboteQ_FBLG2360T_IwBCAAlQMTlZMDIg_2074307C3931-if00"                                            

stty -F "$DEV" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo

printf '%%RESET 321654987\r' > "$DEV"
sleep 3                                        

printf '?FF\r' > "$DEV"
printf '?FS\r' > "$DEV"
printf '?FM 1\r' > "$DEV"
printf '?FM 2\r' > "$DEV"
printf '?V 2\r' > "$DEV"
printf '?STT\r' > "$DEV"