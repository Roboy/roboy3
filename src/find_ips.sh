# sudo nmap -sn 192.168.1.0/24 > /dev/null 2>&1

raspberrypi=$(arp -n | grep -w -i 'B8:27:EB:B0:E7:9C' | awk '{print $1}')
fpga=$(arp -n | grep -w -i '00:00:F3:BE:EF:01' | awk '{print $1}')
projector=$(arp -n | grep -w -i '00:0a:cd:32:ae:b7' | awk '{print $1}')

echo "beamer:" $projector 
echo "raspberrypi:" $raspberrypi
echo "fpga:" $fpga
