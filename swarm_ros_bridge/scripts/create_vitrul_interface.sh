sudo ip link add name gs0 type dummy
sudo ip link set gs0 up
sudo ip addr add 172.16.0.200/24 dev gs0

drone_num=$1
i=0
while [ $i -le $((drone_num - 1)) ]
do
    echo "create drone($i) virtual interface"
    sudo ip link add name drone$i type dummy
    sudo ip link set drone$i up
    sudo ip addr add 172.16.0.$((100+i))/24 dev drone$i
    i=$((i+1))
done