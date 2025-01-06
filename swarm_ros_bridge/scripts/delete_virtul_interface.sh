sudo ip link delete name gs0
drone_num=$1
i=0
while [ $i -le $((drone_num - 1)) ]
do
    echo "Delete drone($i) virtual interface"
    sudo ip link delete name drone$i
    i=$((i+1))
done