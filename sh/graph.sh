#!/usr/bin/env bash

function gen_loss_graph(){
    log=$1;
    xtics_size=1;
    last_x=`tail ${log} --lines=1000 | grep -e "^.\+Iteration \([0-9]\+\), loss = \([0-9\.]\+\)$" | tail -1 | sed -e "s/^.\+Iteration \([0-9]\+\), loss = \([0-9\.]\+\)$/\\1/g"`;
    xtics_scale=`echo "${last_x} / ${xtics_size}" | bc`;
    ## gen log data for graph
    echo "output data -> /tmp/${log}.dat";
    echo -n "" > /tmp/${log}.dat ; ID=0;
    cat ${log}  | grep -e "^.\+Iteration \([0-9]\+\), loss = \([0-9\.]\+\)$" | sed -e "s/^.\+Iteration \([0-9]\+\), loss = \([0-9\.]\+\)$/\\1 \\2/g" | while read d; do
	id=`echo ${d} | sed -e "s/^\(.\+\) .\+$/\\1/g"`;
	val=`echo ${d} | sed -e "s/^.\+ \(.\+\)$/\\1/g"`;
	id2=`echo "$id / ${xtics_scale}" | bc -l`;
        echo $id2 $val >> /tmp/${log}.dat ; ID=`expr $ID + 1`;
    done
    echo " --- output data done, start plotting";
    ##
    gnuplot <<EOF
set terminal postscript eps color enhanced
set output "${log}.eps"
set grid
set size ratio 0.5
set xlabel "STEP/${xtics_scale}"
set ylabel "MSE"
set title "_"
set yrange [ 0 : 1 ]
set mxtics 5
set mytics 5
set xtics 0.1
set ytics 0.1
plot "/tmp/${log}.dat" using 1:2 notitle w l
EOF
    echo " --- plotting done";
}
