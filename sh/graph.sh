#!/usr/bin/env bash

function gen_loss_graph(){
    log=$1;
    ## gen log data for graph
    echo -n "" > /tmp/${log}.dat ; ID=0;
    for d in `grep ": loss =" ${log}  | sed -e "s/^.\+: loss = \([^ ]\+\) .\+$/\\1/g"`; do
        echo $ID $d >> /tmp/${log}.dat ; ID=`expr $ID + 1`;
    done
    ##
    gnuplot <<EOF
set terminal postscript eps color enhanced
set output "${log}.eps"
set xlabel "STEP"
set ylabel "LOSS"
set title "loss/step for ${log}"
set yrange [ 0 : 2 ]
set mxtics 5
set mytics 5
set xtics 5
set ytics 0.5
plot "/tmp/${log}.dat" using 1:2 notitle w l
EOF
}
