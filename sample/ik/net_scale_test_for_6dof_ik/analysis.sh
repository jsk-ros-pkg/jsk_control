#!/usr/bin/env bash

if [ ! "`ls | grep caffemodel`" ];
then
    cd `rospack find eus_caffe`;
    cd sample/ik/net_scale_test_for_6dof_ik;
fi

for solver in `ls | grep caffemodel`
do
    net_size=`echo $solver | sed -e "s/^ik_net_\(.\+\)_iter_[0-9]\+\.caffemodel$/\\1/g"`;
    iteration=`echo $solver | sed -e "s/^ik_net_.\+_iter_\([0-9]\+\)\.caffemodel$/\\1/g"`;
    echo $net_size $iteration;
    echo "" > analysis_${net_size}.log.raw;
    ## DISPLAY="" NET_SIZE=${net_size} SOLVER_ITERATION=${iteration} roseus analysis.l 1>analysis_${net_size}.log.raw 2>&1  ;
done
