#!/usr/bin/env bash

cd `rospack find eus_caffe`;
cd sample/ik/net_scale_test_for_6dof_ik;

for net_size in `ls | grep caffemodel | sed -e "s/^ik_net_\(.\+\)_iter_3000000\.caffemodel$/\\1/g"`
do
    echo $net_size;
    echo "" > analysis_${net_size}.log.raw;
    DISPLAY="" NET_SIZE=${net_size} roseus analysis.l 1>analysis_${net_size}.log.raw 2>&1  ;
done
