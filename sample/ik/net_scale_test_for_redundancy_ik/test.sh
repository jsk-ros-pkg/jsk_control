#!/usr/bin/env bash

if [ ! "`ls | grep -e "solver_[0-9]\+x[0-9]\+.prototxt$"`" ];
then
    cd `rospack find eus_caffe`;
    cd sample/ik/net_scale_test_for_redundancy_ik;
fi

for solver in `ls | grep -e "solver_[0-9]\+x[0-9]\+.prototxt$"`
do
    echo $solver;
    DISPLAY="" roseus "../learn.l" "(progn (redundancy-ik-learn :solver \"${solver}\") (exit 0))" 1>test_${solver}.log 2>&1 &
    sleep 1;
done
