#!/usr/bin/env bash

cd `rospack find eus_caffe`;
cd sample/ik/net_scale_test_for_6dof_ik;

for solver in `ls | grep -e "solver_[0-9]\+x[0-9]\+.prototxt$"`
do
    echo $solver;
    DISPLAY="" roseus "../learn.l" "(progn (redundancy-ik-learn :solver \"${solver}\") (exit 0))" 1>test_${solver}.log 2>&1 &
done
