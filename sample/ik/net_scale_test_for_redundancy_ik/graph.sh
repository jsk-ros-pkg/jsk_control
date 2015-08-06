#!/usr/bin/env bash

source `rospack find eus_caffe`/sh/graph.sh;

for log in `ls | grep -e "test_ik" | grep -e "\.prototxt\.log$"`;
do
    gen_loss_graph $log;
done;
