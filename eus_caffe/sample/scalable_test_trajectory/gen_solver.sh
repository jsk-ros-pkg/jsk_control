#!/usr/bin/env bash

if [ ! "$BASE_LR_LIST" ];
then
    if [ "$1" ];
    then
        BASE_LR_LIST=$1;
    else
        BASE_LR_LIST="0.004 0.002 0.001";
    fi
fi

for base_lr in $BASE_LR_LIST;
do
    for id in 0 1 2;
    do
        echo -e "\e[33mbase_lr=${base_lr} x ${id}\e[m";
        roseus gen-solver \
            "(gen-solver :path \"traj_solver_${base_lr}blr_${id}id.prototxt\" :base_lr ${base_lr} :snapshot_prefix \"\\\"traj_net_${base_lr}blr_${id}id\\\"\")" \
            "(exit)" > /dev/null 2>&1;
    done
done
