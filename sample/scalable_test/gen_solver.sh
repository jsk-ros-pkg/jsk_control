#!/usr/bin/env bash

for base_lr in 0.01 0.005 0.001;
do
    for id in 0 1 2;
    do
	echo -e "\e[33mbase_lr=${base_lr} x ${id}\e[m";
	roseus gen-solver \
	    "(gen-solver :path \"ik_solver_${base_lr}blr_${id}id.prototxt\" :base_lr ${base_lr} :snapshot_prefix \"\\\"ik_net_${base_lr}blr_${id}id\\\"\")" \
	    "(exit)" > /dev/null 2>&1;
    done
done
