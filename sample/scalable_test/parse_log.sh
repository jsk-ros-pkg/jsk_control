#!/usr/bin/env bash

for p in `ls`;
do
    ## p="${_p}";
    if [ -e "${p}/log.learn" ];
    then
        echo -e "\e[36m ----------- \e[m";
        echo -e "\e[36m`head ${p}/predict_ik_net.prototxt -n 1`\e[m";
        echo -n -e "\e[36m${p}:  ";
        cat ${p}/log.learn | grep vain | grep now \
            | sed -e "s/^.\+now: \([0-9\.+-e]\+\)$/\\1/g" \
            | awk '{if (NR==1) min=$1} {if($1 < min) min=$1} END {print min}';
        echo -n -e "\e[m";
        cat ${p}/log.learn | grep vain | grep now | awk '{print "  " $0}';
    fi
done

