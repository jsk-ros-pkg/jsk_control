#!/usr/bin/env bash

if [ ! "`basename \`pwd\``" == "scalable_test" ]
then
    if [ "`rospack find eus_caffe`" ];
    then
	cd "`rospack find eus_caffe`";
    elif [ "`locate eus_caffe | grep "src" | grep -e "eus_caffe$" | tail -1`" ];
    then
	cd `locate eus_caffe | grep "src" | grep -e "eus_caffe$" | tail -1`
    fi
    ##
    if [ ! "`basename \`pwd\``" == "scalable_test" ]
    then
	echo -e "\e[31meus_caffe not found\e[m"
	exit -1;
    fi
    ##
    cd sample/scalable_test;
fi

DATE="`date +"%S%M%I%d%m%Y"`_$$"
ORG_DIR=`pwd`;
echo -e "\e[33morg_dir: ${ORG_DIR}\e[m";
TMP_DIR="tmp/${DATE}";
echo -e "\e[33morg_dir: ${TMP_DIR}\e[m";
mkdir -p $TMP_DIR;

echo -e "\e[33mgenerate links ... \e[m";
rm -rf latest.tmp;
ln -s $TMP_DIR latest.tmp;
cd $TMP_DIR;

ln -s $ORG_DIR/learn.l;
mkdir teacher;
ln -s $ORG_DIR/teacher/angle-vector-map.dat teacher/;
ln -s $ORG_DIR/teacher/ef-coords-map.dat teacher/;
cp $ORG_DIR/ik_net.prototxt .;
## cp $ORG_DIR/ik_solver.prototxt .;
cp $ORG_DIR/gen_solver.sh .;
cp $ORG_DIR/gen-solver.l .;
cp $ORG_DIR/predict_ik_net.prototxt .;

echo -e "\e[33mgen solver ... \e[m";
./gen_solver.sh;

echo -e "\e[33mstart nan killer ...\e[m";
# ( touch log.learn; while [ ! "`tail log.learn | grep \" = \" | grep \"nan\|inf\"`" ]; do sleep 3; done; echo -e "\e[31mnan detected\e[m"; echo -e "\e[31mkill $$\e[m"; kill $$;  ) &
(
    sleep 10;
    PID=`lsof log.learn 2> /dev/null | grep irteusgl | sed -e "s/^irteusgl \([0-9]\+\) .\+$/\\1/g"`;
    while [ "$PID" ];
    do
        PID=`lsof log.learn 2> /dev/null | grep irteusgl | sed -e "s/^irteusgl \([0-9]\+\) .\+$/\\1/g"`;
        if [ "`tail log.learn | grep \" = \" | grep \"nan\|inf\"`" ];
        then
            echo -e "\e[31mnan detected\e[m";
            echo -e "\e[31mkill $PID\e[m"
            kill $PID;
        fi
        sleep 3;
    done
) &

echo -e "\e[33mrun learning ... \e[m";
roseus learn.l "(bench (progn (print (ik-learn-with-all-solver)) (print (ik-learn-best-loop :max_iter 100000 :lp 40 :vain-cnt-max 20))))" "(exit)" > log.learn 2>&1;

echo -e "\e[33m${TMP_DIR} done\e[m";
tail log.learn;

head ik_net.prototxt -n 1;

cd $ORG_DIR;

