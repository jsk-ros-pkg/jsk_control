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

DATE=`date +"%S%M%I%d%m%Y"`
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
ln -s $ORG_DIR/teacher;
cp $ORG_DIR/ik_net.prototxt .;
## cp $ORG_DIR/ik_solver.prototxt .;
cp $ORG_DIR/gen_solver.sh .;
cp $ORG_DIR/gen-solver.l .;
cp $ORG_DIR/predict_ik_net.prototxt .;

echo -e "\e[33mgen solver ... \e[m";
./gen_solver.sh;
echo -e "\e[33mrun learning ... \e[m";
roseus learn.l "(progn (print (ik-learn-with-all-solver)) (exit))" > log.learn 2>&1;
echo -e "\e[33m${TMP_DIR} done\e[m";
tail log.learn;
head ik_net.prototxt -n 1;

cd $ORG_DIR;
