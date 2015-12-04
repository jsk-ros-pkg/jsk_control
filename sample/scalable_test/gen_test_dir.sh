#!/usr/bin/env bash

if [ ! -e "gen_test_dir.sh" ];
then
    if [ "`rospack find eus_caffe`" ];
    then
	roscd eus_caffe;
    elif [ "`locate eus_caffe | grep "src" | grep -e "eus_caffe$" | tail -1`" ];
    then
	cd `locate eus_caffe | grep "src" | grep -e "eus_caffe$" | tail -1`
    fi
    ##
    if [ ! -e "gen_test_dir.sh" ];
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
cp $ORG_DIR/ik_solver.prototxt .;
cp $ORG_DIR/predict_ik_net.prototxt .;

echo -e "\e[33mrun learning ... \e[m";
roseus learn.l "(progn (ik-learn) (exit))" > log.learn 2>&1;

cd $ORG_DIR;
