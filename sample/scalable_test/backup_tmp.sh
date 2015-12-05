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
    if [ !  "`basename \`pwd\``" == "scalable_test" ];
    then
	echo -e "\e[31meus_caffe not found\e[m"
	exit -1;
    fi
    ##
    cd sample/scalable_test;
fi

if [ -e "tmp" ];
then
    cd tmp;
    if [ ! -e "backup" ];
    then
	mkdir "backup";
    fi
    mv * backup;
fi
