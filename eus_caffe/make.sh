#!/usr/bin/env bash

DIR_3rdparty="";
DIR_ROOT="";
LOG_HEAD="_/_/_/_/_/_/_/_/_/";

function _mlog () {
    echo $LOG_HEAD;
    echo $1;
    echo $LOG_HEAD;
}

if [ ! -e "3rdparty" ];
then
    _mlog "gen 3rdparty dir";
    mkdir 3rdparty;
fi
cd 3rdparty;

if [ ! -e "caffe" ];
then
    ##
    ##
    _mlog "download caffe";
    wget https://github.com/BVLC/caffe/archive/d362894887af9dca8581906b2284f5be81dbd403.zip
    unzip d362894887af9dca8581906b2284f5be81dbd403.zip;
    mv caffe-d362894887af9dca8581906b2284f5be81dbd403 caffe;
    ## mv caffe-master caffe;
    ## git clone https://github.com/BVLC/caffe.git;
    cd caffe;
    DIR_ROOT=`pwd`;
    ##
    ## sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libboost-all-dev libhdf5-serial-dev libatlas-base-dev; ## move to rosdep
    ##
    mkdir 3rdparty;
    cd 3rdparty;
    DIR_3rdparty=`pwd`;
    ##
    ## glog
    _mlog "download glog";
    ## wget https://google-glog.googlecode.com/files/glog-0.3.3.tar.gz
    wget https://github.com/google/glog/archive/0b0b022be1c9c9139955af578fe477529d4b7b3c.zip;
    mv 0b0b022be1c9c9139955af578fe477529d4b7b3c.zip glog-0.3.3.zip;
    unzip glog-0.3.3.zip;
    mv glog-0b0b022be1c9c9139955af578fe477529d4b7b3c glog-0.3.3;
    cd glog-0.3.3;
    _mlog "build glog";
    ./configure;
    make;
    ## make && make install;
    cd $DIR_3rdparty;
    _mlog "done glog";
    ##
    ## gflags
    _mlog "download gflag";
    wget https://github.com/gflags/gflags/archive/d4e971c10b1557292b5371807a23921d15e7fece.zip;
    unzip d4e971c10b1557292b5371807a23921d15e7fece.zip;
    mv gflags-d4e971c10b1557292b5371807a23921d15e7fece gflags;
    cd gflags;
    _mlog "build gflag";
    mkdir build && cd build
    export CXXFLAGS="-fPIC" && cmake .. && make VERBOSE=1
    ## make && make install
    make;
    cd $DIR_3rdparty;
    _mlog "done gflag";
    ##
    ## lmdb
    _mlog "download lmdb";
    wget https://github.com/LMDB/lmdb/archive/7e476e4983cfba45cefe793b8bd6e13c486b3989.zip;
    unzip 7e476e4983cfba45cefe793b8bd6e13c486b3989.zip;
    mv lmdb-7e476e4983cfba45cefe793b8bd6e13c486b3989 mdb;
    cd mdb/libraries/liblmdb;
    _mlog "build lmdb";
    make;
    ## make && make install
    cd $DIR_3rdparty;
    _mlog "done lmdb";
    ##
    cd $DIR_ROOT;
    if [ ! -e "Makefile.config" ];
    then
	_mlog "gen caffe build config";
	cat Makefile.config.example | sed -e "s#/usr/local/lib#/usr/local/lib ${DIR_ROOT}/3rdparty/gflags/build/lib ${DIR_ROOT}/3rdparty/mdb/libraries/liblmdb ${DIR_ROOT}/3rdparty/glog-0\.3\.3/\.libs#g" | sed -e "s#/usr/local/include#/usr/local/include ${DIR_ROOT}/3rdparty/gflags/build/include#g" | sed -e "s/# CUSTOM_CXX := g++/CUSTOM_CXX := g++/g" > Makefile.config;
	cat Makefile.config;
    fi
    _mlog "build caffe";
    CPU_ONLY=1 make;
    # ## python
    if [ "$PYCAFFE_INSTALL" ];
    then
	# wget -O- http://neuro.debian.net/lists/precise.jp.libre | sudo tee /etc/apt/sources.list.d/neurodebian.sources.list;
	# sudo apt-key adv --recv-keys --keyserver hkp://pgp.mit.edu:80 0xA5D32F012649A5A9;
	# sudo aptitude install python-skimage;
	sudo apt-get install python-pip
	for req in $(cat python/requirements.txt); do sudo pip install $req; done
	CPU_ONLY=1 make pycaffe;
    fi
    ##
    _mlog "done caffe";
fi
