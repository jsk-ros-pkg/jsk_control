#!/usr/bin/env bash

DATE=`date +"%M%I%d%m%Y"`

mkdir -p "tmp.${DATE}";
rm -rf latest.tmp;
ln -s "tmp.${DATE}" latest.tmp;
cd "tmp.${DATE}";

ln -s ../learn.l;
ln -s ../teacher;
cp ../ik_net.prototxt .;
cp ../ik_solver.prototxt .;
cp ../predict_ik_net.prototxt .;

roseus learn.l "(progn (ik-learn) (exit))" > log.learn 2>&1;

cd --;
