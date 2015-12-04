#!/usr/bin/env bash

DATE=`date +"%M%I%d%m%Y"`

mkdir -p "tmp.${DATE}";
cd "tmp.${DATE}";

ln -s ../learn.l;
ln -s ../teacher;
ln -s ../ik_net.prototxt;
ln -s ../ik_solver.prototxt;
ln -s ../predict_ik_net.prototxt;

roseus learn.l "(progn (ik-learn) (exit))" > log.learn 2>&1;

cd --;
