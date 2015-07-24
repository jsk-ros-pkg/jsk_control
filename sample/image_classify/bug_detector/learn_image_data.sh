#!/usr/bin/env bash

cd `rospack find eus_caffe`/sample/image_classify;

bash `rospack find eus_caffe`/sample/image_classify/gen_image_data.sh;
roseus learn.l "(progn (gen-learning-data) (db-image-learn))";
