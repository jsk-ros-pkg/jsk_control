#!/usr/bin/env bash

source `rospack find eus_caffe`/sh/scraping.sh;

mkdir image;
mkdir image_org;

for name in `echo "cockroach spider mosquitto cat rat human"`;
do
    ID=0;
    mkdir image/${name};
    mkdir image_org/${name};
    dl_images_loop "${name}" 20 "image_org/${name}";
    for p in `ls image_org/${name}`;
    do
	convert -resize 64x64! image_org/${name}/$p image/${name}/${ID}.jpg;
	ID=`expr $ID + 1`;
    done
done
