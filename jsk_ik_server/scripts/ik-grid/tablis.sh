#!/bin/sh

rosrun jsk_ik_server tablis.l
find $(rospack find jsk_ik_server) -name tablis-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;

# Concatenate pickview images
for prefix in xup yup zup
do
    convert +append ../../data/tablis-${prefix}-input-slant.png ../../data/tablis-${prefix}-input-up.png ../../data/tablis-${prefix}-input-side.png ../../data/tablis-${prefix}-input-front.png ../../data/tablis-${prefix}-input-concatenated.png

    for i in $(seq 0 9)
    do
        convert +append ../../data/tablis-${prefix}-iterate$(printf "%03d" ${i})-slant.png ../../data/tablis-${prefix}-iterate$(printf "%03d" ${i})-up.png ../../data/tablis-${prefix}-iterate$(printf "%03d" ${i})-side.png ../../data/tablis-${prefix}-iterate$(printf "%03d" ${i})-front.png ../../data/tablis-${prefix}-iterate$(printf "%03d" ${i})-concatenated.png
    done
done
