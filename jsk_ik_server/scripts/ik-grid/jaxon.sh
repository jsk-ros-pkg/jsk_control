#!/bin/sh

rosrun jsk_ik_server jaxon.l
find $(rospack find jsk_ik_server) -name jaxon-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;

# Concatenate pickview images
for prefix in xup yup zup
do
    convert +append ../../data/jaxon-${prefix}-input-slant.png ../../data/jaxon-${prefix}-input-up.png ../../data/jaxon-${prefix}-input-side.png ../../data/jaxon-${prefix}-input-front.png ../../data/jaxon-${prefix}-input-concatenated.png

    for i in $(seq 0 9)
    do
        convert +append ../../data/jaxon-${prefix}-iterate$(printf "%03d" ${i})-slant.png ../../data/jaxon-${prefix}-iterate$(printf "%03d" ${i})-up.png ../../data/jaxon-${prefix}-iterate$(printf "%03d" ${i})-side.png ../../data/jaxon-${prefix}-iterate$(printf "%03d" ${i})-front.png ../../data/jaxon-${prefix}-iterate$(printf "%03d" ${i})-concatenated.png
    done
done
