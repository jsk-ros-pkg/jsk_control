 #!/bin/sh

rosrun jsk_ik_server fetch.l
find $(rospack find jsk_ik_server) -name fetch-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;

# Concatenate pickview images
for prefix in xup yup zup
do
    convert +append ../../data/fetch-${prefix}-input-slant.png ../../data/fetch-${prefix}-input-up.png ../../data/fetch-${prefix}-input-side.png ../../data/fetch-${prefix}-input-front.png ../../data/fetch-${prefix}-input-concatenated.png

    for i in $(seq 0 9)
    do
        convert +append ../../data/fetch-${prefix}-iterate$(printf "%03d" ${i})-slant.png ../../data/fetch-${prefix}-iterate$(printf "%03d" ${i})-up.png ../../data/fetch-${prefix}-iterate$(printf "%03d" ${i})-side.png ../../data/fetch-${prefix}-iterate$(printf "%03d" ${i})-front.png ../../data/fetch-${prefix}-iterate$(printf "%03d" ${i})-concatenated.png
    done
done
