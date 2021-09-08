 #!/bin/sh

rosrun jsk_ik_server hrp2.l
find $(rospack find jsk_ik_server) -name hrp2-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;

# Concatenate pickview images
for prefix in xup yup zup
do
    convert +append ../../data/hrp2-${prefix}-input-slant.png ../../data/hrp2-${prefix}-input-up.png ../../data/hrp2-${prefix}-input-side.png ../../data/hrp2-${prefix}-input-front.png ../../data/hrp2-${prefix}-input-concatenated.png

    for i in $(seq 0 9)
    do
        convert +append ../../data/hrp2-${prefix}-iterate$(printf "%03d" ${i})-slant.png ../../data/hrp2-${prefix}-iterate$(printf "%03d" ${i})-up.png ../../data/hrp2-${prefix}-iterate$(printf "%03d" ${i})-side.png ../../data/hrp2-${prefix}-iterate$(printf "%03d" ${i})-front.png ../../data/hrp2-${prefix}-iterate$(printf "%03d" ${i})-concatenated.png
    done
done
