 #!/bin/sh

rosrun jsk_ik_server pr2.l
find $(rospack find jsk_ik_server) -name pr2-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;

# Concatenate pickview images
for prefix in xup yup zup
do
    convert +append ../../data/pr2-${prefix}-input-slant.png ../../data/pr2-${prefix}-input-up.png ../../data/pr2-${prefix}-input-side.png ../../data/pr2-${prefix}-input-front.png ../../data/pr2-${prefix}-input-concatenated.png

    for i in $(seq 0 9)
    do
        convert +append ../../data/pr2-${prefix}-iterate$(printf "%03d" ${i})-slant.png ../../data/pr2-${prefix}-iterate$(printf "%03d" ${i})-up.png ../../data/pr2-${prefix}-iterate$(printf "%03d" ${i})-side.png ../../data/pr2-${prefix}-iterate$(printf "%03d" ${i})-front.png ../../data/pr2-${prefix}-iterate$(printf "%03d" ${i})-concatenated.png
    done
done
