 #!/bin/sh

rosrun jsk_ik_server baxter.l
find $(rospack find jsk_ik_server) -name baxter-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;

# Concatenate pickview images
for prefix in xup yup zup
do
    convert +append ../../data/baxter-${prefix}-input-slant.png ../../data/baxter-${prefix}-input-up.png ../../data/baxter-${prefix}-input-side.png ../../data/baxter-${prefix}-input-front.png ../../data/baxter-${prefix}-input-concatenated.png

    for i in $(seq 0 9)
    do
        convert +append ../../data/baxter-${prefix}-iterate$(printf "%03d" ${i})-slant.png ../../data/baxter-${prefix}-iterate$(printf "%03d" ${i})-up.png ../../data/baxter-${prefix}-iterate$(printf "%03d" ${i})-side.png ../../data/baxter-${prefix}-iterate$(printf "%03d" ${i})-front.png ../../data/baxter-${prefix}-iterate$(printf "%03d" ${i})-concatenated.png
    done
done
