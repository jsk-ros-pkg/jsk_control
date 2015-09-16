#!/bin/sh

rosrun jsk_ik_server jaxon.l
find $(rospack find jsk_ik_server) -name jaxon-*.csv -exec rosrun jsk_ik_server plot_ik_grid.py {} {}.png \;
