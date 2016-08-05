#!/usr/bin/env bash

LAYER="layer {
  name: \"ip@depth@\"
  type: \"InnerProduct\"
  top: \"ip@depth@\"
  bottom: \"sigmoid@depth_1@\"
  param {
    lr_mult: 1
    decay_mult: 1
  }
  param {
    lr_mult: 1
    decay_mult: 0
  }
  inner_product_param {
    num_output: @WIDTH@
    weight_filler {
      type: \"gaussian\"
      std: 1
      sparse: 1
    }
    bias_filler {
      type: \"constant\"
      value: 0
    }
  }
}
layer {
  name: \"sigmoid@depth@\"
  type: \"Sigmoid\"
  top: \"sigmoid@depth@\"
  bottom: \"ip@depth@\"
}" ;

for scale in 4x100 4x200 4x300 5x100 5x200 5x300 6x100 6x200 6x300;
do
    DEPTH=`echo $scale | sed "s/^\(.\+\)x.\+$/\\1/g"`;
    WIDTH=`echo $scale | sed "s/^.\+x\(.\+\)$/\\1/g"`;
    dw=`expr ${WIDTH} - 40`;
    ds=`expr ${DEPTH} - 1`;
    dw=`expr ${dw} / ${ds}`;
    depth=1;
    ##
    sed "s/@SCALE@/${scale}/g" ik_solver.prototxt > ik_solver_${scale}.prototxt;
    sed "s/@DEPTH@/${DEPTH}/g" ik_net1.prototxt > ik_net_${scale}.prototxt;
    sed "s/@DEPTH@/`expr $depth - 1`/g" ik_net1_predict.prototxt > ik_net_${scale}_predict.prototxt;
    ##
    while [ "$depth" -le "$DEPTH" ];
    do
	echo "$LAYER" | sed "s/@DEPTH@/${DEPTH}/g" | sed "s/@WIDTH@/${WIDTH}/g" | sed "s/@depth@/${depth}/g" | sed "s/@depth_1@/`expr $depth - 1`/g" > /tmp/buf;
	cat /tmp/buf >> ik_net_${scale}.prototxt;
	cat /tmp/buf >> ik_net_${scale}_predict.prototxt;
	depth=`expr $depth + 1`;
	WIDTH=`expr ${WIDTH} - ${dw}`;
    done
    sed "s/@DEPTH@/${DEPTH}/g" ik_net2.prototxt >> ik_net_${scale}.prototxt;
    sed "s/@DEPTH@/${DEPTH}/g" ik_net2_predict.prototxt >> ik_net_${scale}_predict.prototxt;
done
