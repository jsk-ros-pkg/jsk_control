#!/usr/bin/env bash

LAYER1="layer {
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

LAYER2="layer {
  name: \"_ip@depth@\"
  type: \"InnerProduct\"
  top: \"_ip@depth@\"
  bottom: \"_sigmoid@depth_1@\"
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
  name: \"_sigmoid@depth@\"
  type: \"Sigmoid\"
  top: \"_sigmoid@depth@\"
  bottom: \"_ip@depth@\"
}" ;


## for scale in 3x50 3x100 3x200 3x400 4x50 4x100 4x200 4x400 5x50 5x100 5x200 5x400;
## do
scale=5x200;
DEPTH=`echo $scale | sed "s/^\(.\+\)x.\+$/\\1/g"`;
WIDTH=`echo $scale | sed "s/^.\+x\(.\+\)$/\\1/g"`;
dw=`expr ${WIDTH} - 20`;
ds=`expr ${DEPTH} - 1`;
dw=`expr ${dw} / ${ds}`;
depth=0;
##
sed "s/@SCALE@/${scale}/g" ik_solver.prototxt > ik_solver_${scale}.prototxt;
sed "s/@SCALE@/${scale}/g" ik_NESTEROV_solver.prototxt > ik_NESTEROV_solver_${scale}.prototxt;
sed "s/@SCALE@/${scale}/g" ik_ADAGRAD_solver.prototxt > ik_ADAGRAD_solver_${scale}.prototxt;
##
sed "s/@DEPTH@/${depth}/g" ik_net1.prototxt > ik_net_${scale}.prototxt;
depth=`expr $depth + 1`;
##
while [ "$depth" -le "$DEPTH" ];
do
    echo "$LAYER1" | sed "s/@DEPTH@/${DEPTH}/g" | sed "s/@WIDTH@/${WIDTH}/g" | sed "s/@depth@/${depth}/g" | sed "s/@depth_1@/`expr $depth - 1`/g" >> ik_net_${scale}.prototxt;
    depth=`expr $depth + 1`;
    WIDTH=`expr ${WIDTH} - ${dw}`;
done
##
depth=`expr $depth - 1`;
sed "s/@DEPTH@/${depth}/g" ik_net2.prototxt | sed "s/@DEPTH_1@/`expr ${depth} + 1`/g" >> ik_net_${scale}.prototxt;
##
WIDTH=`echo $scale | sed "s/^.\+x\(.\+\)$/\\1/g"`;
while [ "$depth" -ge "1" ];
do
    echo "$LAYER2" | sed "s/@DEPTH@/${DEPTH}/g" | sed "s/@WIDTH@/${WIDTH}/g" | sed "s/@depth@/${depth}/g" | sed "s/@depth_1@/`expr $depth + 1`/g" >> ik_net_${scale}.prototxt;
    depth=`expr $depth - 1`;
    WIDTH=`expr ${WIDTH} - ${dw}`;
done
##
sed "s/@DEPTH@/${depth}/g" ik_net3.prototxt | sed "s/@DEPTH_1@/`expr ${depth} + 1`/g" >> ik_net_${scale}.prototxt;
## done
