# eus_caffe

euslisp wrapper of caffe deep learning package.

## install

`catkin build eus_caffe;`

## demo

```
roscd eus_caffe;
cd sample/linear_equation;
roseus learn.l
(setq *teacher* (gen+3x-2y+4-data :size 3200)) ;; sample 3x-2y+4=0
(caffe::learn :solver "linear_equation.prototxt" :size 3200 :idata (car *teacher*) :ddata (cadr *teacher*))
(caffe::gen-test-net :netproto "predict_linear_equation_net.prototxt" :caffemodel "linear_equation_iter_100000.caffemodel")
(caffe::calc-forward-double :input (float-vector 0 0)) ;; +3*0-2*0+4 = 4
(caffe::calc-forward-double :input (float-vector 0 2)) ;; +3*0-2*2+4 = 0
(caffe::calc-forward-double :input (float-vector 2 5)) ;; +3*2-2*5+4 = 0
```
