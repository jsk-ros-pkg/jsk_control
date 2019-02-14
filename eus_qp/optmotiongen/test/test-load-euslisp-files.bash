#!/bin/bash

cd "$(rospack find eus_qp)"/optmotiongen/euslisp
EUSLISP_FILES="$(ls -1 *.l)"

for f in $EUSLISP_FILES
do
    echo $f
    roseus "(progn (load \"$f\") (exit))"
    if [ $? -ne 0 ]; then
        echo "error when loading $f !!!"
        exit 1
    fi
done

exit 0
