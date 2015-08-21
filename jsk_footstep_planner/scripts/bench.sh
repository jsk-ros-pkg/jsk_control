#!/bin/sh


MIN_X=-3.0
MAX_X=3.0
DX=0.5
MIN_Y=-3.0
MAX_Y=3.0
DY=0.5
RESOLUTION_X=0.1
RESOLUTION_Y=0.1
RESOLUTION_THETA=0.2
N_THETA=8
HEURISTIC="step_cost"
FIRST_ROTATION_WEIGHT=1.0
SECOND_ROTATION_WEIGHT=1.0
MODEL="none"

# or empty
ENABLE_LAZY_PERCEPTION=""
ENABLE_LOCAL_MOVEMENT=""


TEST_COUNT=5
OUTPUT=output.csv

run() {
    rosrun jsk_footstep_planner bench_footstep_planner --min_x ${MIN_X} --max_x ${MAX_X} \
       --min_y ${MIN_Y} --max_y ${MAX_Y} \
       --dx ${DX} --dy ${DY} \
       --resolution_x ${RESOLUTION_X} --resolution_y ${RESOLUTION_Y} --resolution_theta ${RESOLUTION_THETA} \
       --n_theta ${N_THETA} --heuristic ${HEURISTIC} \
       --first_rotation_weight ${FIRST_ROTATION_WEIGHT} --second_rotation_weight ${SECOND_ROTATION_WEIGHT} \
       ${ENABLE_LAZY_PERCEPTION} ${ENABLE_LOCAL_MOVEMENT} \
       --model ${MODEL} \
       --test_count ${TEST_COUNT}\
       -o ${OUTPUT}
    echo --min_x ${MIN_X} --max_x ${MAX_X} \
       --min_y ${MIN_Y} --max_y ${MAX_Y} \
       --dx ${DX} --dy ${DY} \
       --resolution_x ${RESOLUTION_X} --resolution_y ${RESOLUTION_Y} --resolution_theta ${RESOLUTION_THETA} \
       --n_theta ${N_THETA} --heuristic ${HEURISTIC} \
       --first_rotation_weight ${FIRST_ROTATION_WEIGHT} --second_rotation_weight ${SECOND_ROTATION_WEIGHT} \
       ${ENABLE_LAZY_PERCEPTION} ${ENABLE_LOCAL_MOVEMENT} \
       --model ${MODEL} \
       --test_count ${TEST_COUNT}\
       -o ${OUTPUT}
}

# First, evaluate heuristics with empty model
MODEL="none"
for heuristic in straight step_cost straight_rotation
do
    HEURISTIC=$heuristic
    MODEL="none"
    OUTPUT="bench_result_$heuristic.csv"
    run
done

# Next, evaluate weight of step_cost
HEURISTIC="step_cost"
MODEL="none"
for a in $(seq 0 0.1 2)
do
    for b in $(seq 0 0.1 2)
    do
        FIRST_ROTATION_WEIGHT=$a
        SECOND_ROTATION_WEIGHT=$b
        OUTPUT="bench_result_weight_${a}_${b}.csv"
        run
    done
done

# Different resolution
HEURISTIC="step_cost"
MODEL="none"
RESOLUTION_X=0.05
RESOLUTION_Y=0.05
RESOLUTION_THETA=0.1
OUTPUT="bench_result_smaller_resolution.csv"
run

RESOLUTION_X=0.1
RESOLUTION_Y=0.1
RESOLUTION_THETA=0.2
FIRST_ROTATION_WEIGHT=1.0
SECOND_ROTATION_WEIGHT=1.0
HEURISTIC="step_cost"
# Different models
for model in flat stairs hilld
do
    for lazy in true false
    do
        for local_movement in true false
        do
            if [ "$lazy" = "true" ]; then
                ENABLE_LAZY_PERCEPTION="--enable_lazy_perception"
            else
                ENABLE_LAZY_PERCEPTION=""
            fi
            if [ "$local_movement" = "true" ]; then
                ENABLE_LOCAL_MOVEMENT="--enable_local_movement"
            else
                ENABLE_LOCAL_MOVEMENT=""
            fi
            MODEL=$MODEL
            OUTPUT="bench_result_${model}_lazy_${lazy}_local_${local_movement}.csv"
            run
        done
    done
done
