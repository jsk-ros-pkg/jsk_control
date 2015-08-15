#!/usr/bin/env bash

RET="\begin{table}[htb]
  \begin{tabular}{|l|r|r||r|r|r||r|r|} \hline
    Depth x Width & Time & $p_{x}$ & $p_{y}$ & $p_{z}$ & $r_{x}$ & $r_{y}$ & $r_{z}$ \\ \hline \hline
  \end{tabular}
\end{table}";

function eus_bc(){
    ##roseus
    `which eus` "(let* ((out (open \"/tmp/eus_bc.log\" :direction :output))) (format out \"~A~%\" (eval (read-from-string \"$1\"))) (close out) (exit))" 2>/dev/null 1>/dev/null;
    cat /tmp/eus_bc.log | tail -1;
}

function format_string(){
    TARGET=$1;
    ID=1;
    VAL=`echo $TARGET | cut -c $ID`;
    RET="";
    CDOWN="";
    while [ "$VAL" ];
    do
	if [ "$CDOWN" ];
	then
	    CDOWN=`expr $CDOWN - 1`;
	    if [ "$CDOWN" -lt "0" ];
	    then
		break;
	    fi
	else
	    if [ "$VAL" == "." ];
	    then
		CDOWN=1;
	    fi
	fi
	RET=$RET$VAL;
	## echo $RET;
	ID=`expr $ID + 1`;
	VAL=`echo $TARGET | cut -c $ID`;
    done
    echo $RET;
}

function gen_table(){
    TEST="test";
    AVER="average";
    if [ "$1" ];
    then
	TEST=$1;
    fi
    if [ "$2" ];
    then
	AVER=$2;
    fi
    echo "";
    ##
    echo "\begin{table}[htb]
  \begin{center}
  \caption{${AVER} for ${TEST} data set\\\\ conputational time and distance between target coordinates}
  \label{tab:${AVER}_diff_for_${TEST}_data}
    \begin{tabular}{|l||r|r||r|r|r||r|r|} \\hline
      Depth & Time & \$p_{x}$ & \$p_{y}$ & \$p_{z}$ & \$r_{x}$ &
      \$r_{y}$ & \$r_{z}$ \\\\
      x Width & [us] & [mm] & [mm] & [mm] & [deg] & [deg] & [deg]
      \\\\ \\hline \\hline";
    ##
    ls | grep -e "analysis_.\+\.log\.${TEST}" | grep -v "x50\." | grep -v "7x"| while read line;
    do
	SIZE=`echo $line | sed -e "s/^analysis_\(.\+\)\.log\.${TEST}$/\\1/g"`;
	DEPTH=`echo $SIZE | sed -e "s/^\([0-9]\+\)x.\+$/\\1/g"`;
	WIDTH=`echo $SIZE | sed -e "s/^.\+x\([0-9]\+\)$/\\1/g"`;
	id=0;
	scale=(1000000 1 1 1 57.3 57.3 57.3);
	echo -n "$SIZE";
	for val in `tail $line | grep -A 1 ":$AVER" | tail -1`;
	do
	    if [ "$val" ];
	    then
	    ## echo "";
	    ## echo "echo \"${scale[$id]} * $val\" | bc -l";
	    ## echo ""+
	    ## val=`echo "${scale[$id]} * $val" | bc -l`;
		val=`eus_bc "(* ${scale[$id]} $val)"`;
		echo -n " & `format_string \"$val\"`";
		id=`expr $id + 1`;
	    fi
	done
	echo " \\\\ \\hline";
    done
    echo "    \end{tabular}
  \end{center}
\end{table}";
}

function gen_net_config(){
    echo "";
    ##
    echo "\begin{table}[htb]
  \begin{center}
  \caption{Network configuration: Depth x Width}
  \label{tab:net_config}
    \begin{tabular}{|l||r|r|r|r|r|r|} \\hline
      Depth & layer1 & layer2 & layer3 & layer4 & layer5 & layer6 \\\\
      x Width & cells & cells & cells & cells & cells & cells
      \\\\ \\hline \\hline";
    ##
    ls | grep -e "ik_net_.\+\.prototxt" | grep -v "x50\." | grep -v "7x" | grep -v "predict" | while read line;
    do
	SIZE=`echo $line | sed -e "s/^ik_net_\(.\+\)\.prototxt$/\\1/g"`;
	DEPTH=`echo $SIZE | sed -e "s/^\([0-9]\+\)x.\+$/\\1/g"`;
	WIDTH=`echo $SIZE | sed -e "s/^.\+x\([0-9]\+\)$/\\1/g"`;
	id=0;
	echo -n "$SIZE";
	for val in `cat $line | grep num_output | sed -e "s/^ *num_output: \(.\+\)$/\\1/g"`;
	do
	    if [ "$val" ];
	    then
		if [ "$id" -lt "$DEPTH" ];
		then
		    echo -n " & $val";
		    id=`expr $id + 1`;
		fi
	    fi
	    if [ "$id" -ge "$DEPTH" ];
	    then
		while [ "$id" -le "5" ];
		do
		    echo -n " & \-";
		    id=`expr $id + 1`;
		done
	    fi
	done
	echo " \\\\ \\hline";
    done
    echo "    \end{tabular}
  \end{center}
\end{table}";
}

gen_net_config
gen_table "test" "average";
gen_table "test" "variance";
gen_table "train" "average";
gen_table "train" "variance";
