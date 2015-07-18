#!/usr/bin/env bash

## QUERY=apple;
## wget https://www.google.co.jp/search?q=${QUERY}&tbm=isch;

function get_tag_value(){
    HTML_PATH=$1;
    TAG=$2;
    if [ "$3" ];
    then
	## echo `cat $HTML_PATH | grep "<$TAG"  | sed -e "s/^.*<$TAG .*$3=\(.\+\)>.*$/\\1/g"` ;
	for l in `cat $HTML_PATH | grep "<$TAG"  | sed -e "s/>/\n/g"` ;
	do
	    if [ "`echo $l | grep $3=`" ];
	    then
		echo `echo $l | sed -e "s/$3=//g"`;
	    fi
	done
    else
	echo `cat $HTML_PATH | grep "<$TAG"  | sed -e "s/^.*<$TAG>\(.\+\)<\/$TAG>.\+$/\\1/g"` ;
    fi
}

function dl_images(){
    UA=Mozilla/5
    QUERY=$1;
    START=$2;
    if [ ! "$QUERY" ];
    then
	echo no query detected;
	return 0;
    fi
    if [ ! "$START" ];
    then
	START=0;
    fi
    URL="http://www.google.com/search?q=${QUERY}&tbm=isch&start=${START}";

    echo "dl from $URL ... ";
    rm -rf tmp.html;
    curl -s -A $UA $URL -o tmp.html;

    while [ "`get_tag_value tmp.html TITLE | grep [301\|302]`" ];
    do
	echo "error code 301|302. retry";
	sleep 1;
	URL=`get_tag_value tmp.html A HREF`;
	URL=`echo $URL | sed "s/\&amp\;/\&/g"`;
    ## echo $URL;
	if [ "$URL" ];
	then
	    rm -rf tmp.html;
	    eval "curl -s -A $UA $URL -o tmp.html";
	    sleep 1;
	else
	    echo "abort, no candidates";
	    break;
	fi
    done

    mkdir img;
    mkdir img/$QUERY;
    get_tag_value tmp.html img src | grep "gstatic.com" | while read url;
    do
	eval "wget $url -P img/$QUERY";
    done
}

function gokiburi_get(){
    MAX=1000;
    STEP=20;
    while [ "$MAX" -gt 0 ];
    do
	MAX=`expr $MAX - $STEP`;
	echo "dl_images ゴキブリ $MAX;"
	dl_images ゴキブリ $MAX;
    done
}
