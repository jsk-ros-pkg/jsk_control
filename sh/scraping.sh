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
    OUTPUT=$3;
    if [ ! "$QUERY" ];
    then
	echo no query detected;
	return 0;
    fi
    if [ ! "$START" ];
    then
	START=0;
    fi
    if [ ! "$OUTPUT" ];
    then
        OUTPUT=img/$QUERY;
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

    if [ ! -e "$OUTPUT" ];
    then
        mkdir -p "$OUTPUT";
    fi
    ## WC=`ls img/$QUERY | wc -l`;
    get_tag_value tmp.html img src | grep "gstatic.com" | while read url;
    do
        echo "$url -> `basename $url`";
	if [ ! -e "\"$OUTPUT/`basename $url`" ];
	then
	    echo "wget $url -O \"$OUTPUT/`basename $url`";
	    eval "wget $url -O \"$OUTPUT/`basename $url`";
	    # _WC=`ls img/$QUERY | wc -l`;
	    # if [ "$WC" -eq "$_WC" ];
	    # then
	    # 	echo dl fialed, abort;
	    # 	return -1;
	    # fi
	    # WC=$_WC;
	else
	    echo deprecated $url skipped;
	fi
    done
    return 0;
}

function dl_images_loop (){
    MAX=$2;
    STEP=20;
    while [ "$MAX" -gt 0 ];
    do
	MAX=`expr $MAX - $STEP`;
	echo "dl_images $1 $MAX $3;"
	if [ "`dl_images $1 $MAX $3;`" -lt 0 ];
	then
	    return -1;
	fi
    done
    return 0;
}

function gokiburi_get(){
    dl_images_loop "blattella" 500 "img/cockroach";
    dl_images_loop "periplaneta" 500 "img/cockroach";
    dl_images_loop "spider" 500 "img/spider";
}
function sonota_get(){
    dl_images_loop "wall" 100 "img/else";
    dl_images_loop "grass" 100 "img/else";
    dl_images_loop "human" 200 "img/else";
    dl_images_loop "cat" 200 "img/else";
    dl_images_loop "horse" 200 "img/else";
}

function convert_to_jpg (){
    TAG=$1;
    ORG="img/$TAG";
    OUT="jpg/$TAG";
    ID=0;
    mkdir -p $OUT;
    for p in `ls $ORG`;
    do
	convert -resize 128x128! $ORG/$p $OUT/$ID.jpg;
	ID=`expr $ID + 1`;
    done
}
