
##

FLAG=false ;
INC="includes.txt" ;

echo "" > $INC ;

for hoge in `echo "" > /tmp/test.cpp ; LANG=C g++ -v /tmp/test.cpp 2> /tmp/makelog ; cat /tmp/makelog` ;
do
    if [ `echo "${hoge}" | grep -e "End"` ] ;
    then FLAG=false ;
    fi ;
    ##
    if $FLAG ;
    then if [ -d $hoge ] ;
	then
	    echo $hoge ;
	    echo "include_directories(\"$hoge\")" >> $INC ;
	    echo ; >> $INC ;
	fi ;
    fi ;
    ##
    if [ `echo "${hoge}" | grep "here\:"` ] ;
    then FLAG=true ;
    fi ;
done

cmake -G"Eclipse CDT4 - Unix Makefiles" -DECLIPSE_CDT4_GENERATE_SOURCE_PROJECT=TRUE .