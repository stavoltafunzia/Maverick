#!/bin/sh
PROBLEMNAME="SingleMassPointE"
OUTPUT="SingleMassPointE"
CCEXT=".cc"
MAIN="_main.cc"
LIB="_lib.cc"

# compile flags
OPTLEVEL="-O3"

#script vars
SHARED=true
THISPLATFORM=true
DEBUG=false
SEP=.

# Parse the input
for i in "$@"
do
case $i in
    -e|--exec)
    SHARED=false
    ;;
    -n|--noplatform)
    THISPLATFORM=false
    ;;
    -d|--debug)
    DEBUG=true
    ;;
#    -l=*|--lib=*)
#    DIR=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
#    ;;
    --default)
#    DEFAULT=YES
    ;;
    *)
            # unknown option
    ;;
esac
done

# change some variables depending on the platform
if [ "$(uname)" = "Darwin" ]; then
    SLEXT=".dylib"
    SHAREDFLAG="-dynamiclib -undefined dynamic_lookup"
    CC11="-std=c++11 -stdlib=libc++"
    ADDSLFLAGS=""
    PLATFORM=macosx
else
    SLEXT=".so"
    SHAREDFLAG="-shared"
    CC11="-std=c++11"
    ADDSLFLAGS=-Wl,-soname,$(pwd)/lib$PROBLEMNAME$SLEXT
    PLATFORM=unix
fi

if [ "$THISPLATFORM" = true ]; then
   OUTPUT=$OUTPUT$SEP$PLATFORM
fi

if [ "$DEBUG" = true ]; then
    MAVPATH=/usr/local/maverick_debug
else
    MAVPATH=/usr/local/maverick
fi
  
#build
if [ "$SHARED" = true ]; then
    g++ $OPTLEVEL $CC11 -O3 -march=native -m64 $SHAREDFLAG -fPIC $PROBLEMNAME$CCEXT $PROBLEMNAME$LIB -o lib$PROBLEMNAME$SLEXT -I$MAVPATH/include -L$MAVPATH/lib -lmaverick $ADDSLFLAGS
    g++ $OPTLEVEL $CC11 -O3 -march=native -m64 $PROBLEMNAME$MAIN -o $OUTPUT -I$MAVPATH/include -L$(pwd) -l$PROBLEMNAME -L$MAVPATH/lib -lmaverick -lmavericktoip -L/usr/local/ipopt/lib -lipopt -ldl
else
    #make the exec
    g++ $OPTLEVEL $CC11 -O3 -march=native -m64 $PROBLEMNAME$MAIN $PROBLEMNAME$CCEXT -o $OUTPUT -I$MAVPATH/include -L$MAVPATH/lib -lmaverick -lmavericktoip -L/usr/local/ipopt/lib -lipopt -ldl
    #make the library anyway
    g++ $OPTLEVEL $CC11 -O3 -march=native -m64 $SHAREDFLAG -fPIC $PROBLEMNAME$CCEXT $PROBLEMNAME$LIB -o lib$PROBLEMNAME$SLEXT -I$MAVPATH/include -L$MAVPATH/lib -lmaverick $ADDSLFLAGS
fi
