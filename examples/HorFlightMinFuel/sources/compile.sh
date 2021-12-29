#!/bin/sh
PROBLEMNAME="HorFlightMinFuel"
OUTPUT="HorFlightMinFuel"
CCEXT=".cc"
MAIN="_main.cc"
LIB="_lib.cc"

# compile flags
OPTLEVEL="-O3"

#script vars
SHARED=true
DEBUG=false
SEP=.

# Parse the input
for i in "$@"
do
case $i in
    -e|--exec)
    SHARED=false
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
    ADDSLFLAGS=""
    PLATFORM=macosx
else
    SLEXT=".so"
    SHAREDFLAG="-shared"
    ADDSLFLAGS=-Wl,-soname,$(pwd)/lib$PROBLEMNAME$SLEXT
    PLATFORM=unix
fi

MAVPATH=/usr/local/maverick

#build
if [ "$SHARED" = true ]; then
    g++ $OPTLEVEL -O3 -march=native -m64 -Wall $SHAREDFLAG -fPIC $PROBLEMNAME$CCEXT $PROBLEMNAME$LIB -o lib$PROBLEMNAME$SLEXT -I$MAVPATH/include -L$MAVPATH/lib -lmaverick $ADDSLFLAGS
    g++ $OPTLEVEL -O3 -march=native -m64 -Wall $PROBLEMNAME$MAIN -o $OUTPUT -I$MAVPATH/include -L$(pwd) -l$PROBLEMNAME -L$MAVPATH/lib -lmaverick -lmavericktoip -L/usr/local/ipopt/lib -lipopt -ldl
else
    #make the exec
    g++ $OPTLEVEL -O3 -march=native -m64 -Wall $PROBLEMNAME$MAIN $PROBLEMNAME$CCEXT -o $OUTPUT -I$MAVPATH/include -L$MAVPATH/lib -lmaverick -lmavericktoip -L/usr/local/ipopt/lib -lipopt -ldl
    #make the library anyway
    g++ $OPTLEVEL -O3 -march=native -m64 -Wall $SHAREDFLAG -fPIC $PROBLEMNAME$CCEXT $PROBLEMNAME$LIB -o lib$PROBLEMNAME$SLEXT -I$MAVPATH/include -L$MAVPATH/lib -lmaverick $ADDSLFLAGS
fi
