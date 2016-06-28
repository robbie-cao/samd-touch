#!/bin/bash

#******************************************************************************
#
# Flash script for SAMD with edbg
#
#*****************************************************************************

function usage()
{
    echo "Usage:"
    echo "  $0 [path] [operation] [target]"
    echo "  - path      - relative path to xxx.bin"
    echo "  - operation - operation to do, E - erase, P - program(write), V - verify"
    echo "  - target    - target options, eg atmel_cm0p, atmel_cm3, atmel_cm4 (currently support cm0p ONLY)"
    echo "Example:"
    echo "  $0 Apps/hello EP"
    echo "  $0 Apps/hello"
    echo "  $0 . E"
    echo "  $0"
    echo "  $0 . E --debug"
}

HOST_OS=$(uname -s)

#
# ROOT_PATH comes from envsetup.sh
# Make sure run `source envsetup.sh` first of all
#
if [ "x$ROOT_PATH" = "x" ]; then
    ROOT_PATH=$(pwd)
fi

ATFLASH=$ROOT_PATH/tools/edbg-mac

if [ "$(uname)" == "Darwin" ]; then
    # Mac OS X
    ATFLASH=$ROOT_PATH/tools/edbg-mac
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    # Linux
    ATFLASH=$ROOT_PATH/tools/edbg-linux
elif [ "$(expr substr $(uname -s) 1 6)" == "CYGWIN" ]; then
    # Cygwin
    ATFLASH=$ROOT_PATH/tools/edbg-cygwin.exe
else
    echo "Platform not support!"
    exit 1
fi

if [ "$1" = "h" -o "$1" = "-h" -o "$1" = "help" ]; then
    usage
    exit 0
fi

if [ $# -gt 4 ]; then
    echo "Too many parameters!"
    usage
    exit 1
fi

DIR=$1
ACT=$2
TGT=$3

if [ "x$DIR" = "x" ]; then
    DIR=.
fi

if [ ! -d $DIR ]; then
    echo "Path not exist!"
    usage
    exit 1
fi

if [ $(echo `ls $DIR/*.bin` | wc -w) -lt 1 ]; then
    echo "Not foud binary image under $DIR!"
    ls $DIR/*.bin
    exit 1
fi

if [ $(echo `ls $DIR/*.bin` | wc -w) -gt 1 ]; then
    echo "Too many binary images under $DIR!"
    ls $DIR/*.bin
    exit 1
fi

OPTION="-bepv"
TARGET="atmel_cm0p"

IMAGE=$(ls $DIR/*.bin)

if [ "x$ACT" = "x" ]; then
    OPTION="-bepv"
fi

if [ "x$ACT" = "xE" ]; then
    OPTION="-be"
fi

if [ "x$ACT" = "xEP" ]; then
    OPTION="-bep"
fi

if [ "x$ACT" = "xPV" ]; then
    OPTION="-bpv"
fi

if [ "x$ACT" = "xEPV" ]; then
    OPTION="-bepv"
fi

CMD="$ATFLASH $OPTION -t $TARGET -f $IMAGE"
echo $CMD
echo "Action: $ACT (E = erase, P = program, V = verify)"
echo "Image : $IMAGE"

$CMD

echo "Done!"

echo $CMD
