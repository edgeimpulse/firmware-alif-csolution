#!/bin/bash
set -e
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

TARGET=$1

if [ -z "$TARGET" ]; then
    TARGET="HP"
fi

echo "make: using ${MAKE_JOB} jobs"

if [ "$TARGET" == "HE" ] || [ "$TARGET" == "HP" ] || [ "$TARGET" == "HP_SRAM" ]; then
    echo "Building firmware for ${TARGET}"
    cbuild ./firmware-alif.csolution.yaml -j ${MAKE_JOBS:-$(nproc)} --context-set --update-rte --packs --context firmware-alif.debug+${TARGET}
elif [ "$TARGET" == "clean" ]; then
    echo "Cleaning"
    cbuild ./firmware-alif.csolution.yaml -j ${MAKE_JOBS:-$(nproc)} --context-set --update-rte --packs --context firmware-alif -C
else
    echo "Invalid target: $TARGET"
    exit 1
fi
