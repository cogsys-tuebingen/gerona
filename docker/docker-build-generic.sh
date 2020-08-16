#!/bin/sh -e

SCRIPT_PATH=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT_PATH)

TYPE=$(basename $0 .sh)
DOCKERFILE=${SCRIPT_DIR}/${TYPE}.docker

set -x
cd $SCRIPT_DIR/../..
pwd
docker build -f ${DOCKERFILE} -t gerona-${TYPE} --network=host .