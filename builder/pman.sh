#/bin/bash
##Project Manager##
TOP_DIR=$(cd $(dirname $0)/.. ; pwd)
SRC_DIR=${TOP_DIR}/src
BUILDER_DIR=${TOP_DIR}/builder
BUILD_DIR=${TOP_DIR}/build
TARGET_DIR=

##create init function
env_init()
{
	echo "${FUNCNAME[0]}"

#	SRC_BINUTILS_DIR=$( ls $SRC_DIR | grep "binutils")
#	SRC_GCC_DIR=$( ls $SRC_DIR | grep "linux")
#	SRC_KERNEL_DIR=$( ls $SRC_DIR | grep "binutils")
#	SRC_GLIBC_DIR=$( ls $SRC_DIR | grep "binutils")

}

binutils_config()
{
	echo "${FUNCNAME[0]}"
	SRC_BINUTILS_DIR=$( ls $SRC_DIR | grep "binutils")
	BUILD_BINUTILS_DIR=${BUILD_DIR}/binutils
	#echo "$SRC_BINUTILS_DIR"
	#echo $BUILD_BINUTILS_DIR
	cd $BUILD_BINUTILS_DIR
	$SRC_BINUTILS_DIR/configure --help
}

gcc_config()
{
	echo "${FUNCNAME[0]}"
}

kernel_config()
{
	echo "${FUNCNAME[0]}"
}

glibc_config()
{
	echo "${FUNCNAME[0]}"
}

##create build function
##crete clean function

test_function()
{
	echo "${FUNCNAME[0]}"
	echo "$TOP_DIR"
	echo "$SRC_DIR"
	echo "$BUILDER_DIR"
	return 0
}

if [ $# -eq 0 ] 
then 
	echo "Usage : $1 <function [parameters ... ] >"
	echo "Function list :"
	declare -F | sed -rn 's/declare -f (\w*)/\1/p'

else
	$@
	if [ $? -eq 0 ]
	then echo "Function Worked!"
	else echo "Function Failed!"
	fi
fi
