#/bin/bash
##Project Manager##
TOP_DIR=$(cd $(dirname $0)/.. ; pwd)
SRC_DIR=${TOP_DIR}/src
BUILDER_DIR=${TOP_DIR}/builder

##create init function
env_init()
{

}

binutils_config()
{

}

gcc_config()
{

}

kernel_config(){}

glicb_config(){}

##create build function
##crete clean function

test_function()
{
	echo "${FUNCNAME[0]}"
	echo "$TOP_DIR"
	echo "$SRC_DIR"
	echo "$BUILDER_DIR"
	return 1
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
