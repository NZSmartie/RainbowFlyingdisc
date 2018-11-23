## Actvate the virtual environment

_PWD=$(dirname $(realpath $BASH_SOURCE))

if [ -z ${VIRTUAL_ENV+x} ]
then
        ACTIVATE_SCRIPT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/env/bin/activate"
        if [ -f "$ACTIVATE_SCRIPT" ]
        then
                source $ACTIVATE_SCRIPT

                pip install -q -r zephyr/scripts/requirements.txt
        else
                echo "Setting up vrtual environment."
                virtualenv -p $(which python2) env
                source $ACTIVATE_SCRIPT

                echo "Installing required packages inside virtual environment."

                pip install -r zephyr/scripts/requirements.txt
        fi
fi

source $_PWD/zephyr/zephyr-env.sh
