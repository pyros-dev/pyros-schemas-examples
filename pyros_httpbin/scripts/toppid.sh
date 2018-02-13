#!/bin/bash

export APP_PID=$1
echo "pid: $APP_PID"
top -b -n 1000 -p $APP_PID | grep $APP_PID &
#konsole -e htop -p $APP_PID
                                   
