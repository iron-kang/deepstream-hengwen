#!/bin/bash
SERVICE="image_analysis"

function remove_expired_files () {
    expired_file=`find /home/nvidia/ftp/ -mtime +0 | wc -l`
    if [ $expired_file -gt 0 ]; then
        echo "delete 1 day ago files"
        echo "remove fise $expired_file"
        find /home/nvidia/ftp/ -mtime +0 -delete
    fi
}

remove_expired_files

while true
do
    #date_check=`date --date='1 day ago' +%m%d`
    #rm /home/nvidia/ftp/*$date_check*
    date_check=`date +%H%M`
    #if [ $date_check -eq '00' ]; then
    #    remove_expired_files
    #fi

    if pgrep -x "$SERVICE" >/dev/null
    then
        echo "$SERVICE is running" >/dev/null
    else
        echo "$SERVICE stopped"
        cd /home/nvidia/hengwen/
        ./$SERVICE pgie_config.txt &
    fi
    sleep 1
done

