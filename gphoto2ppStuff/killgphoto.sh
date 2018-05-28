ps -ef | grep 'gvfsd-gphoto2' | grep -v grep | awk '{print $2}' | xargs -r kill -9
