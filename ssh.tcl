#!/usr/bin/expect -f
log_user 1
set timeout 10;

proc ssh {username ip} {
    spawn ssh -t $username@$ip "bash -l"
    interact
}

proc sync {ip} {
    set username {administrator}
    set local_dir {src/*}
    set remote_dir {~/barzin_catkin_ws/src/path_tracking/scripts/}
    spawn bash -c "rsync -avzP --no-perms --no-owner --no-group $local_dir $username@$ip:$remote_dir"
    interact
}

proc delSync {ip} {
    set username {administrator}
    set local_dir {src/*}
    set remote_dir {~/barzin_catkin_ws/src/path_tracking/scripts/}
    spawn bash -c "rsync -avzP --no-perms --no-owner --delete --no-group $local_dir $username@$ip:$remote_dir"
    interact
}


set ip 192.168.5.11
set user {ubuntu}
ssh $user $ip
