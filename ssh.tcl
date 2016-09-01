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

# send "\n"
# spawn bash -c "rsync -avzP --no-perms --no-owner --no-group $local_dir $user@$ip:$remote_dir"
# # authenticate
# interact

# set command [lindex $argv 1]
set ip 192.168.5.11
set user {ubuntu}
ssh $user $ip
# switch $command {
#     "ssh" {
#         ssh $user $ip
#     }
#     "sync" {
#         sync $ip
#     }
#     "delSync" {
#         delSync $ip
#     }
#     default {
#         puts "default"
#         # printUsage $argv0
#         # puts [findFile conf]
#         # puts [readFile [findFile conf]]
        
#     }
# }