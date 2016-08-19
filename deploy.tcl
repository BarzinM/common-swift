#!/usr/bin/expect -f
log_user 1

set timeout 10;
set user {ubuntu}
set local_dir {~/Barzin/common-swift/src/*}
set remote_dir {~/hexa_ws/src/}
set ip {192.168.5.11}

send "\n"
spawn bash -c "rsync -avzP --no-perms --no-owner --no-group $local_dir $user@$ip:$remote_dir"
interact