#!/usr/bin/expect -f
set pass "witwurov"

spawn ssh ubuntu@10.0.10.100
expect "password: "
sleep 1
send "$pass\r"
expect eof