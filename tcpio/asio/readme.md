sudo service apport stop
ulimit -c unlimited
gdb app core
bt