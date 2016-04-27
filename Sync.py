#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 14.03.2015

@author: micha
'''

import MySQLdb as mdb
import paramiko
import time

# synchronize logger contents
ssh = None
con = None
time.sleep(5)
try:
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.connect('buwx.de', username='user')

    # retrieve last time stamp
    [stdin, stdout, stderr] = ssh.exec_command('echo "select max(dateTime) from sensor" | mysql -N --user=weewx --password=weewx weewxdb')
    last_time = stdout.readline().strip()
    last_time = 0 if last_time == 'NULL' else int(last_time)

    # insert data
    con = mdb.connect('localhost', 'davis', 'davis', 'davis');
    cur = con.cursor()
    cur.execute("SELECT dateTime,data FROM sensor WHERE dateTime>%d ORDER BY dateTime ASC" % (last_time))

    [stdin, stdout, stderr] = ssh.exec_command('mysql -N --user=weewx --password=weewx weewx')
    for (date_time, data) in cur:
        print >> stdin, "INSERT INTO sensor(dateTime,data) VALUES(%d,'%s');" % (date_time, data)

    stdin.close()

except Exception, e:
    print "Error: " + str(e)

finally:
    if ssh:
        ssh.close()
    if con:
        con.close()
