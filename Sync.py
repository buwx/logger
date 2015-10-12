#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 14.03.2015

@author: micha
'''

import MySQLdb as mdb
import paramiko

# synchronize logger contents
ssh = None
con = None
try:
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.connect('buwx.de', username='micha')

    # retrieve last id
    [stdin, stdout, stderr] = ssh.exec_command('echo "select max(id) from logger" | mysql -N --user=weewx --password=weewx weewx')
    last_id = stdout.readline().strip()
    last_id = 0 if last_id == 'NULL' else int(last_id)

    # insert data
    con = mdb.connect('localhost', 'davis', 'davis', 'davis');
    cur = con.cursor()
    cur.execute("SELECT id,ts,sensor,data FROM logger WHERE id>%d ORDER BY id ASC" % (last_id))

    [stdin, stdout, stderr] = ssh.exec_command('mysql -N --user=weewx --password=weewx weewx')
    for (the_id, ts, sensor, data) in cur:
        print >> stdin, "INSERT INTO logger(id,ts,sensor,data) VALUES(%d,'%s','%s','%s');" % (the_id, ts, sensor, data)

    stdin.close()

except Exception, e:
    print "Error: " + str(e)

finally:
    if ssh:
        ssh.close()
    if con:
        con.close()
