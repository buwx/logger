#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 12.03.2015

@author: micha
'''

import MySQLdb as mdb

from util import formatData, description, sensor 

# the main procedure

con = None

try:

    con = mdb.connect('localhost', 'davis', 'davis', 'davis');
    cur = con.cursor()

    cur.execute("SELECT id, data FROM logger WHERE id>=18020 AND id<=64164")
    for row in cur:
        identifier = row[0]
        data = row[1]
        print "UPDATE logger set sensor='" + sensor(data) + "', data='" + formatData(data) + "', description='" + description(data) + "' WHERE id=" + str(identifier) + ";"
    # cur.execute("INSERT INTO logger(sensor,data,description) VALUES(%s,%s,%s)", (sensor(line),formatData(line),description(line)))
    # con.commit()

except Exception:
    print "Error!"

finally:
    if con:
        con.close()
