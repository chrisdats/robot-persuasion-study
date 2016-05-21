import time
import logging
import threading
import Queue
from threading import Thread

start_time = time.time()

filename = "PB/PBAoverrides.txt"

with open(filename, "r") as file:
	q = Queue.Queue(100)
	for line in file:
		q.put(line)

while q.empty() == False:
	line = q.get()
	words = line.split(",")
	while (time.time() - start_time) < float(words[1]) :
		pass # do nothing
	# when we reach the correct time
	print words[0] #send command

print "finished"
