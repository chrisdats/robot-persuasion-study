import time
import logging
import threading
import Queue
from threading import Thread

filename = "/PB/PBAoverrides.txt"

with open(filename, "r") as file:
	Queue.Queue(100)
	for line in file:
		Queue.put(line)
