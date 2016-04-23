import time
import logging
import threading
from threading import Thread

# create a logger to record all the robot commands that occured
# for this particular participant
FORMAT = '%(asctime)-15s [%(levelname)s] (%(threadName)-10s) %(message)s'
log_filename1 =  "loggerTest1.txt"  
logger1 = logging.getLogger('logger1')
logging.basicConfig(level=logging.DEBUG, format=FORMAT)        
file_handler = logging.FileHandler(log_filename1)
file_handler.setFormatter(logging.Formatter(FORMAT))
logger1.addHandler(file_handler)

log_filename2 =  "loggerTest2.txt"  
logger2 = logging.getLogger('logger2')
logging.basicConfig(level=logging.DEBUG, format=FORMAT)        
file_handler = logging.FileHandler(log_filename2)
file_handler.setFormatter(logging.Formatter(FORMAT))
logger2.addHandler(file_handler)

logger1.info("test1")
logger2.info("test2")


class Demo:
	def __init__(self):
		self.ignore= 0
	def run(self):
		script_filename = "delete"
		start_time = time.time()
		script_filename= "junk.txt"
		t1 = Thread(target=self.readScript, args=(script_filename, start_time, ))
		t2 = Thread(target=self.monitorParticipant, args=(130, start_time, ))
		t1.start()
		t2.start()
		t1.join()
		t2.join()
	def readScript(self, script_filename, start_time):
		logger1 = logging.getLogger("logger1")  # return reference to logger object
		logger1.info("hi")
		time.sleep(1)
	def monitorParticipant(self, time_limit, start_time):
		logger2 = logging.getLogger("logger2")
		logger2.info("hello")
		time.sleep(1)

d = Demo()
d.run()


#   def setup_logger(logger_name, log_file, level=logging.INFO):
#        l = logging.getLogger(logger_name)
#        formatter = logging.Formatter('%(asctime)s : %(message)s')
#        fileHandler = logging.FileHandler(log_file, mode='w')
#        fileHandler.setFormatter(formatter)
#        l.setLevel(level)
#        l.addHandler(fileHandler)


#    setup_logger('log1', r'C:\temp\log1.log')
#    setup_logger('log2', r'C:\temp\log2.log')
#    log1 = logging.getLogger('log1')
#    log2 = logging.getLogger('log2')

#    log1.info('Info for log 1!')
#    log2.info('Info for log 2!')
#    log1.error('Oh, no! Something went wrong!')
