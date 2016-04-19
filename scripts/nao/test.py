import time
import logging

# create a logger to record all the robot commands that occured
# for this particular participant
FORMAT = '%(asctime)-15s [%(levelname)s] (%(threadName)-10s) %(message)s'
logging.basicConfig(level=logging.DEBUG,
                    format=FORMAT)   
log_filename = "test.txt"      
file_handler = logging.FileHandler(log_filename)
file_handler.setFormatter(logging.Formatter(FORMAT))
logging.getLogger().addHandler(file_handler)
time.sleep(1)
logging.info("Starting " + str(time.time()))
time.sleep(3)
logging.info("Ending " + str(time.time()))