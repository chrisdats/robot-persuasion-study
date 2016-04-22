import time
from naoqi import ALProxy

#Get the Nao's IP
ipAdd = None
try:
    ipFile = open("ip.txt")
    ipAdd = ipFile.readline().replace("\n","").replace("\r","")
except Exception as e:
    print "Could not open file ip.txt"
    ipAdd = raw_input("Please write Nao's IP address... ") 


#Create Speech Recognition Proxy
try:
    asr = ALProxy("ALSpeechRecognition", ipAdd, 9559)
except Exception, e:
    print "Error when creating speech recognition proxy:"+str(e)
    exit(1)

# Create memoryProxy
try:
    memoryProxy = ALProxy("ALMemory", ipAdd, 9559)
except Exception, e:
    print "Error when creating memory proxy:"+str(e)
    exit(1)

asr.setLanguage("English")
vocabulary = ["yes", "no", "please"]
asr.setVocabulary(vocabulary, False)

# Start the speech recognition engine with user Test_ASR
asr.subscribe("Test_ASR")
print 'Speech recognition engine started'
time.sleep(10)
asr.unsubscribe("Test_ASR")
memoryProxy.getData("LastWordRecognized")