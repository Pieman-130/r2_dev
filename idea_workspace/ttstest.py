import pyttsx3
import time
engine = pyttsx3.init()


#All the different voices
voices = engine.getProperty('voices')
for i in range(len(voices)):
	print(i, voices[i].id, voices[i].name)
print(len(voices))
engine.setProperty('rate', 110)
engine.setProperty('volume', 1)
'''
for i in range(1000):
	try:
		engine.setProperty('voice', voices[i].id)
		print(f"Voice Number {i}")
		engine.say("Hello, my name is R Too.  I am an Emotional Suupport Robot.")
		engine.runAndWait()
	except:
		print("Last Voice")
'''
#engine.setProperty('rate', 130)
engine.setProperty('voice',voices[23].id)

#for n in range(70,176):
	#print(n)

#while True:
#text = input("What to say? ")
text = "Hey Johnny, how are you today?  How is Charlie? "
engine.say(text)
engine.runAndWait()
print(dir(engine))
#text = "I intend to talk for a while so I need to speak at the right pace.  Don't you think?"
#engine.say(text)
#engine.runAndWait()
#engine.setProperty('rate', 300)  # Set to a slower rate
#rate = engine.getProperty('rate')
#print(f"Current speech rate: {rate}")

#engine.setProperty('rate', 1)
#rate = engine.getProperty('rate')
#print(f"Current speech rate: {rate}")
start = time.time()
print( time.time()-start, " seconds.")

