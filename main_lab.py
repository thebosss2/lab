import cv2
import pickle
import subprocess
import time

'''
p = subprocess.Popen(['a.out'], shell=True, stdout=PIPE, stdin=PIPE)
for ii in range(10):
    value = str(ii) + '\n'
    #value = bytes(value, 'UTF-8')  # Needed in Python 3.
    p.stdin.write(value)
    p.stdin.flush()
    result = p.stdout.readline().strip()
    print(result)
'''

frames_per_sec=15
num_of_seconds=10
text_file_name= "rgb.txt"

open(text_file_name, "w+").close() #clears the text file
for i in range(num_of_seconds*frames_per_sec):
    
    #captures frame
    cap=cv2.VideoCapture(0)
    ret,frame=cap.read() 
    retval=cv2.imwrite('frames/'+str(i)+'.png',frame)
    
    #writes to the text file
    list_of_frames = open(text_file_name, "a")
    list_of_frames.write("frame: "+ str(i)+ ", time: " + str(time.time())+"\n")
    list_of_frames.close()
    
    #waits for next frame
    time.sleep(1.0/frames_per_sec)
    
    
    
    '''
    #subprocess
    args = ("bin/bar", "-c", "somefile.xml", "-d", "text.txt", "-r", "aString", "-f", "anotherString")
    #Or just:
    #args = "bin/bar -c somefile.xml -d text.txt -r aString -f anotherString".split()
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output
'''
    # code for adding to rgb.txt

cap.release()
cv2.destroyAllWindows()