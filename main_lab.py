import cv2;
import pickle;
import subprocess
import time



cap=cv2.VideoCapture(0)
ret,frame=cap.read()
retval=cv2.imwrite('frame1.png',frame)

frames_per_sec=15
num_of_seconds=60
fo = open("rgb.txt", "w+")
for i in range(num_of_seconds*frames_per_sec):
    
    
    cap=cv2.VideoCapture(0)
    ret,frame=cap.read()
    retval=cv2.imwrite('frames/frame '+str(i)+'.png',frame)
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