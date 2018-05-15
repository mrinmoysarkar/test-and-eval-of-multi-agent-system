import numpy as np
import cv2




if __name__=='__main__':
    cap = cv2.VideoCapture('rtsp://127.0.0.1:8554/bottom')
    flag = False
    if cap.isOpened():
        print('open')
        flag=True
    else:
        print('close')
        #cap.open()

    while(flag):
        # Capture frame-by-frame
        ret, frame = cap.read()
        #print(ret)
        # Our operations on the frame come here
        
        # Display the resulting frame
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
