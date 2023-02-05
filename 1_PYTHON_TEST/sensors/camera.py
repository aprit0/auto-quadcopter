# import the opencv library
import cv2 as cv


# define a video capture object
vid = cv.VideoCapture(0)
vid.set(cv.CAP_PROP_BUFFERSIZE, 2)
vid.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))

while(True):
	
	# Capture the video frame
	# by frame
	ret, frame = vid.read()
	img = frame[55:, :]
	scale_percent = 50 # percent of original size
	width = int(img.shape[1] * scale_percent / 100)
	height = int(img.shape[0] * scale_percent / 100)
	new_frame = cv.resize(img, (width, height), interpolation=cv.INTER_AREA)
	# Display the resulting frame
	cv.imshow('frame', new_frame)
	
	# the 'q' button is set as the
	# quitting button you may use any
	# desired button of your choice
	if cv.waitKey(1) & 0xFF == ord('q'):
		break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()

