from noPlateRecognition import ANPR
from PIL import Image
import cv2 
from skimage import io
import imutils

image = cv2.imread("001.jpg")
image = imutils.resize(image, width=600)
cv2.imshow("image", image)

# create object to automatically recognize number plates
anpr = ANPR(debug=False)
# apply automatic license plate recognition
(lpText, lpCnt) = anpr.find_and_ocr(image, psm=7,clearBorder=True)
print(lpText)
if lpText is not None and lpCnt is not None:
		# fit a rotated bounding box to the license plate contour and
		# draw the bounding box on the license plate
		box = cv2.boxPoints(cv2.minAreaRect(lpCnt))
		box = box.astype("int")
		cv2.drawContours(image, [box], -1, (0, 255, 0), 2)

		# compute a normal (unrotated) bounding box for the license
		# plate and then draw the OCR'd license plate text on the
		# image
		(x, y, w, h) = cv2.boundingRect(lpCnt)
		cv2.putText(image, anpr.cleanup_text(lpText), (x, y - 15),
			cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

		# show the output ANPR image
		#print("[INFO] {}".format(lpText))
		cv2.imshow("Output ANPR", image)
		cv2.waitKey(0)
