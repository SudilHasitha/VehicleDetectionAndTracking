from numberPlateExtractor2 import ANPR
from PIL import Image
import cv2 
from skimage import io
import imutils

image = cv2.imread("001.jpg")
#cv2.imshow("image", image)
# cv2.waitKey(0)
# create object to automatically recognize number plates
anpr = ANPR()
res = anpr.numberV(image)
print(res)