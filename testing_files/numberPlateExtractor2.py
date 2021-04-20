import os
import sys
import requests
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
from io import BytesIO

class ANPR:
        def numberV(self):
                subscription_key = "0f5bb668eab84237bd5742644f3cae02"
                endpoint = "https://ocr-test-for-python.cognitiveservices.azure.com/"
                image_path = "/home/pi/Desktop/VehicleDetectionProject/001.jpg"

                ocr_url = endpoint + "vision/v3.1/ocr"

                # Set image_url to the URL of an image that you want to analyze.
                # image_url = "http://cdni.autocarindia.com/ExtraImages/20180402113123_NumberPlate_Swift.jpg"

                # headers = {'Ocp-Apim-Subscription-Key': subscription_key}
                params = {'language': 'unk', 'detectOrientation': 'true'}
                # data = {'url': image_url}

                # Read the image into a byte array
                image_data = open(image_path, "rb").read()
                # Set Content-Type to octet-stream
                headers = {'Ocp-Apim-Subscription-Key': subscription_key, 'Content-Type': 'application/octet-stream'}
                # put the byte array into your post request
                response = requests.post(ocr_url, headers=headers, params=params, data = image_data)

                response.raise_for_status()

                analysis = response.json()
                return analysis
            
if "__name__" == "__main__":
    anrp = ANRP()
    r = anrp.numberV()
    print(r)