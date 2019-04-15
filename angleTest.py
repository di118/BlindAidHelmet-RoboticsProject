import cv2
x = 236
y = 180
image3 =  cv2.imread('images/image1.jpg')
height, width, channels = image3.shape
print(width,height)