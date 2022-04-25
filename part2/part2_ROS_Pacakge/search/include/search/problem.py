
import cv2 as cv
import numpy as np

img = np.ones((1000,1000,3), np.uint8)
#Clearance
c=15
rec1 = np.array( [[[25-c,425-c],[175+c,425-c],[175+c,575+c],[25-c,575+c]]], dtype=np.int32 )
rec2= np.array( [[[375-c,425-c],[625+c,425-c],[625+c,575+c],[375-c,575+c]]], dtype=np.int32 )
rec3=np.array( [[[725-c,200-c],[875+c,200-c],[875+c,400+c],[725-c,400+c]]], dtype=np.int32 )


colors = (0,0,255)
cv.fillPoly(img, rec1, colors )
cv.fillPoly(img, rec2, colors )
cv.fillPoly(img, rec3, colors)
cv.circle(img,(200,800), 100+c,colors, -1)
cv.circle(img,(200,200), 100+c,colors, -1)
