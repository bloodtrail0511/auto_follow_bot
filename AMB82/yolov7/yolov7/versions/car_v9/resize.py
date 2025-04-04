import cv2
import numpy as np
img = cv2.imread('..\car_v9\IMG_7512.JPG')
img_resize = cv2.resize(img, (576,320))

cv2.imwrite('../car_v9/IMG_7512_resize.jpg', img_resize)
cv2.imshow('img', img_resize)
cv2.waitKey(0)
cv2.destroyAllWindows()