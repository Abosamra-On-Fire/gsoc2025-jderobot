import GUI
import cv2
import numpy as np

# Initialize variables for optical flow (needs previous frame)
prev_gray = None
prev_corners = None
color_filter_lower = np.array([0, 0, 0])  # Default values (will be updated)
color_filter_upper = np.array([255, 255, 255])  # Default values

while True:
    image = GUI.getImage()
    if image is not None:


        # Grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        

        # opening
        kernel = np.ones((5,5), np.uint8)
        morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
        

        # red filter
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        
        color_mask = mask1 + mask2
        color_filtered = cv2.bitwise_and(image, image, mask=color_mask)
        

        # edge detection (Canny)
        edges = cv2.Canny(gray, 100, 200)
        

        # Convolution with custom kernel (sharpening)
        kernel = np.array([[0, -1, 0],
                          [-1, 5, -1],
                          [0, -1, 0]])
        sharpened = cv2.filter2D(image, -1, kernel)
        

        # Optical Flow (Lucas-Kanade method)
        flow_display = image.copy()

        feature_params = dict(maxCorners=200,
                            qualityLevel=0.01,
                            minDistance=7,
                            blockSize=7)

        lk_params = dict(winSize=(15, 15),
                        maxLevel=2,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        if prev_gray is not None:
            new_corners, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_corners, None, **lk_params)
            
            if new_corners is not None:
                good_new = new_corners[status==1]
                good_old = prev_corners[status==1]
                
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    flow_display = cv2.line(flow_display, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                    # flow_display = cv2.circle(flow_display, (int(a), int(b)), 5, (0, 0, 255), -1)
            
            if len(good_new) < 25:
                prev_corners = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
                if prev_corners is not None:
                    prev_corners = prev_corners.reshape(-1, 1, 2)
            else:
                prev_corners = good_new.reshape(-1, 1, 2)
        else:
            prev_corners = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
            if prev_corners is not None:
                prev_corners = prev_corners.reshape(-1, 1, 2)

        prev_gray = gray.copy()
        

        # Corner Detection (Harris)
        corner_display = image.copy()
        dst = cv2.cornerHarris(gray, 2, 3, 0.04)
        dst = cv2.dilate(dst, None)
        corner_display[dst > 0.01 * dst.max()] = [0, 0, 255]
        

        # Hough Transform (circle detection)
        hough_display = image.copy()

        gray_circle = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image.copy()

        gray_blurred = cv2.medianBlur(gray_circle, 5)

        circles = cv2.HoughCircles(gray_blurred, 
                                cv2.HOUGH_GRADIENT, 
                                dp=1, 
                                minDist=20,
                                param1=100,
                                param2=50,
                                minRadius=10,
                                maxRadius=40)

        if circles is not None:

            circles = np.uint16(np.around(circles))
            
            for circle in circles[0, :]:
                cv2.circle(hough_display,(circle[0], circle[1]), circle[2],(0, 255, 0), 2)
                
                cv2.circle(hough_display,(circle[0], circle[1]),2,(0, 0, 255),3)

        #---------------------------------------------------------------------

        # Grayscale
        # GUI.showImage(gray)

        # opening
        # GUI.showImage(morph)

        # red filter
        # GUI.showImage(color_filtered)

        # edge detection (Canny)
        # GUI.showImage(edges)

        # (sharpening)
        # GUI.showImage(sharpened)

        # Optical Flow
        # GUI.showImage(flow_display)

        # Corner Detection
        # GUI.showImage(corner_display)

        # Hough Transform (circle detection)
        GUI.showImage(hough_display)