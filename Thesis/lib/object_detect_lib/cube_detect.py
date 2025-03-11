import cv2
import numpy as np

class detect_edges_pic():

    # Load the image
    path_to_image = "E://NVIDIA//isaacsim//myscripts//Thesis//lib//object_detect_lib//resources//overheadangled.png"
    path_to_image_2 = "E://NVIDIA//isaacsim//myscripts//Thesis//lib//object_detect_lib//resources//topdown.png"
    img = cv2.imread(path_to_image)

    # Convert to grayscale and blur
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Perform edge detection
    edges = cv2.Canny(blur, 50, 200)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours")
    
    cv2.imshow("Contours", edges)
    cv2.waitKey(0)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

    for cnt in contours:
        # Approximate the contour to a polygon
        epsilon = 0.0001 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Print the number of vertices of the approximated polygon
        print(f"Number of vertices: {len(approx)}")

        # Draw the approximated contour on the image
        cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)

    # Display the image with the approximated contours
    cv2.imshow("Contours", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()