import cv2
import numpy as np

class detect_edges_pic():

    # Load the image
    img = cv2.imread("testpic3.png")

    # Convert to grayscale and blur
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Perform edge detection
    edges = cv2.Canny(blur, 50, 200)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours")

    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    cv2.imshow("Contours", img)
    cv2.waitKey(0)

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


# for cnt in contours:
#     print(f"Contour length: {len(cnt)}")
#     # Approximate the contour to a polygon
#     epsilon = 0.0005 * cv2.arcLength(cnt, True)  # Decrease the value from 0.02 to 0.01
#     approx = cv2.approxPolyDP(cnt, epsilon, True)

#     # If the polygon has four vertices, it is a rectangle
#     if len(approx) == 4:
#         # Get the coordinates of the corners
#         src_pts = approx.reshape(4, 2).astype(np.float32)
#         print(f"Source points: {src_pts}")

#         # Define the destination points
#         dst_pts = np.float32([[0, 0], [800, 0], [800, 600], [0, 600]])

#         # Get the perspective transformation matrix
#         matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

#         # Apply the perspective transformation
#         img_transformed = cv2.warpPerspective(img, matrix, (800, 600))

#         # Display the transformed image
#         cv2.imshow("Transformed", img_transformed)
#         cv2.waitKey(0)

# cv2.destroyAllWindows()
