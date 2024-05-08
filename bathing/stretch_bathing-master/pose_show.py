import cv2
import cv2.aruco as aruco
import pickle
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
from datetime import date


mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils 

DESIRED_HEIGHT = 480
DESIRED_WIDTH = 480

def pixel_transform(tag_loc, pose_keypoint):
  return [tag_loc[0]-pose_keypoint[0],tag_loc[1]-pose_keypoint[1]]

def tag_toknee_transform(tag_loc, results, w, h):
  knee_x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_KNEE].x * w
  knee_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].y * h
  return (tag_loc, [knee_x,knee_y], pixel_transform(tag_loc,[knee_x,knee_y]))

def knee_to_ankle_transform(results, w, h):
  knee_x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_KNEE].x * w
  knee_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_KNEE].y * h
  ankle_x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ANKLE].x * w
  ankle_y = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ANKLE].y * h
  return ([knee_x,knee_y],[ankle_x,ankle_y], [knee_x-ankle_x,knee_y-ankle_y])

def show_estimated_pose(image):
  with mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.5) as pose:
      # Convert the BGR image to RGB and process it with MediaPipe Pose.
    results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    if not results.pose_landmarks:
      return None
    annotated_image = image.copy()
    mp_drawing.draw_landmarks(
        annotated_image,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS)
    return annotated_image, results

#https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def show_aruco_tags(img, dist, mtx):
  image = img.copy()
  arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
  arucoParams = cv2.aruco.DetectorParameters_create()
  (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)
  #distance detection
  markerSizeInCM = 4
  rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)
  centers = {}
  metercenters = {id:i for ([id],i) in zip(ids,tvec)}
  #draw on image
  if len(corners)>0:
    # flatten the ArUco IDs list
    ids = ids.flatten()
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      # extract the marker corners (which are always returned in
      # top-left, top-right, bottom-right, and bottom-left order)
      corners = markerCorner.reshape((4, 2))
      (topLeft, topRight, bottomRight, bottomLeft) = corners
      # convert each of the (x, y)-coordinate pairs to integers
      topRight = (int(topRight[0]), int(topRight[1]))
      bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
      bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
      topLeft = (int(topLeft[0]), int(topLeft[1]))
      # draw the bounding box of the ArUCo detection
      cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
      cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
      cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
      cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
      # compute and draw the center (x, y)-coordinates of the ArUco
      # marker
      cX = int((topLeft[0] + bottomRight[0]) / 2.0)
      cY = int((topLeft[1] + bottomRight[1]) / 2.0)
      centers[markerID]=[cX,cY]
      cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
      # draw the ArUco marker ID on the image
      cv2.putText(image, str(markerID),
        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)
  return image, centers, metercenters
        
if __name__ == '__main__':
  today = date.today()

  #start realsense camera
  pipe = rs.pipeline()
  config = rs.config()
  config.enable_device('141722070195')
  config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
  config.enable_stream(rs.stream.color)
  pipe.start(config)

  #capturing image
  for i in range(10):
    frames = pipe.wait_for_frames()
    
  color_frame = frames.get_color_frame()
  intrinsics=color_frame.profile.as_video_stream_profile().get_intrinsics() 

  color = np.asanyarray(color_frame.get_data())
  colorizer = rs.colorizer()
  # Create alignment primitive with color as its target stream:
  align = rs.align(rs.stream.color)
  frameset = align.process(frames)
  img = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
  cv2.imwrite('images/'+str(today)+'.png', img)
  imgs = ['images/'+str(today)+'.png']
  images = {imgs[0]: cv2.imread(imgs[0])}

  #calculating camera intrinsics
  dist = np.array(intrinsics.coeffs)
  mtx = np.array([[intrinsics.fx, 0, intrinsics.width],[0,intrinsics.fy,intrinsics.height],[0,0,1]])
  for name, image in images.items():
    image_height, image_width, _ = image.shape

    #pose landmarks are w.r.t bottom left corner
    est, results = show_estimated_pose(image)
    cv2.imshow("Image",est)

    cv2.waitKey(0)
    aru, centers, metercenters = show_aruco_tags(image, dist, mtx)
    cv2.imshow("Image", aru)

    cv2.waitKey(0)
    loc, pose_point, finaltf = tag_toknee_transform(centers[1],results,image_width, image_height)
    ([k1,k2],[a1,a2], knee_ankle_transform) = knee_to_ankle_transform(results, image_width, image_height)
    #line from tag to knee
    cv2.line(image, (loc[0],loc[1]), (loc[0]-int(finaltf[0]),loc[1]-int(finaltf[1])), (255, 0, 0), 1)
    #line from knee to ankle
    cv2.line(image, (int(k1),int(k2)), (int(a1),int(a2)), (255, 0, 0), 1)
    #location of tag (from bottom right) in cm and pixels
    tagloc_m = [metercenters[1][0][0],-1*metercenters[1][0][1]]
    tagloc_p = [image_width-loc[0],image_height-loc[1]]
    #cms/pixel ratio
    pixel2meterRatio = [tagloc_m[0]/tagloc_p[0], tagloc_m[1]/tagloc_p[1]]
    #print transformations in meters
    tag_to_knee_m = (pixel2meterRatio[0]*finaltf[0],pixel2meterRatio[1]*finaltf[1])
    knee_to_ankle_m = (pixel2meterRatio[0]*knee_ankle_transform[0],pixel2meterRatio[1]*knee_ankle_transform[1])

    print("Tag to Knee: %d",tag_to_knee_m)
    print("Knee to ankle: %d",knee_to_ankle_m)

    cv2.imshow("Image",image)
    cv2.waitKey(0)

    file_name = 'transforms/'+str(today)+'.pickle'
    with open(file_name, 'wb') as fobj:
        pickle.dump((tag_to_knee_m,knee_to_ankle_m), fobj, protocol = 2)
