# Face message

std_msgs/Header 				header				# Header
string 							id					# Id of the user
float64 						confidence			# Confidence
vision_msgs/BoundingBox2D 		bbox				# Bounding box with the face of the user
sensor_msgs/Image 				source_img			# Cropped image with the face of the user
