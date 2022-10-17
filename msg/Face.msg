# Face message

std_msgs/Header 				header				# Header
string 							id					# Id of the user or "Spoof" if spoofing has been detected.
float64 						confidence			# Confidence
bool 							has_mask			# Whether the user has a facial mask or not
vision_msgs/BoundingBox2D 		bbox				# Bounding box with the face of the user
sensor_msgs/Image 				source_img			# Cropped image with the face of the user

# Constant
string 			SPOOF = Spoof