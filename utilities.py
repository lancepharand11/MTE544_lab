from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# DONE Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = sqrt(1 + 2 * (w * y - x * z))
    cosp = sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * atan2(sinp, cosp) - M_PI / 2

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = atan2(siny_cosp, cosy_cosp)

    return yaw


#DONE Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    goal_x = goal_pose[0]
    goal_y = goal_pose[1]
    current_x = current_pose[0]
    current_y = current_pose[1]
    error_linear = sqrt((goal_y - current_y) ** 2 + (goal_x - current_x) ** 2)
    return error_linear

def normalize_angle(angle):
    angle = angle % (2 * M_PI)
    if angle > M_PI:
        angle -= 2 * M_PI
    return angle


#DONE Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    goal_x = goal_pose[0]
    goal_y = goal_pose[1]
    current_x = current_pose[0]
    current_y = current_pose[1]
    theta = current_pose[2]

    # Map theta between -pi and pi
    theta = normalize_angle(theta) 

    # theta = (theta + M_PI) % (2 * M_PI) - M_PI 
    angle_to_goal = atan2(goal_y - current_y, goal_x - current_x)
    error_angular = angle_to_goal - theta
    error_angular = normalize_angle(error_angular)

    return error_angular
