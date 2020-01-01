import numpy as np
import output as fuzzy

class FuzzyImplementation:

    def __init__(self, goal):

        self.goal = goal

    def getOdometryErrors(self, current_pos, current_orientation):

        # current position in meters and angular deviation between the current heading of the robot and the line joining the current position and goal

        dist = np.sqrt((self.goal[0] - current_pos[0])**2 + (self.goal[1] - current_pos[1])**2)
        angle  = np.arctan2((self.goal[1] - current_pos[1]), (self.goal[0] - current_pos[0]))*180/np.pi
        angle_deviation = -(current_orientation - angle)

        return  dist, angle_deviation

    def membership_functions_TFLC(self, dist, angle_dev):

        # Distance - Zero:z, Very Near: vn, Near:n, Mid-way:m, Far:f
        mem_dist = {'z': 0.1, 'n':1.0, 'f':2.0}
        mem_dist_keys = ['z', 'n', 'f']

        # Angular Position - Neg Right-Angled: nr, Neg Thirty: nt, Aligned: a
        #                    Pos Thirty: rt, Pos Right-Angled: rr
        mem_angle_dev = {'nr': -90, 'nt': -30, 'a': 0, 'rt':30, 'rr':90}
        mem_angle_dev_keys = ['nr', 'nt', 'a', 'rt', 'rr']

        nan = 'nan'
        # print (dist, angle_dev)

        TFLC_dict_dist = {'z': 0.0, 'n': 0.0, 'f': 0.0}
        TFLC_dict_angle = {'nr': 0.0, 'nt': 0.0, 'a': 0.0, 'rt':0.0, 'rr':0.0}

        def membership_dist():

            prev_key = None
            lenL = len(mem_dist_keys)

            for index, key in enumerate(mem_dist_keys):

                if index==0 and dist <= mem_dist[key]:
                    TFLC_dict_dist[key] = 1.0
                elif index == lenL-1 and dist>mem_dist[mem_dist_keys[lenL-1]]:
                    TFLC_dict_dist[mem_dist_keys[lenL-1]] = 1.0
                elif dist<=mem_dist[key] and dist>mem_dist[prev_key]:
                    w = ((dist - mem_dist[prev_key]) / (float)(mem_dist[key] - mem_dist[prev_key]))
                    TFLC_dict_dist[key] = w
                    TFLC_dict_dist[prev_key] = 1.0-w

                prev_key = key

        def membership_angle():

            pprev_key = None
            lenLL = len(mem_angle_dev_keys)

            for index, key in enumerate(mem_angle_dev_keys):

                if index==0 and angle_dev <= mem_angle_dev[key]:
                    TFLC_dict_angle[key] = 1.0
                elif index == lenLL-1 and angle_dev>mem_angle_dev[mem_angle_dev_keys[lenLL-1]]:
                    TFLC_dict_angle[mem_angle_dev_keys[lenLL-1]] = 1.0
                elif angle_dev<=mem_angle_dev[key] and angle_dev>mem_angle_dev[pprev_key]:
                    w = ((angle_dev-mem_angle_dev[pprev_key])/(float)(mem_angle_dev[key] - mem_angle_dev[pprev_key]))
                    TFLC_dict_angle[key] = w
                    TFLC_dict_angle[pprev_key] = 1.0-w

                pprev_key = key

        membership_dist()
        membership_angle()

        # print TFLC_dict_dist
        # print TFLC_dict_angle

        # Linear Velocity: 0, low, high
        TFLC_linVelDict = {'z': 0.0, 'l': 0.2, 'h': 0.4}
        TFLC_linV_keys = ['z', 'l', 'h']

        # Angular Velocity:  -high, -low, 0, low, high
        TFLC_angVelDict = {'nh': -0.2, 'nl': -0.1, 'z': 0, 'pl':0.1, 'ph': 0.2}
        TFLC_angV_keys = ['nh', 'nl', 'z', 'pl', 'ph']

        # nr - z,n,f | nt - z,n,f | a - z,n,f | rt - z,n,f | rr - z,n,f
        # Each is a tuple with linear velocity and angular velocity
        TFLC_VelocityValues = []

        for angKey in TFLC_angV_keys:
            for linKey in TFLC_linV_keys:
                TFLC_VelocityValues.append([TFLC_linVelDict[linKey], TFLC_angVelDict[angKey]])

        count = -1
        weight_sum, linearVelocity_sum, angularVelocity_sum = 0.0, 0.0, 0.0

        for ang_dev_key in mem_angle_dev_keys:
            for dist_key in mem_dist_keys:
                count += 1
                weii = min(TFLC_dict_angle[ang_dev_key], TFLC_dict_dist[dist_key])
                weight_sum += weii
                linearVelocity_sum += TFLC_VelocityValues[count][0]*weii
                angularVelocity_sum += TFLC_VelocityValues[count][1]*weii

        linearVelocity = linearVelocity_sum/weight_sum
        angularVelocity = angularVelocity_sum/weight_sum

        return linearVelocity, angularVelocity


    def membership_functions_OAFLC(self, obs_values):

        # Distance - Very Near:vn, Near:n, Far:f
        mem_obs_dist = {'vn': 0.6, 'n':1.5, 'f':3.0}
        mem_obs_dist_keys = ['vn', 'n', 'f']

        nan = 'nan'

        # print (obs_dist, obs_angle)

        def membership_obs_dist(value):

            prev_key = None
            lenL = len(mem_obs_dist_keys)

            for index, key in enumerate(mem_obs_dist_keys):
                
                if index==0 and value <= mem_obs_dist[key]:
                    return 1, key, nan
                elif index== lenL-1 and value>mem_obs_dist[mem_obs_dist_keys[lenL-1]]:
                    return 1, mem_obs_dist_keys[lenL-1], nan
                elif value<=mem_obs_dist[key] and value>mem_obs_dist[prev_key]:
                    return ((value - mem_obs_dist[prev_key]) / (mem_obs_dist[key] - mem_obs_dist[prev_key])), key, prev_key
                
                prev_key = key

        def calculateVelocity(membership_dict):

            pointer = 0

            weightSum = 0
            linVelSum = 0
            angVelSum = 0

            for aKey in mem_obs_dist_keys:
                for bKey in mem_obs_dist_keys:
                    for cKey in mem_obs_dist_keys:
                        
                        minVal = min(membership_dict[0][aKey], membership_dict[1][bKey], membership_dict[2][cKey])
                        lin, ang = fuzzy.OAFLC_VelocityValues[pointer]

                        if minVal!=0:
                            weightSum += minVal
                            linVelSum += minVal*lin
                            angVelSum += minVal*ang

                        pointer += 1

            return (linVelSum/weightSum, angVelSum/weightSum)

        mdd_a = [] 
        for value in obs_values:
            membership_weights = {'vn': 0.0, 'n': 0.0, 'f':0.0}
            w, key, next_key = membership_obs_dist(value)
            
            if next_key == nan:
                membership_weights[key] = w
            else:
                membership_weights[key] = w
                membership_weights[next_key] = 1-w

            mdd_a.append(membership_weights)

        linearVelocity, angularVelocity = calculateVelocity(mdd_a)

        return linearVelocity, angularVelocity

    def membership_functions_FUSION(self, weight_tau):
        '''Defining membership functions for behavior fusion of the osbtacle avoidance and the tracking FLC systems'''

        # This function basically uses the same membership as does the OAFLC as the distribution of the same is
        # responsible for determining how much value is to be given to the fusion system
        nan = 'nan'

        mfw_OAFLC = {'small':0.25, 'medium':0.50, 'large':0.75}
        mfw_OAFLC_keys = ['small', 'medium', 'large']

        def calc():
            if weight_tau <= mfw_OAFLC[mfw_OAFLC_keys[0]]:
                return 1, mfw_OAFLC_keys[0], nan
            elif weight_tau > mfw_OAFLC[mfw_OAFLC_keys[-1]]:
                return 1, mfw_OAFLC_keys[-1], nan
            elif weight_tau <= mfw_OAFLC[mfw_OAFLC_keys[1]] and weight_tau > mfw_OAFLC[mfw_OAFLC_keys[0]]:
                return ((weight_tau - mfw_OAFLC[mfw_OAFLC_keys[0]])/(mfw_OAFLC[mfw_OAFLC_keys[1]] - mfw_OAFLC[mfw_OAFLC_keys[0]])), mfw_OAFLC_keys[1],\
                                                                                                                                mfw_OAFLC_keys[0]
            elif weight_tau <= mfw_OAFLC[mfw_OAFLC_keys[2]] and weight_tau > mfw_OAFLC[mfw_OAFLC_keys[1]]:
                return ((weight_tau - mfw_OAFLC[mfw_OAFLC_keys[1]])/(mfw_OAFLC[mfw_OAFLC_keys[2]] - mfw_OAFLC[mfw_OAFLC_keys[1]])), mfw_OAFLC_keys[2],\
                                                                                                                                mfw_OAFLC_keys[1]
        mf_fusion = calc()
        print ("Membership function for behavior fusion", mf_fusion)

        return mf_fusion

    def behavior_fusion(self, values_tflc, values_oaflc):
        '''Fusing  the individual behaviors of the TFLC and the OAFLC

        values_tflc     - linear X, linear Y and angular Z velocities finally provided by the TFLC
        values_oaflc    - linear X, linear Y and angular Z velocities finally provided by the OAFLC
        '''

        # Need to call the weight deciding function
        weights = None
        w = 0.62

        values_final = [ w*a - (1-w)*b for a,b in zip(values_oaflc, values_tflc)]

        # Final values of the linear X, linear Y and  angular Z velocities
        print ("Final values of velocities and accelerations going to the TurtleBot", values_final)

        return values_final


# if __name__ == '__main__':

#     obje = FuzzyImplementation((6,9))
    
#     a, b = obje.membership_functions_TFLC(0.82, -42)
#     c, d = obje.membership_functions_OAFLC(0.79, -7.5)
    
#     e = obje.membership_functions_FUSION(0.65)