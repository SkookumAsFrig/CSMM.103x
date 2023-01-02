#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S

def omega_from_euler(phi, theta, psi):
    rot_mat = numpy.array([[1, 0, math.sin(theta)],[0, math.cos(psi), -math.sin(psi)*math.cos(theta)]\
        ,[0, -math.sin(psi), -math.cos(psi)*math.cos(theta)]])
    omega = numpy.dot(rot_mat, numpy.array([psi, theta, phi]))
    return omega

def R_from_euler(phi, theta, psi):
    R = numpy.zeros((3,3))
    R[0,0] = math.cos(phi)*math.cos(theta)
    R[0,1] = math.cos(phi)*math.sin(theta)*math.sin(psi) - math.sin(phi)*math.cos(psi)
    R[0,2] = math.cos(phi)*math.sin(theta)*math.sin(psi) + math.sin(phi)*math.cos(psi)
    R[1,0] = math.sin(phi)*math.cos(theta)
    R[1,1] = math.sin(phi)*math.sin(theta)*math.sin(psi) + math.cos(phi)*math.cos(psi)
    R[1,2] = math.sin(phi)*math.sin(theta)*math.cos(psi) - math.cos(phi)*math.sin(psi)
    R[2,0] = -math.sin(theta)
    R[2,1] = math.cos(theta)*math.sin(phi)
    R[2,2] = math.cos(theta)*math.cos(phi)
    return R

def euler_from_R(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    if abs(R[2,0])==1:
        raise ValueError("cos theta = 0, solutions degenerate")
    else:
        phi1 = math.atan2(R[1,0], R[0,0])
        theta1 = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
        psi1 = math.atan2(R[2,1], R[2,2])
        phi2 = math.atan2(-R[1,0], -R[0,0])
        theta2 = math.atan2(-R[2,0], -math.sqrt(R[2,1]**2 + R[2,2]**2))
        psi2 = math.atan2(-R[2,1], -R[2,2])
        # [phi1, theta1, psi1, phi2, theta2, psi2]
    return numpy.array([psi1, theta1, phi1])

# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    num_joints = len(joint_transforms)
    dq = numpy.zeros(num_joints)
    #-------------------- Fill in your code here ---------------------------    
    deltaXmat = numpy.dot(numpy.linalg.inv(b_T_ee_current), b_T_ee_desired)
    deltaTrans = deltaXmat[:3, 3]
    ang, ax = rotation_from_matrix(deltaXmat)
    deltaRot = ang*ax

    # print('New Data\n######################')
    # print('New Rotation Data\n######################')
    ang_base, ax_base = rotation_from_matrix(b_T_ee_current)
    ang_base_des, ax_base_des = rotation_from_matrix(b_T_ee_desired)
    current_rotvec = ang_base*ax_base
    des_rotvec = ang_base_des*ax_base_des
    delta_xrot_base = des_rotvec - current_rotvec
    # print('desired angle*axis base: {}, current angle*axis base: {}, delta omega_base is {}').format(des_rotvec, current_rotvec, delta_xrot_base)
    # vel_rot = numpy.dot(b_T_ee_current[:3,:3].transpose(), delta_x_base)
    # print('delta omega_ee from base is ')
    # print(vel_rot)

    # print('Translation Data\n######################')
    # print('delta x_ee from original is ')
    # print(deltaTrans)
    # print('delta x_base from original is ')
    # print(numpy.dot(b_T_ee_current[:3,:3],deltaTrans))
    # print('delta x_base from new method is ')
    deltaX_base = b_T_ee_desired[:3, 3] - b_T_ee_current[:3, 3]
    # print(deltaX_base)

    # print('Rotation Data\n######################')
    # print('delta omega_ee from original is ')
    # print(deltaRot)
    # print('delta omega_base from original is ')
    deltaRot_base = numpy.dot(b_T_ee_current[:3,:3],deltaRot)
    # print(deltaRot_base)
    # print('delta omega_base using euler is (wrong, cannot directly add/subtract euler angles)')
    # print(euler_from_R(b_T_ee_desired)-euler_from_R(b_T_ee_current))
    # print('delta omega_ee using euler is ')
    # euler_ee = euler_from_R(deltaXmat[:3,:3])
    # print(euler_ee)
    # print('delta omega_ee using euler rot_method is ')
    # print(omega_from_euler(euler_ee[2], euler_ee[1], euler_ee[0]))
    #-------------------- DEBUGGING ---------------------------

    # print(b_T_ee_current)
    # print(b_T_ee_desired)
    # print(deltaXmat)
    # print("deltaTrans is {}").format(deltaTrans)
    # deltaglobepos = b_T_ee_desired[:3, 3] - b_T_ee_current[:3, 3]
    # print("deltaTrans before REE is {}").format(deltaglobepos)
    # REE = b_T_ee_current[:3,:3]
    # deltaEEpos = numpy.dot(REE, deltaglobepos)
    # print("deltaTrans from rotational is {}").format(deltaEEpos)

    #-------------------- DEBUGGING ---------------------------
    Tlgain = 5
    Rotgain = 1
    VTEE = Tlgain*deltaTrans
    VTsize = numpy.linalg.norm(VTEE)
    VREE = Rotgain*deltaRot
    VRsize = numpy.linalg.norm(VREE)

    Tth = 0.3
    if VTsize > Tth:
        VTEE = (Tth/VTsize)*VTEE
    
    Rth = 3
    if VRsize > Rth:
        VREE = (Rth/VRsize)*VREE

    VEE = numpy.hstack((VTEE, VREE))

    vth = 1e-3

    for ind in range(len(VEE)):
        if abs(VEE[ind])<vth:
            VEE[ind] = 0

    #-------------------- Global Frame Code ---------------------------
    VTEE_b = Tlgain*deltaX_base
    VREE_b = Rotgain*deltaRot_base #delta_xrot_base
    #using delta xrot base obtained from subtracting two angle-axis rotation vectors appears to result in some instabilities near singularity
    VTsize_b = numpy.linalg.norm(VTEE_b)
    VRsize_b = numpy.linalg.norm(VREE_b)
    if VTsize_b > Tth:
        VTEE_b = (Tth/VTsize_b)*VTEE_b
    if VRsize_b > Rth:
        VREE_b = (Rth/VRsize_b)*VREE_b

    VEE_b = numpy.hstack((VTEE_b, VREE_b))

    for ind in range(len(VEE_b)):
        if abs(VEE_b[ind])<vth:
            VEE_b[ind] = 0

    Jac_b = numpy.zeros((6,1))
    #-------------------- Global Frame Code End ---------------------------

    # print(VEE)

    Jac = numpy.zeros((6,1))

    last_jt = joint_transforms[0]
    for jt in joint_transforms:
        # debugging lines

        # if numpy.array_equal(last_jt,jt):
        #     print(jt)
        # else:
        #     print(numpy.dot(numpy.linalg.inv(last_jt),jt))
        # last_jt = jt
        # print('###################')
        # print(jt[:3,3])
        # print('###################')

        # above lines print each joint transform to the next joint. shows that there is some
        # funky business going on with the align rotation axis with z operation done in the bottom
        # those funky ones (joints 1, 3, 5 corresponding to jt 2, 4, 6) do their z alignment and
        # distance offset in the same rotation matrix, so the offset column changes
        # all z aligned frames behave as expected
        # NOTE: jt frames do not align with rviz joint frames! they are not representative.
        # debugging lines end
        Transf_EE = numpy.dot(numpy.linalg.inv(jt), b_T_ee_current)
        Rba = numpy.linalg.inv(Transf_EE[:3, :3])
        Tab = Transf_EE[:3, 3]
        MomArm = -numpy.dot(Rba, S_matrix(Tab))
        RightHalf = numpy.vstack((MomArm, Rba))
        LastCol = RightHalf[:,2]
        LastCol.shape = (6,1)
        Jac = numpy.hstack((Jac, LastCol))
        # Note, can also use given adjoint matrix function to simplify code

        #-------------------- Global Frame Code ---------------------------
        Rb_i_1_last_col = jt[:3, 2]
        db_ee = joint_transforms[-1][:3, 3]
        db_i_1 = jt[:3, 3]
        Jac_b_newcol_top = numpy.dot(S_matrix(Rb_i_1_last_col), db_ee - db_i_1)
        Jac_b_newcol_btm = Rb_i_1_last_col
        # print('newcol_top is {}').format(numpy.shape(Jac_b_newcol_top))
        # print(numpy.shape(Jac_b_newcol_btm))
        LastCol = numpy.hstack((Jac_b_newcol_top, Jac_b_newcol_btm))
        LastCol.shape = (6,1)
        Jac_b = numpy.hstack((Jac_b, LastCol))

    Jac_b = Jac_b[:, 1:]
    epsilon_b = 1e-7
    invJac_b = numpy.linalg.pinv(Jac_b, epsilon_b)

    dq_b = numpy.dot(invJac_b, VEE_b)

    biggestq = max(dq_b)

    if biggestq>1:
        dq_b = dq_b/biggestq

    if red_control:
        qsize = len(q_current)
        qmat = numpy.zeros(qsize)
        qgain = 1
        qmat[0] = qgain*(q0_desired - q_current[0])
        
        qn = numpy.dot(numpy.identity(qsize) - numpy.dot(invJac_b, Jac_b), qmat)
        dq_b = dq_b + qn

        #-------------------- Global Frame Code End ---------------------------

    Jac = Jac[:, 1:]
    epsilon = 1e-7
    invJac = numpy.linalg.pinv(Jac, epsilon)

    dq = numpy.dot(invJac, VEE)

    if red_control:
        qsize = len(q_current)
        qmat = numpy.zeros(qsize)
        qgain = 1
        qmat[0] = qgain*(q0_desired - q_current[0])
        
        qn = numpy.dot(numpy.identity(qsize) - numpy.dot(invJac, Jac), qmat)
        dq = dq + qn

    biggestq = max(dq)
    
    if biggestq>1:
        dq = dq/biggestq

    # print (dq_b)
    # print(invJac_b)

    #----------------------------------------------------------------------
    return dq_b
    
def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans,rot)
    return T

def adjoint_matrix(i_T_j):

    i_p_j = numpy.linalg.inv(i_T_j)[:3, 3]
    i_R_j = i_T_j[:3, :3]
    Ad = numpy.zeros((6,6))
    Ad[:3,:3] = i_R_j
    Ad[3:,3:] = i_R_j
    Ad[:3,3:] = -numpy.dot(i_R_j,S_matrix(i_p_j))

    return Ad

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
        
    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)
        
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
        
if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
