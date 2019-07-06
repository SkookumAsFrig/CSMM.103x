#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg


def publish_transforms():
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame"
    t1.transform.translation.x = 0.0
    t1.transform.translation.y = 1.0
    t1.transform.translation.z = 1.0
    q1 = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
    br.sendTransform(t1)

    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = -1.0
    t2.transform.translation.z = 0.0
    q2 = tf.transformations.quaternion_about_axis(1.50, (0,0,1))
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]
    br.sendTransform(t2)

    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame"
    t3.transform.translation.x = 0.0
    t3.transform.translation.y = 0.1
    t3.transform.translation.z = 0.1
    # t3.transform.rotation.x = 0
    # t3.transform.rotation.y = 0
    # t3.transform.rotation.z = 0
    # t3.transform.rotation.w = 1

    t1mat = numpy.dot(tf.transformations.translation_matrix((0.0, 1.0, 1.0)),
            tf.transformations.quaternion_matrix(q1))

    Pos1mat = t1mat[0:3, 3]
    Pos1mat = numpy.append(Pos1mat/numpy.linalg.norm(Pos1mat),1)

    t2mat = numpy.dot(tf.transformations.translation_matrix((0.0, -1.0, 0.0)),
            tf.transformations.quaternion_matrix(q2))

    t3mat = numpy.dot(tf.transformations.translation_matrix((0.0, 0.1, 0.1)),
            tf.transformations.euler_matrix(0,0,0))

    Tall = numpy.dot(t2mat, t3mat)

    # Posmat = Tall[0:3, 3]
    # Oriloc = -(Posmat/numpy.linalg.norm(Posmat))

    # whole = numpy.append(Oriloc,1)
    # whole.shape = (4,1)

    newarr = numpy.linalg.inv(Tall)

    Orimat = Pos1mat #numpy.array([0, 0, 0, 1]) #In global frame
    CamVec = numpy.dot(newarr, Orimat)[0:3] #In Camera frame,
    #the vector pointing from camera to origin
    CamVec = CamVec/numpy.linalg.norm(CamVec)
    #In Camera frame, x axis vector
    RotVec = numpy.array([1, 0, 0])

    v = numpy.cross(RotVec, CamVec)
    # s = numpy.linalg.norm(v)
    c = numpy.dot(RotVec, CamVec)

    vx = numpy.array([[0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0]])
    R = numpy.identity(3) + vx + numpy.dot(vx, vx)/(1+c) 
    #rotation matrix from RotVec to CamVec
    # finalVec = numpy.dot(R, RotVec)
    Rwhole = numpy.vstack((numpy.hstack((R, numpy.zeros((3,1)))), numpy.array([0,0,0,1])))

    q3 = tf.transformations.quaternion_from_matrix(Rwhole)

    t3.transform.rotation.x = q3[0]
    t3.transform.rotation.y = q3[1]
    t3.transform.rotation.z = q3[2]
    t3.transform.rotation.w = q3[3]

    br.sendTransform(t3)


if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
