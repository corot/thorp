#!/usr/bin/env python
import argparse
import rospy
import actionlib
import moveit_msgs.msg as moveit_msgs
import geometry_msgs
import shape_msgs

def main():
    rospy.init_node('add_collision_objects')
    
    parser = argparse.ArgumentParser(description='Add objects into the MoveIt planning scene')
    parser.add_argument('--clean', dest='clean', action='store_true', default=False, help='Remove all objects from the scene')
    args=parser.parse_args()
    
    pub_collision_object = rospy.Publisher("/korus/collision_object",
                                           moveit_msgs.CollisionObject,
                                           latch = True)
    rospy.sleep(0.5)
    
    ''' publish table '''
    collision_object = moveit_msgs.CollisionObject()
    collision_object.header.stamp = rospy.Time.now()
    collision_object.header.frame_id = "/base_footprint";
    collision_object.id = "table"
#    collision_object.type = 
    object_shape = shape_msgs.msg.SolidPrimitive()
    object_shape.type = shape_msgs.msg.SolidPrimitive.BOX
    object_shape.dimensions.append(0.6) # BOX_X
    object_shape.dimensions.append(1.0) # BOX_Y
    object_shape.dimensions.append(0.65) # BOX_Z
#    object_shape.dimensions.append(0.4) # BOX_Z
    collision_object.primitives.append(object_shape)
    object_pose = geometry_msgs.msg.Pose()
    object_pose.position.x = 1
    object_pose.position.y = 0.0
    object_pose.position.z = 0.325
#    object_pose.position.z = 0.2
    object_pose.orientation.w = 1.0
    collision_object.primitive_poses.append(object_pose)
    if args.clean:
        collision_object.operation = moveit_msgs.CollisionObject.REMOVE
        rospy.loginfo('Removing table ...')
    else:
        collision_object.operation = moveit_msgs.CollisionObject.ADD
        rospy.loginfo('Adding table ...')
    pub_collision_object.publish(collision_object)

    
    rospy.sleep(0.5)
    
    ''' publish object '''
    collision_object = moveit_msgs.CollisionObject()
    collision_object.header.stamp = rospy.Time.now()
    collision_object.header.frame_id = "/base_footprint";
    collision_object.id = "pickup_object"
#    collision_object.type = 
    object_shape = shape_msgs.msg.SolidPrimitive()
    object_shape.type = shape_msgs.msg.SolidPrimitive.CYLINDER
    object_shape.dimensions.append(0.20) # CYLINDER_HEIGHT
    object_shape.dimensions.append(0.025) # CYLINDER_RADIUS
    collision_object.primitives.append(object_shape)
    object_pose = geometry_msgs.msg.Pose()
    object_pose.position.x = 0.8
    object_pose.position.y = 0.0
    object_pose.position.z = 0.77
    object_pose.orientation.w = 1.0
    collision_object.primitive_poses.append(object_pose)
    if args.clean:
        collision_object.operation = moveit_msgs.CollisionObject.REMOVE
        rospy.loginfo('Removing object ...')
    else:
        collision_object.operation = moveit_msgs.CollisionObject.ADD
        rospy.loginfo('Adding object ...')
    pub_collision_object.publish(collision_object)
    
#    rospy.spin()
    
    return
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass