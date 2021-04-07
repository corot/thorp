<!--
  Thorp perception
 -->

<launch>

  <!--  ************ COB Object Detection *************  -->

  <node name="cob_object_detection" pkg="cob_people_object_detection_tensorflow" type="cob_people_object_detection_tensorflow.py" output="screen" respawn="true">
    <rosparam command="load" file="$(find thorp_perception)/config/cob/object_detection.yaml"/>
  </node>

  <node name="cob_object_projection" pkg="cob_people_object_detection_tensorflow" type="projection.py" output="screen" respawn="true">
    <rosparam command="load" file="$(find thorp_perception)/config/cob/object_projection.yaml"/>
  </node>
</launch>
