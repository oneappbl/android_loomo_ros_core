package org.loomo.ros;

import android.util.Log;

import com.segway.robot.sdk.perception.sensor.Sensor;

import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;
import org.ros.time.NtpTimeProvider;

import nav_msgs.Odometry;
import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import sensor_msgs.Range;
import std_msgs.Float32;
import std_msgs.Int8;
import tf2_msgs.TFMessage;
import geometry_msgs.Twist;

public class LoomoRosBridgeNode extends AbstractNodeMain {
    private static final String TAG = "LoomoRosBridgeNode";

    public String RsDepthOpticalFrame = "rs_depth_optical_frame";
    public String RsColorOpticalFrame = "rs_color_optical_frame";
    public String FisheyeOpticalFrame = "fisheye_optical_frame";
    public String UltrasonicFrame     = "ultrasonic_frame";
    public String LeftInfraredFrame   = "infrared_left_frame";
    public String RightInfraredFrame  = "infrared_right_frame";

    public ConnectedNode mConnectedNode;
    public MessageFactory mMessageFactory;
    public Publisher<Image> mFisheyeCamPubr;
    public Publisher<CompressedImage> mFisheyeCompressedPubr;
    public Publisher<CameraInfo> mFisheyeCamInfoPubr;
    public Publisher<Image> mRsColorPubr;
    public Publisher<CompressedImage> mRsColorCompressedPubr;
    public Publisher<Image> mRsDepthPubr;
    public Publisher<CameraInfo> mRsColorInfoPubr;
    public Publisher<CameraInfo> mRsDepthInfoPubr;
    public Publisher<TFMessage> mTfPubr;
    public Publisher<Range> mInfraredPubrLeft;
    public Publisher<Range> mInfraredPubrRight;
    public Publisher<Range> mUltrasonicPubr;
    public Publisher<Float32> mBasePitchPubr;
    public Publisher<Odometry> mOdometryPubr;

    public Subscriber<Int8> mTransformSubr;
    public Subscriber<Twist> mCmdVelSubr;

    public NtpTimeProvider mNtpProvider;

    public String node_name = "loomo_ros_bridge_node";
    public String tf_prefix = "LO01";
    public boolean should_pub_ultrasonic = true;
    public boolean should_pub_infrared = true;
    public boolean should_pub_base_pitch = true;
    public boolean use_tf_prefix = true;

    private boolean is_started = false;
    private Runnable mOnStarted;
    private Runnable mOnShutdown;

    public LoomoRosBridgeNode(Runnable onStarted, Runnable onShutdown) {
        super();
//        this.mNtpProvider = ntpTimeProvider;
        Log.d(TAG, "Created instance of LoomoRosBridgeNode().");
        mOnStarted = onStarted;
        mOnShutdown = onShutdown;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Log.d(TAG, "onStart()");
        super.onStart(connectedNode);

        Log.d(TAG, "onStart() creating publishers.");
        mConnectedNode = connectedNode;
        mMessageFactory = connectedNode.getTopicMessageFactory();

        if (use_tf_prefix == false){
            tf_prefix = "";
        }

        if (use_tf_prefix){
            RsDepthOpticalFrame = tf_prefix + "_" + RsDepthOpticalFrame;
            RsColorOpticalFrame = tf_prefix + "_" + RsColorOpticalFrame;
            FisheyeOpticalFrame = tf_prefix + "_" + FisheyeOpticalFrame;
            UltrasonicFrame = tf_prefix + "_" + UltrasonicFrame;
            LeftInfraredFrame = tf_prefix + "_" + LeftInfraredFrame;
            RightInfraredFrame =  tf_prefix + "_" + RightInfraredFrame;
        }

        // Create publishers for many Loomo topics
        mFisheyeCamPubr = connectedNode.newPublisher(tf_prefix+"/fisheye/rgb/image", Image._TYPE);
        mFisheyeCompressedPubr = connectedNode.newPublisher(tf_prefix+"/fisheye/rgb/image/compressed", CompressedImage._TYPE);
        mFisheyeCamInfoPubr = connectedNode.newPublisher(tf_prefix+"/fisheye/rgb/camera_info", CameraInfo._TYPE);
        mRsColorPubr = connectedNode.newPublisher(tf_prefix+"/realsense_loomo/rgb/image", Image._TYPE);
        mRsColorCompressedPubr = connectedNode.newPublisher(tf_prefix+"/realsense_loomo/rgb/image/compressed", CompressedImage._TYPE);
        mRsColorInfoPubr = connectedNode.newPublisher(tf_prefix+"/realsense_loomo/rgb/camera_info", CameraInfo._TYPE);
        mRsDepthPubr = connectedNode.newPublisher(tf_prefix+"/realsense_loomo/depth/image", Image._TYPE);
        mRsDepthInfoPubr = connectedNode.newPublisher(tf_prefix+"/realsense_loomo/depth/camera_info", CameraInfo._TYPE);
        mTfPubr = connectedNode.newPublisher("/tf", TFMessage._TYPE);
        mInfraredPubrLeft = connectedNode.newPublisher(tf_prefix+"/left_infrared", Range._TYPE);
        mInfraredPubrRight = connectedNode.newPublisher(tf_prefix+"/right_infrared", Range._TYPE);
        mUltrasonicPubr = connectedNode.newPublisher(tf_prefix+"/ultrasonic", Range._TYPE);
        mBasePitchPubr = connectedNode.newPublisher(tf_prefix+"/base_pitch", Float32._TYPE);
        mOdometryPubr = connectedNode.newPublisher(tf_prefix+"/odom", Odometry._TYPE);

        // Subscribe to commanded twist msgs (e.g. from joystick or autonomous driving software)
        mCmdVelSubr = mConnectedNode.newSubscriber(tf_prefix+"/cmd_vel", Twist._TYPE);

        // Subscribe to a topic instructing the loomo to change modes
        mTransformSubr = mConnectedNode.newSubscriber(tf_prefix+"/mode", Int8._TYPE);

        mOnStarted.run();
    }

    @Override
    public void onShutdown(Node node) {
        super.onShutdown(node);

        mOnShutdown.run();
    }

    @Override
    public void onShutdownComplete(Node node) {
        super.onShutdownComplete(node);
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
    }

    @Override
    public GraphName getDefaultNodeName() {
        if (use_tf_prefix) {
            node_name = tf_prefix + "/" + node_name;
        }
        return GraphName.of(node_name);
    }

}
