package org.loomo.ros;

import android.util.Log;
import android.util.Pair;

import com.segway.robot.algo.tf.AlgoTfData;
import com.segway.robot.sdk.locomotion.sbv.AngularVelocity;
import com.segway.robot.sdk.locomotion.sbv.Base;
import com.segway.robot.sdk.locomotion.sbv.LinearVelocity;
import com.segway.robot.sdk.perception.sensor.Sensor;
import com.segway.robot.sdk.perception.sensor.SensorData;
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.vision.calibration.ColorDepthCalibration;
import com.segway.robot.sdk.vision.calibration.Extrinsic;
import com.segway.robot.sdk.vision.calibration.MotionModuleCalibration;

import org.ros.message.Duration;
import org.ros.message.Time;

import java.util.Arrays;
import java.util.List;
import java.util.Queue;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import tf2_msgs.TFMessage;

/**
 * Created by mfe on 7/17/18.
 */

public class TFPublisher implements LoomoRosBridgeConsumer {
    public static final String TAG = "TFPublisher";

    public boolean mIsPubTF;
    private Thread mTFPublishThread;

    private Sensor mSensor = null;
    private Vision mVision = null;
    private Base mBase = null;
    private LoomoRosBridgeNode mBridgeNode = null;
    private Queue<Long> mDepthStamps;
    private Queue<Pair<Long, Time>> mDepthRosStamps;
    private ColorDepthCalibration mDepthCalibration;
    private MotionModuleCalibration mMotionCalibration;

    private boolean mStarted = false;

    public TFPublisher(Queue<Long> mDepthStamps, Queue<Pair<Long, Time>> mDepthRosStamps) {
        this.mDepthStamps = mDepthStamps;
        this.mDepthRosStamps = mDepthRosStamps;
    }

    public void loomo_started(Sensor mSensor) {
        this.mSensor = mSensor;
    }

    public void loomo_started(Vision mVision) {
        this.mVision = mVision;
    }

    public void loomo_started(Base mBase) {
        this.mBase = mBase;
    }

    @Override
    public void node_started(LoomoRosBridgeNode mBridgeNode) {
        this.mBridgeNode = mBridgeNode;
    }

    @Override
    public void start() {
        if (mSensor == null || mVision == null || mBridgeNode == null || mBase == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        } else if (mStarted) {
            Log.d(TAG, "already started");
            return;
        }

        mStarted = true;

        mDepthCalibration = mVision.getColorDepthCalibrationData();
        mMotionCalibration = mVision.getMotionModuleCalibrationData();

        Log.d(TAG, "Depth extrinsic data: " + mDepthCalibration.depthToColorExtrinsic.toString());

        Log.d(TAG, "start_tf()");
        if (mTFPublishThread == null) {
            mTFPublishThread = new TFPublisherThread();
        }
        mTFPublishThread.start();
    }

    @Override
    public void stop() {
        if(!mStarted) {
            return;
        }

        Log.d(TAG, "stop_tf()");
        try {
            mTFPublishThread.join();
        } catch (InterruptedException e) {
            Log.w(TAG, "onUnbind: mSensorPublishThread.join() ", e);
        }
    }

    private TransformStamped realsenseColorToDepthExtrinsic(Extrinsic extrinsic, Time time) {
        Vector3 vector3 = mBridgeNode.mMessageFactory.newFromType(Vector3._TYPE);

        // Extrinsic data is in mm, convert to meters
        vector3.setX(extrinsic.translation.x / 1000.0f);
        vector3.setY(extrinsic.translation.y / 1000.0f);
        vector3.setZ(extrinsic.translation.z / 1000.0f);

        Quaternion quaternion = mBridgeNode.mMessageFactory.newFromType(Quaternion._TYPE);
        org.ros.rosjava_geometry.Quaternion identity_quaternion = org.ros.rosjava_geometry.Quaternion.identity();
        quaternion.setX(identity_quaternion.getX());
        quaternion.setY(identity_quaternion.getY());
        quaternion.setZ(identity_quaternion.getZ());
        quaternion.setW(identity_quaternion.getW());

        Transform transform = mBridgeNode.mMessageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector3);
        transform.setRotation(quaternion);
        TransformStamped transformStamped = mBridgeNode.mMessageFactory.newFromType(TransformStamped._TYPE);
        transformStamped.setTransform(transform);
        transformStamped.setChildFrameId(mBridgeNode.RsDepthOpticalFrame);
        transformStamped.getHeader().setFrameId(mBridgeNode.RsColorOpticalFrame);
        // Future-date this static transform so that depth_image_proc/register can get the TF easier
        transformStamped.getHeader().setStamp(time.add(Duration.fromMillis(1000)));
        return transformStamped;
    }

    private TransformStamped realsenseColorToOpticalFrame(Time time) {
        Vector3 vector3 = mBridgeNode.mMessageFactory.newFromType(Vector3._TYPE);

        // Assume that rscolor_optical_frame is in the same physical location as Sensor.RS_COLOR_FRAME, but with a different rotation
        vector3.setX(0);
        vector3.setY(0);
        vector3.setZ(0);

        // Rotate the vector around the X axis
        org.ros.rosjava_geometry.Quaternion camera_rotation_x = org.ros.rosjava_geometry.Quaternion.fromAxisAngle(org.ros.rosjava_geometry.Vector3.xAxis(), Math.PI / -2.0);
        org.ros.rosjava_geometry.Quaternion camera_rotation_y = org.ros.rosjava_geometry.Quaternion.fromAxisAngle(org.ros.rosjava_geometry.Vector3.yAxis(), Math.PI / 2.0);
        org.ros.rosjava_geometry.Quaternion camera_rotation_z = org.ros.rosjava_geometry.Quaternion.fromAxisAngle(org.ros.rosjava_geometry.Vector3.zAxis(), Math.PI / 2.0);

        camera_rotation_x = camera_rotation_x.multiply(camera_rotation_y);
        //camera_rotation_x = camera_rotation_x.multiply(camera_rotation_z);

        Quaternion quaternion = mBridgeNode.mMessageFactory.newFromType(Quaternion._TYPE);
        quaternion.setX(camera_rotation_x.getX());
        quaternion.setY(camera_rotation_x.getY());
        quaternion.setZ(camera_rotation_x.getZ());
        quaternion.setW(camera_rotation_x.getW());

        Transform transform = mBridgeNode.mMessageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector3);
        transform.setRotation(quaternion);

        TransformStamped transformStamped = mBridgeNode.mMessageFactory.newFromType(TransformStamped._TYPE);
        transformStamped.setTransform(transform);
        transformStamped.setChildFrameId(mBridgeNode.RsColorOpticalFrame);
        transformStamped.getHeader().setFrameId(mBridgeNode.tf_prefix + "_" + Sensor.RS_COLOR_FRAME);
        transformStamped.getHeader().setStamp(time);
        return transformStamped;
    }

    private Transform staticFullTransform(float t_x, float t_y, float t_z, float q_x, float q_y, float q_z, float q_w) {
        Vector3 vector3 = mBridgeNode.mMessageFactory.newFromType(Vector3._TYPE);
        vector3.setX(t_x);
        vector3.setY(t_y);
        vector3.setZ(t_z);

        Quaternion quaternion = mBridgeNode.mMessageFactory.newFromType(Quaternion._TYPE);
        quaternion.setX(q_x);
        quaternion.setY(q_y);
        quaternion.setZ(q_z);
        quaternion.setW(q_w);

        Transform transform = mBridgeNode.mMessageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector3);
        transform.setRotation(quaternion);

        return transform;
    }

    private Transform staticTranslationTransform(float x, float y, float z) {
        org.ros.rosjava_geometry.Quaternion identity_quaternion = org.ros.rosjava_geometry.Quaternion.identity();
        return staticFullTransform(x, y, z, (float)identity_quaternion.getX(), (float)identity_quaternion.getY(), (float)identity_quaternion.getZ(), (float)identity_quaternion.getW());
    }

    private TransformStamped baseLinkToNeckTransform(Time time) {
        TransformStamped transformStamped = mBridgeNode.mMessageFactory.newFromType(TransformStamped._TYPE);

        // Neck is approx 50cm above the base
        transformStamped.setTransform(staticTranslationTransform(0, 0, 0.5f));

        String sourceFrame = Sensor.BASE_ODOM_FRAME;
        String targetFrame = Sensor.NECK_POSE_FRAME;

        if (mBridgeNode.use_tf_prefix) {
            sourceFrame = mBridgeNode.tf_prefix + "_" + sourceFrame;
            targetFrame = mBridgeNode.tf_prefix + "_" + targetFrame;
        }

        transformStamped.getHeader().setFrameId(sourceFrame);
        transformStamped.setChildFrameId(targetFrame);

        // Future-date this static transform
        transformStamped.getHeader().setStamp(time.add(Duration.fromMillis(100)));

        return transformStamped;
    }

    // Added by Nilesh ******************************************************************************
    private Quaternion setRPY(double roll, double pitch, double yaw)
    {
        double halfYaw = yaw * 0.5;
        double halfPitch = pitch * 0.5;
        double halfRoll = roll * 0.5;
        double cosYaw = Math.cos(halfYaw);
        double sinYaw = Math.sin(halfYaw);
        double cosPitch = Math.cos(halfPitch);
        double sinPitch = Math.sin(halfPitch);
        double cosRoll = Math.cos(halfRoll);
        double sinRoll = Math.sin(halfRoll);
        Quaternion quaternion = mBridgeNode.mMessageFactory.newFromType(Quaternion._TYPE);
        quaternion.setX(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw);
        quaternion.setY(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);
        quaternion.setZ(cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw);
        quaternion.setW(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
        return quaternion;
    }
    private TransformStamped neckToLaserTransform(Time time, float pitch_val) {
        TransformStamped transformStamped = mBridgeNode.mMessageFactory.newFromType(TransformStamped._TYPE);

        // Neck to laser
        Vector3 vector3 = mBridgeNode.mMessageFactory.newFromType(Vector3._TYPE);
        vector3.setX(0.0f);
        vector3.setY(0.0f);
        vector3.setZ(0.0f);

        double roll = 0.0;
        double pitch = -pitch_val*2.0;
        double yaw = 0.0;
        Quaternion quaternion = setRPY(roll, pitch, yaw);

        Transform transform = mBridgeNode.mMessageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector3);
        transform.setRotation(quaternion);

        transformStamped.setTransform(transform);

        String sourceFrame = "rsdepth_center_neck_fix_frame";
        String targetFrame = "stabilized_laser";

        if (mBridgeNode.use_tf_prefix) {
            sourceFrame = mBridgeNode.tf_prefix + "_" + sourceFrame;
            targetFrame = mBridgeNode.tf_prefix + "_" + targetFrame;
        }

        transformStamped.getHeader().setFrameId(sourceFrame);
        transformStamped.setChildFrameId(targetFrame);

        // Future-date this static transform
        transformStamped.getHeader().setStamp(time.add(Duration.fromMillis(100)));

        return transformStamped;
    }
    //***********************************************************************************************

    private TransformStamped baseLinkToUltrasonicTransform(Time time) {
        Vector3 vector3 = mBridgeNode.mMessageFactory.newFromType(Vector3._TYPE);

        // 44cm = 0.44m
        // 12cm = 0.12m
        vector3.setX(0.12f);
        vector3.setY(0.0f);
        vector3.setZ(0.44f);

        Quaternion quaternion = mBridgeNode.mMessageFactory.newFromType(Quaternion._TYPE);
        org.ros.rosjava_geometry.Quaternion identity_quaternion = org.ros.rosjava_geometry.Quaternion.identity();
        quaternion.setX(identity_quaternion.getX());
        quaternion.setY(identity_quaternion.getY());
        quaternion.setZ(identity_quaternion.getZ());
        quaternion.setW(identity_quaternion.getW());

        Transform transform = mBridgeNode.mMessageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector3);
        transform.setRotation(quaternion);
        TransformStamped transformStamped = mBridgeNode.mMessageFactory.newFromType(TransformStamped._TYPE);
        transformStamped.setTransform(transform);
        transformStamped.setChildFrameId(mBridgeNode.UltrasonicFrame);

        String sourceFrame = "base_link";
        if (mBridgeNode.use_tf_prefix) {
            sourceFrame = mBridgeNode.tf_prefix + "_" + sourceFrame;
        }

        transformStamped.getHeader().setFrameId(sourceFrame);
        // Future-date this static transform so that depth_image_proc/register can get the TF easier
        transformStamped.getHeader().setStamp(time.add(Duration.fromMillis(100)));
        return transformStamped;
    }

    private TransformStamped algoTf2TfStamped(AlgoTfData tfData, Time time) {
        Vector3 vector3 = mBridgeNode.mMessageFactory.newFromType(Vector3._TYPE);
        vector3.setX(tfData.t.x);
        vector3.setY(tfData.t.y);
        vector3.setZ(tfData.t.z);
        Quaternion quaternion = mBridgeNode.mMessageFactory.newFromType(Quaternion._TYPE);
        quaternion.setX(tfData.q.x);
        quaternion.setY(tfData.q.y);
        quaternion.setZ(tfData.q.z);
        quaternion.setW(tfData.q.w);
        Transform transform = mBridgeNode.mMessageFactory.newFromType(Transform._TYPE);
        transform.setTranslation(vector3);
        transform.setRotation(quaternion);
        TransformStamped transformStamped = mBridgeNode.mMessageFactory.newFromType(TransformStamped._TYPE);
        transformStamped.setTransform(transform);
        transformStamped.setChildFrameId(tfData.tgtFrameID);
        transformStamped.getHeader().setFrameId(tfData.srcFrameID);
        transformStamped.getHeader().setStamp(time);
        return transformStamped;
    }

    private Odometry produceOdometryMessage(AlgoTfData tfData, LinearVelocity linearVelocity, AngularVelocity angularVelocity, Time time) {
        // Start assembling a nav_msg/Odometry
        Odometry odom_message = mBridgeNode.mOdometryPubr.newMessage();

        odom_message.getPose().getPose().getPosition().setX(tfData.t.x);
        odom_message.getPose().getPose().getPosition().setY(tfData.t.y);
        odom_message.getPose().getPose().getPosition().setZ(tfData.t.z);

        odom_message.getPose().getPose().getOrientation().setX(tfData.q.x);
        odom_message.getPose().getPose().getOrientation().setY(tfData.q.y);
        odom_message.getPose().getPose().getOrientation().setZ(tfData.q.z);
        odom_message.getPose().getPose().getOrientation().setW(tfData.q.w);

        // TODO: make sure it's fine to leave Y and Z uninitialized
        // Segway can only travel in the X direction
        odom_message.getTwist().getTwist().getLinear().setX(linearVelocity.getSpeed());

        // TODO: make sure it's fine to leave X and Y uninitialized
        // Segway can only rotate around the Z axis
        odom_message.getTwist().getTwist().getAngular().setZ(angularVelocity.getSpeed());

        // Child frame is the frame of the twist: base link
        // Add tf_prefix to each transform before ROS publishing (in case of multiple loomos on one network)
        if (mBridgeNode.use_tf_prefix) {
            odom_message.setChildFrameId(mBridgeNode.tf_prefix + "_" + "base_link");
        } else {
            odom_message.setChildFrameId("base_link");
        }

        // parent frame is the odometry frame
        // Add tf_prefix to each transform before ROS publishing (in case of multiple loomos on one network)
        if (mBridgeNode.use_tf_prefix) {
            odom_message.getHeader().setFrameId(mBridgeNode.tf_prefix + "_" + "odom");
        } else {
            odom_message.getHeader().setFrameId("odom");
        }

        odom_message.getHeader().setStamp(time);

        return odom_message;
    }

    private class TFPublisherThread extends Thread {
        @Override
        public void run() {
            Log.d(TAG, "run: SensorPublisherThread");
            super.run();

            // New TF tree:
            // Sensor.WORLD_ODOM_ORIGIN
            //  | Sensor.BASE_POSE_FRAME                <- base_link, the co-ordinate frame relative to the robot platform
            //     | Sensor.NECK_POSE_FRAME             <- neck_link, the co-ordinate frame relative to the neck attached to the base, not moving
            //        | Sensor.HEAD_POSE_Y_FRAME        <- neck_link_yaw, the co-ordinate frame of the head. O translation, but incorporates the yaw of the head
            //           | Sensor.RS_COLOR_FRAME
            //           | Sensor.RS_DEPTH_FRAME
            //           | Sensor.HEAD_POSE_P_R_FRAME
            //              | Sensor.PLATFORM_CAM_FRAME

            final List<String> frameNames = Arrays.asList(
                    Sensor.WORLD_ODOM_ORIGIN,   // 0, odom
                    Sensor.BASE_POSE_FRAME,     // 1, base_link
                    Sensor.NECK_POSE_FRAME,     // 2, neck_link
                    Sensor.HEAD_POSE_Y_FRAME,   // 3, head_link
                    Sensor.RS_COLOR_FRAME,      // 4, rs_color
                    Sensor.RS_DEPTH_FRAME,      // 5, rs_depth
                    Sensor.HEAD_POSE_P_R_FRAME, // 6, tablet_link
                    Sensor.PLATFORM_CAM_FRAME); // 7, plat_cam_link

            final List<Pair<Integer, Integer>> frameIndices = Arrays.asList(
                    new Pair<>(0, 1), // odom to base_link
                    new Pair<>(1, 2), // base_link to neck_link
                    new Pair<>(2, 3), // neck_link to head_link
                    new Pair<>(3, 4), // head_link to rs_color
                    new Pair<>(3, 5), // head_link to rs_depth
                    new Pair<>(3, 6), // head_link to tablet_link
                    new Pair<>(6, 7));// tablet_link to plat_cam_link

            while (null != mSensor) {

                if (mDepthRosStamps == null) {
                    continue;
                }

                Pair<Long, Time> stamp = mDepthRosStamps.poll();
                if (null != stamp) {

                    // Get an appropriate ROS time to match the platform time of this stamp
                    /*
                    Time currentRosTime = mBridgeNode.mConnectedNode.getCurrentTime();
                    Time currentSystemTime = Time.fromMillis(System.currentTimeMillis());
                    Time stampTime = Time.fromNano(Utils.platformStampInNano(stamp));
                    Duration rosToSystemTimeOffset = currentRosTime.subtract(currentSystemTime);
                    Time correctedStampTime = stampTime.add(rosToSystemTimeOffset);

                    Log.d(TAG, "node: " + currentRosTime.toString());
                    Log.d(TAG, "sys: " + currentSystemTime.toString());
                    Log.d(TAG, "node-sys diff: " + rosToSystemTimeOffset.toString());

                    Log.d(TAG, "node: " + currentRosTime.toString());
                    Log.d(TAG, "stamp: " + stampTime.toString());
                    Log.d(TAG, "node-stamp diff: " + (currentRosTime.subtract(stampTime)).toString());
                    Log.d(TAG, "True stamp: " + correctedStampTime.toString());
                    Log.d(TAG, "True node-stamp diff: " + (currentRosTime.subtract(correctedStampTime)).toString());
                    */

                    TFMessage tfMessage = mBridgeNode.mTfPubr.newMessage();
                    for (Pair<Integer, Integer> index : frameIndices) {
                        String target = frameNames.get(index.second);
                        String source = frameNames.get(index.first);

                        // Swapped source/target because it seemed backwards in RViz
                        AlgoTfData tfData = mSensor.getTfData(target, source, stamp.first, 100);

                        //Log.d(TAG, tfData.toString());

                        // ROS usually uses "base_link" and "odom" as fundamental tf names
                        // definitely could remove this if you prefer Loomo's names
                        if (source.equals(Sensor.BASE_POSE_FRAME)) {
                            source = "base_link";
                        }
                        if (target.equals(Sensor.BASE_POSE_FRAME)) {
                            target = "base_link";
                        }
                        if (source.equals(Sensor.WORLD_ODOM_ORIGIN)) {
                            source = "odom";
                        }
                        if (target.equals(Sensor.WORLD_ODOM_ORIGIN)) {
                            target = "odom";
                        }

                        // Add tf_prefix to each transform before ROS publishing (in case of multiple loomos on one network)
                        if (mBridgeNode.use_tf_prefix) {
                            tfData.srcFrameID = mBridgeNode.tf_prefix + "_" + source;
                            tfData.tgtFrameID = mBridgeNode.tf_prefix + "_" + target;
                        }

                        if (stamp.first != tfData.timeStamp) {
                            Log.d(TAG, String.format("ERROR: getTfData failed for frames[%d]: %s -> %s",
                                    stamp.first, source, target));
                            continue;
                        }
                        TransformStamped transformStamped = algoTf2TfStamped(tfData, stamp.second);
                        tfMessage.getTransforms().add(transformStamped);
                    }

                    // Publish the Sensor.RS_COLOR_FRAME -> RsOpticalFrame transform
                    TransformStamped loomoToRsCameraTf = realsenseColorToOpticalFrame(stamp.second);
                    tfMessage.getTransforms().add(loomoToRsCameraTf);

                    // Publish the RsOpticalFrame -> RsDepthFrame transform
                    // TODO: compute statically and just update the timestamp
                    TransformStamped rsColorToDepthTf = realsenseColorToDepthExtrinsic(mDepthCalibration.depthToColorExtrinsic, stamp.second);
                    tfMessage.getTransforms().add(rsColorToDepthTf);

                    // Publish the base_link -> ultrasonic frame transform
                    // Ultrasonic is static TF, 44cm up, 12cm forward
                    TransformStamped ultrasonicTf = baseLinkToUltrasonicTransform(stamp.second);
                    tfMessage.getTransforms().add(ultrasonicTf);

                    // Added Nilesh *****************************************************************
                    SensorData mBaseImu = mSensor.querySensorData(Arrays.asList(Sensor.BASE_IMU)).get(0);
                    float mBasePitch = mBaseImu.getFloatData()[0];
                    //Log.d(TAG, "Base_pitch:" + mBasePitch);
                    TransformStamped laserTf = neckToLaserTransform(stamp.second, mBasePitch);
                    tfMessage.getTransforms().add(laserTf);
                    // ******************************************************************************

                    if (tfMessage.getTransforms().size() > 0) {
                        mBridgeNode.mTfPubr.publish(tfMessage);
                    }

                    // Swapped source/target because it seemed backwards in RViz
                    // TODO: this isn't capturing the velocity at the sensor timestamp. Consider moving to another thread to publish this as fast as possible
                    AlgoTfData tfData = mSensor.getTfData(Sensor.BASE_POSE_FRAME, Sensor.WORLD_ODOM_ORIGIN, stamp.first, 100);
                    LinearVelocity linearVelocity = mBase.getLinearVelocity();
                    AngularVelocity angularVelocity = mBase.getAngularVelocity();
                    Odometry odom_message = produceOdometryMessage(tfData, linearVelocity, angularVelocity, stamp.second);
                    mBridgeNode.mOdometryPubr.publish(odom_message);
                }
            }
        }
    }
}
