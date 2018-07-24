package org.ros.android.android_loomo_ros;

import android.util.Log;
import android.util.Pair;

import java.util.Arrays;
import java.util.List;
import java.util.Queue;

import com.segway.robot.sdk.perception.sensor.Sensor;

import org.ros.message.Time;


import com.segway.robot.algo.tf.AlgoTfData;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import tf2_msgs.TFMessage;

/**
 * Created by mfe on 7/17/18.
 */

public class TFPublisher {
    public static final String TAG = "TFPublisher";

    public boolean mIsPubTF;
    private Thread mSensorPublishThread;

    private Sensor mSensor;
    private LoomoRosBridgeNode mBridgeNode;
    private Queue<Long> mDepthStamps;

    public TFPublisher(Sensor mSensor, LoomoRosBridgeNode mBridgeNode, Queue<Long> mDepthStamps) {
        this.mSensor = mSensor;
        this.mBridgeNode = mBridgeNode;
        this.mDepthStamps= mDepthStamps;
    }

    public void start_tf() {
        Log.d(TAG, "start_tf()");
        if (mSensorPublishThread == null) {
            mSensorPublishThread = new SensorPublisherThread();
        }
        mSensorPublishThread.start();
    }

    public void stop_tf() {
        Log.d(TAG, "stop_tf()");
        try {
            mSensorPublishThread.join();
        } catch (InterruptedException e) {
            Log.w(TAG, "onUnbind: mSensorPublishThread.join() ", e);
        }
    }

    //    ServiceBinder.BindStateListener mSensorBindListener = new ServiceBinder.BindStateListener() {
//        @Override
//        public void onBind() {
//            Log.i(TAG, "onBindSensor: ");
//            mSensor = Sensor.getInstance();
//            mSensorPublishThread.start();
//        }
//
//        @Override
//        public void onUnbind(String reason) {
//            Log.i(TAG, "onUnbindSensor: " + reason);
//            mSensor = null;
//            try {
//                mSensorPublishThread.join();
//            } catch (InterruptedException e) {
//                Log.w(TAG, "onUnbind: mSensorPublishThread.join() ", e);
//            }
//        }
//    };


    private TransformStamped algoTf2TfStamped(AlgoTfData tfData, long stamp) {
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
        transformStamped.getHeader().setStamp(Time.fromMillis(Utils.platformStampInMillis(stamp)));
        return transformStamped;
    }

    private class SensorPublisherThread extends Thread {
        @Override
        public void run() {
            Log.d(TAG, "run: SensorPublisherThread");
            super.run();
            final List<String> frameNames = Arrays.asList(Sensor.WORLD_ODOM_ORIGIN, Sensor.BASE_POSE_FRAME,
                    Sensor.BASE_ODOM_FRAME, Sensor.NECK_POSE_FRAME, Sensor.HEAD_POSE_Y_FRAME,
                    Sensor.RS_COLOR_FRAME, Sensor.RS_DEPTH_FRAME, Sensor.HEAD_POSE_P_R_FRAME,
                    Sensor.PLATFORM_CAM_FRAME);
            final List<Pair<Integer, Integer>> frameIndices = Arrays.asList(new Pair<>(0, 1),
                    new Pair<>(1, 2), new Pair<>(2, 3), new Pair<>(3, 4), new Pair<>(4, 5),
                    new Pair<>(4, 6), new Pair<>(4, 7), new Pair<>(7, 8));

            while (null != mSensor) {
                if (mDepthStamps == null) {
                    continue;
                }
                Long stamp = mDepthStamps.poll();
                if (null != stamp) {
                    TFMessage tfMessage = mBridgeNode.mTfPubr.newMessage();
                    for (Pair<Integer, Integer> index : frameIndices) {
                        String target = frameNames.get(index.second);
                        String source = frameNames.get(index.first);
                        AlgoTfData tfData = mSensor.getTfData(source, target, stamp, 500);
                        if (stamp != tfData.timeStamp) {
                            Log.d(TAG, String.format("run: getTfData failed for frames[%d]: %s -> %s",
                                    stamp, source, target));
                            continue;
                        }
                        TransformStamped transformStamped = algoTf2TfStamped(tfData, stamp);
                        tfMessage.getTransforms().add(transformStamped);
                    }
                    if (tfMessage.getTransforms().size() > 0)
                        mBridgeNode.mTfPubr.publish(tfMessage);
                }
            }
        }
    }
}