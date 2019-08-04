package org.ros.android.android_loomo_ros;

import android.os.Handler;
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
import std_msgs.Header;
import tf2_msgs.TFMessage;

/**
 * Created by mfe on 7/17/18.
 */

public class TFPublisher {
    public static final String TAG = "TFPublisher";

    public boolean mIsPubTF;
    private Thread mTFPublishThread;

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
        if (mTFPublishThread == null) {
            mTFPublishThread = new TFPublisherThread();
        }
        Handler handler=new Handler();
        Runnable r=new Runnable() {
            public void run() {
                //what ever you do here will be done after 5 seconds delay.
                Log.d(TAG, "Waited for ROS publisher to connect. Going to start publishing TF data now.");
                mTFPublishThread.start();
            }
        };
        handler.postDelayed(r, 5000);

    }

    public void stop_tf() {
        Log.d(TAG, "stop_tf()");
        try {
            mTFPublishThread.join();
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


    private TransformStamped algoTf2TfStamped(AlgoTfData tfData, Long stamp) {
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
//        Log.d(TAG, "node: " + mBridgeNode.mConnectedNode.getCurrentTime().toString());
//        Log.d(TAG, "system: " + Time.fromMillis(Utils.platformStampInMillis(stamp)).toString());
//        Log.d(TAG, "diff: " + (mBridgeNode.mConnectedNode.getCurrentTime().subtract(Time.fromMillis(Utils.platformStampInMillis(stamp)))).toString());
        transformStamped.getHeader().setStamp(mBridgeNode.mConnectedNode.getCurrentTime());
        return transformStamped;
    }

    private class TFPublisherThread extends Thread {
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

                        // Swapped source/target because it seemed backwards in RViz
                        AlgoTfData tfData = mSensor.getTfData(target, source, stamp, 500);

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

//                        if (stamp != tfData.timeStamp) {
//                            Log.d(TAG, String.format("run: getTfData failed for frames[%d]: %s -> %s",
//                                    stamp, source, target));
//                            continue;
//                        }
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
