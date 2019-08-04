package org.ros.android.android_loomo_ros;

import android.os.Handler;
import android.util.Log;

import com.segway.robot.sdk.perception.sensor.Sensor;
import com.segway.robot.sdk.perception.sensor.SensorData;

import java.util.Arrays;

import std_msgs.Float32;

/**
 * Created by mfe on 8/3/18.
 */

public class SensorPublisher {
    public static final String TAG = "SensorPublisher";

    public boolean mIsPubSensor;
    private Sensor mSensor;
    private LoomoRosBridgeNode mBridgeNode;
    private Thread mSensorPublishThread;

    public SensorPublisher(Sensor mSensor, LoomoRosBridgeNode mBridgeNode) {
        this.mSensor = mSensor;
        this.mBridgeNode = mBridgeNode;
    }

    public void start_sensor() {
        Log.d(TAG, "start_sensor()");
        if (mSensorPublishThread == null) {
            mSensorPublishThread = new SensorPublisherThread();
        }
        Handler handler=new Handler();
        Runnable r=new Runnable() {
            public void run() {
                //what ever you do here will be done after 5 seconds delay.
                Log.d(TAG, "Waited for ROS publisher to connect. Going to start publishing sensor data now.");
                mSensorPublishThread.start();
            }
        };
        handler.postDelayed(r, 5000);

    }

    public void stop_sensor() {
        Log.d(TAG, "stop_sensor()");
        try {
            mSensorPublishThread.join();
        } catch (InterruptedException e) {
            Log.w(TAG, "onUnbind: mSensorPublishThread.join() ", e);
        }
    }

    private class SensorPublisherThread extends Thread {
        @Override
        public void run() {
            Log.d(TAG, "run: SensorPublisherThread");
            super.run();

            while (null != mSensor) {
                if (mBridgeNode.should_pub_ultrasonic){
                    SensorData mUltrasonicData = mSensor.querySensorData(Arrays.asList(Sensor.ULTRASONIC_BODY)).get(0);
                    float mUltrasonicDistance = mUltrasonicData.getIntData()[0];
                    Float32 ultrasonicMessage = mBridgeNode.mUltrasonicPubr.newMessage();
                    ultrasonicMessage.setData(mUltrasonicDistance);
                    mBridgeNode.mUltrasonicPubr.publish(ultrasonicMessage);
                }
                if (mBridgeNode.should_pub_infrared) {
                    SensorData mInfraredData = mSensor.querySensorData(Arrays.asList(Sensor.INFRARED_BODY)).get(0);
                    float mInfraredDistanceLeft = mInfraredData.getIntData()[0];
                    float mInfraredDistanceRight = mInfraredData.getIntData()[1];
                    Float32 infraredMessage = mBridgeNode.mInfraredPubr.newMessage();
                    infraredMessage.setData(mInfraredDistanceLeft);
                    mBridgeNode.mInfraredPubr.publish(infraredMessage);
                }
                if (mBridgeNode.should_pub_base_pitch) {
                    SensorData mBaseImu = mSensor.querySensorData(Arrays.asList(Sensor.BASE_IMU)).get(0);
                    float mBasePitch = mBaseImu.getFloatData()[0];
//                    float mBaseRoll = mBaseImu.getFloatData()[1];
//                    float mBaseYaw = mBaseImu.getFloatData()[2];
                    Float32 basePitchMessage = mBridgeNode.mBasePitchPubr.newMessage();
                    basePitchMessage.setData(mBasePitch);
                    mBridgeNode.mBasePitchPubr.publish(basePitchMessage);
                }

            }
        }
    }
}
