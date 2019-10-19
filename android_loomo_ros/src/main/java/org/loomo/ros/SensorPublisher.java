package org.loomo.ros;

import android.util.Log;

import com.segway.robot.sdk.perception.sensor.InfraredData;
import com.segway.robot.sdk.perception.sensor.Sensor;
import com.segway.robot.sdk.perception.sensor.SensorData;

import org.ros.message.Duration;
import org.ros.message.Time;

import java.util.Arrays;

import sensor_msgs.Range;
import std_msgs.Float32;

/**
 * Created by mfe on 8/3/18.
 */

public class SensorPublisher implements LoomoRosBridgeConsumer {
    private static final String TAG = "SensorPublisher";

    public boolean mIsStarted = false;
    private Sensor mSensor = null;
    private LoomoRosBridgeNode mBridgeNode = null;
    private Thread mSensorPublishThread;

    public SensorPublisher() {
    }

    public void loomo_started(Sensor mSensor) {
        this.mSensor = mSensor;
    }

    @Override
    public void node_started(LoomoRosBridgeNode mBridgeNode) {
        this.mBridgeNode = mBridgeNode;
    }

    @Override
    public void start() {
        if (mSensor == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        } else if (mIsStarted) {
            Log.d(TAG, "already started");
            return;
        }

        mIsStarted = true;

        Log.d(TAG, "start_sensor()");
        if (mSensorPublishThread == null) {
            mSensorPublishThread = new SensorPublisherThread();
        }
        mSensorPublishThread.start();
    }

    @Override
    public void stop() {
        if (!mIsStarted) {
            Log.d(TAG, "not started");
            return;
        }

        Log.d(TAG, "stop_sensor()");
        try {
            mSensorPublishThread.join();
        } catch (InterruptedException e) {
            Log.w(TAG, "onUnbind: mSensorPublishThread.join() ", e);
        }

        mIsStarted = false;
    }

    private class SensorPublisherThread extends Thread {
        @Override
        public void run() {
            Log.d(TAG, "run: SensorPublisherThread");
            super.run();

            while (null != mSensor) {
                // No metadata for this frame yet
                // Get an appropriate ROS time to match the platform time of this stamp
                Time currentRosTime = mBridgeNode.mConnectedNode.getCurrentTime();
                Time currentSystemTime = Time.fromMillis(System.currentTimeMillis());
                Duration rosToSystemTimeOffset = currentRosTime.subtract(currentSystemTime);

                if (mBridgeNode.should_pub_ultrasonic) {
                    SensorData mUltrasonicData = mSensor.querySensorData(Arrays.asList(Sensor.ULTRASONIC_BODY)).get(0);
                    float mUltrasonicDistance = mUltrasonicData.getIntData()[0];
                    Range ultrasonicMessage = mBridgeNode.mUltrasonicPubr.newMessage();

                    Time stampTime = Time.fromNano(Utils.platformStampInNano(mUltrasonicData.getTimestamp()));
                    Time correctedStampTime = stampTime.add(rosToSystemTimeOffset);

                    ultrasonicMessage.getHeader().setFrameId(mBridgeNode.UltrasonicFrame);
                    ultrasonicMessage.getHeader().setStamp(correctedStampTime);

                    // Ultrasonic sensor FOV is 40 degrees
                    ultrasonicMessage.setFieldOfView((float)(40.0f * (Math.PI/180.0f)));
                    ultrasonicMessage.setRadiationType(Range.ULTRASOUND);
                    ultrasonicMessage.setMinRange(0.250f);  // Min range is 250mm
                    ultrasonicMessage.setMaxRange(1.5f);    // Max range is 1500m

                    // Clamp ultrasonic data to max range (ROS requires this)
                    if (mUltrasonicDistance > 1500) {
                        mUltrasonicDistance = 1500;
                    }

                    ultrasonicMessage.setRange(mUltrasonicDistance/1000.0f); // Loomo API provides data in mm
                    mBridgeNode.mUltrasonicPubr.publish(ultrasonicMessage);
                }
                if (mBridgeNode.should_pub_infrared) {
                    InfraredData infraredData = mSensor.getInfraredDistance();

                    float mInfraredDistanceLeft = infraredData.getLeftDistance();
                    float mInfraredDistanceRight = infraredData.getRightDistance();

                    Range infraredMessageLeft = mBridgeNode.mInfraredPubrLeft.newMessage();
                    Range infraredMessageRight = mBridgeNode.mInfraredPubrRight.newMessage();

                    Time stampTime = Time.fromNano(Utils.platformStampInNano(infraredData.getTimestamp()));
                    Time correctedStampTime = stampTime.add(rosToSystemTimeOffset);

                    infraredMessageLeft.getHeader().setStamp(correctedStampTime);
                    infraredMessageRight.getHeader().setStamp(correctedStampTime);

                    infraredMessageLeft.getHeader().setFrameId(mBridgeNode.LeftInfraredFrame);
                    infraredMessageRight.getHeader().setFrameId(mBridgeNode.RightInfraredFrame);

                    // TODO: get real FOV of infrared sensors
                    infraredMessageLeft.setFieldOfView((float)(40.0f * (Math.PI/180.0f)));
                    infraredMessageRight.setFieldOfView((float)(40.0f * (Math.PI/180.0f)));

                    infraredMessageLeft.setRadiationType(Range.INFRARED);
                    infraredMessageRight.setRadiationType(Range.INFRARED);

                    infraredMessageLeft.setRange(mInfraredDistanceLeft/1000.0f);
                    infraredMessageRight.setRange(mInfraredDistanceRight/1000.0f);

                    mBridgeNode.mInfraredPubrLeft.publish(infraredMessageLeft);
                    mBridgeNode.mInfraredPubrRight.publish(infraredMessageRight);

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
