package org.loomo.ros;

import android.util.Log;

import com.segway.robot.sdk.locomotion.head.Head;

public class HeadSubscriber implements LoomoRosBridgeConsumer {
    private static final String TAG = "HeadSubscriber";

    private Head mHead = null;
    private LoomoRosBridgeNode mBridgeNode = null;
    private HeadMotionThread mHeadMotionThread;
    private boolean mIsStarted = false;
    private boolean mThreadRunning = false;

    // The head light is a colored ring of LEDs.
    // Setting the Head Light Mode plays a particular animation on the Head LEDs
    /*
    public enum HeadLightModes {
        OFF, // 0
        BLUE_RING_GREEN_DOT, // 1
        ROTATING_BLUE_RING_GREEN_DOT, // 2
        ROTATING_WHITE_BLUE_RING_GREEN_YELLOW_DOT, // 3
        FADING_WHITE_BLUE_RING_GREEN_YELLOW_DOT, // 4
        RED_RING_5_FLASHES, // 5
        GREEN_RING_5_FLASHES, // 6
        FADING_GREEN_RING, // 7
        FADING_AMBER_RING, // 8
        FADING_BLUE_RING_GREEN_DOT, // 9
        ROTATING_PURPLE_WHITE_RING_AMBER_RED_DOT, // 10
        FADING_PURPLE_WHITE_RING_AMBER_RED_DOT, // 11
        BREATHING_BLUE_RING_GREEN_DOT, // 12
        SPINNING_WHITE_RING_YELLOW_DOT, // 13
    };
    */

    public HeadSubscriber() {
    }

    @Override
    public void node_started(LoomoRosBridgeNode mBridgeNode) {
        this.mBridgeNode = mBridgeNode;
    }

    public void loomo_started(Head mHead) {
        this.mHead = mHead;
    }

    @Override
    public void start() {
        if (mHead == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        } else if (mIsStarted) {
            Log.d(TAG, "already started");
            return;
        }

        mIsStarted = true;

        // TODO: connect to the head controller
        mHead.setMode(Head.MODE_SMOOTH_TACKING);
        mHead.setHeadJointYaw(0.0f);

        Log.d(TAG, "start_sensor()");
        if (mHeadMotionThread == null) {
            mHeadMotionThread = new HeadMotionThread();

            mThreadRunning = true; // Allow the thread to execute
            mHeadMotionThread.start();
        }

        mHead.setHeadLightMode(8);
    }

    @Override
    public void stop() {
        if (!mIsStarted) {
            Log.d(TAG, "not started");
            return;
        }

        Log.d(TAG, "stop_head()");
        try {
            if (mThreadRunning) {
                mThreadRunning = false;
                mHeadMotionThread.interrupt();
                mHeadMotionThread.join();
                mHeadMotionThread = null;
            }
        } catch (InterruptedException e) {
            Log.w(TAG, "onUnbind: mHeadMotionThread.join() ", e);
        }

        mIsStarted = false;
    }

    private class HeadMotionThread extends Thread {
        @Override
        public void run() {
            while(mHead != null && mThreadRunning) {
                // Set head pose to 0 degrees
                mHead.setHeadJointYaw((float) (0.0f));
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    if (!mThreadRunning) {
                        return;
                    }
                }
//                // Set head pose to 45 degrees, left
//                mHead.setHeadJointYaw((float) (30.0f * (Math.PI / 180.0f)));
//                try {
//                    Thread.sleep(1500);
//                } catch (InterruptedException e) {
//                    if (!mThreadRunning) {
//                        return;
//                    }
//                }
//                // Set head pose to 45 degrees, right
//                mHead.setHeadJointYaw((float) (-30.0f * (Math.PI / 180.0f)));
//                try {
//                    Thread.sleep(1500);
//                } catch (InterruptedException e) {
//                    if(!mThreadRunning) {
//                        return;
//                    }
//                }
            }
        }
    }
}
