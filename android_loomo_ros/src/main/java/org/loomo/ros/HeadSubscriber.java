package org.loomo.ros;

import android.util.Log;

import org.ros.message.MessageListener;
import com.segway.robot.sdk.locomotion.head.Head;

public class HeadSubscriber implements LoomoRosBridgeConsumer {
    private static final String TAG = "HeadSubscriber";

    private Head mHead = null;
    private LoomoRosBridgeNode mBridgeNode = null;
    private HeadMotionThread mHeadMotionThread;
    private boolean mIsStarted = false;

    public HeadSubscriber(){
    }

    @Override
    public void node_started(LoomoRosBridgeNode mBridgeNode)
    {
        this.mBridgeNode = mBridgeNode;
    }

    public void loomo_started(Head mHead)
    {
        this.mHead = mHead;
    }

    @Override
    public void start(){
        if (mHead == null || mBridgeNode == null || mIsStarted)
        {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        mIsStarted = true;

        // TODO: connect to the head controller
        mHead.setMode(Head.MODE_SMOOTH_TACKING);
        mHead.setHeadJointYaw(0.0f);

        Log.d(TAG, "start_sensor()");
        if (mHeadMotionThread == null) {
            mHeadMotionThread = new HeadMotionThread();
        }
        mHeadMotionThread.start();
    }

    @Override
    public void stop(){
        if (!mIsStarted) {
            return;
        }

        Log.d(TAG, "stop_head()");
        try {
            mHeadMotionThread.join();
        } catch (InterruptedException e) {
            Log.w(TAG, "onUnbind: mHeadMotionThread.join() ", e);
        }

        mIsStarted = false;
    }

    private class HeadMotionThread extends Thread {
        @Override
        public void run() {
            while(mHead != null) {
                // Set head pose to 45 degrees, left
                mHead.setHeadJointYaw((float) (30.0f * (Math.PI / 180.0f)));
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // Set head pose to 45 degrees, right
                mHead.setHeadJointYaw((float) (-30.0f * (Math.PI / 180.0f)));
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
