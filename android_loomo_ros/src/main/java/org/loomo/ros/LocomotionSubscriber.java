package org.loomo.ros;

import android.util.Log;

import com.segway.robot.sdk.locomotion.sbv.Base;

import org.ros.message.MessageListener;

import geometry_msgs.Twist;

/**
 * Created by mfe on 7/24/18.
 */

public class LocomotionSubscriber implements LoomoRosBridgeConsumer {
    public static final String TAG = "LocomotionSubscriber";

    private Base mBase;
    private LoomoRosBridgeNode mBridgeNode;
    boolean mStarted = false;

    public LocomotionSubscriber() {
        Log.d(TAG, "constructor");
    }

    @Override
    public void node_started(LoomoRosBridgeNode mBridgeNode) {
        Log.d(TAG, "node_started");
        this.mBridgeNode = mBridgeNode;
    }

    public void loomo_started(Base mBase) {
        Log.d(TAG, "loomo_started");

        this.mBase = mBase;

        // Configure Base to accept raw linear/angular velocity commands
        this.mBase.setControlMode(Base.CONTROL_MODE_RAW);
    }

    @Override
    public void start() {

        if (mBase == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        } else if (mStarted) {
            Log.d(TAG, "already started");
            return;
        }

        mStarted = true;
        Log.d(TAG, "start");

        mBridgeNode.mCmdVelSubr.addMessageListener(cmdVelListener);
    }

    @Override
    public void stop() {
        Log.d(TAG, "stop");

        mBridgeNode.mCmdVelSubr.removeAllMessageListeners();
        mStarted = false;
    }

    MessageListener<Twist> cmdVelListener = new MessageListener<Twist>() {
        @Override
        public void onNewMessage(Twist message) {
            Log.d(TAG, "cmdVelListener");

            if (mBase == null) {
                Log.d(TAG, "cmdVelListener - mBase is null");
                return;
            }

            mBase.setLinearVelocity((float)message.getLinear().getX());
            mBase.setAngularVelocity((float)message.getAngular().getZ());
        }
    };
}
