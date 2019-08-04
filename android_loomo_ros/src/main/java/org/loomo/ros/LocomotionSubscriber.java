package org.loomo.ros;

import android.os.AsyncTask;
import android.util.Log;

import com.segway.robot.sdk.locomotion.sbv.Base;

import org.ros.android.RosActivity;
import org.ros.message.MessageListener;

import java.util.concurrent.TimeUnit;

import geometry_msgs.Twist;

import android.os.Handler;

/**
 * Created by mfe on 7/24/18.
 */

public class LocomotionSubscriber implements LoomoRosBridgeConsumer {
    public static final String TAG = "LocomotionSubscriber";

    private Base mBase;
    private LoomoRosBridgeNode mBridgeNode;
    boolean mStarted = false;

    public LocomotionSubscriber(){
    }

    @Override
    public void node_started(LoomoRosBridgeNode mBridgeNode)
    {
        this.mBridgeNode = mBridgeNode;
    }

    public void loomo_started(Base mBase)
    {
        this.mBase = mBase;

        // Configure Base to accept raw linear/angular velocity commands
        this.mBase.setControlMode(Base.CONTROL_MODE_RAW);
    }

    @Override
    public void start(){

        if (mBase == null || mBridgeNode == null || mStarted)
        {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        mStarted = true;

        mBridgeNode.mCmdVelSubr.addMessageListener(cmdVelListener);
    }

    @Override
    public void stop() {
        mBridgeNode.mCmdVelSubr.removeAllMessageListeners();
        mStarted = false;
    }

    MessageListener<Twist> cmdVelListener = new MessageListener<Twist>() {
        @Override
        public void onNewMessage(Twist message) {
            if (mBase == null) {
                return;
            }

            mBase.setLinearVelocity((float)message.getLinear().getX());
            mBase.setAngularVelocity((float)message.getAngular().getZ());
        }
    };

}
