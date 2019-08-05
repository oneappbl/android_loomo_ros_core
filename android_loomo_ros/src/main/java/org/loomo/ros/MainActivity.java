/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.loomo.ros;

import android.Manifest;
import android.content.pm.PackageManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationManager;
import android.location.LocationProvider;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.Button;

import androidx.annotation.RequiresApi;
import androidx.core.content.ContextCompat;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.tasks.OnSuccessListener;
import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.perception.sensor.Sensor;
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.locomotion.sbv.Base;
import com.segway.robot.sdk.locomotion.head.Head;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import java.net.URI;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.TimeUnit;


/**
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 * @author mfe@mit.edu (Michael Everett)
 */
public class MainActivity extends RosActivity implements CompoundButton.OnCheckedChangeListener, View.OnClickListener {
    public static final String TAG = "MainRosActivity";

    private Vision mVision;
    private Sensor mSensor;
    private Base mBase;
    private Head mHead;

    private Button mKillAppButton;

    private Switch mPubRsColorSwitch;
    private Switch mPubRsDepthSwitch;
    private Switch mPubFisheyeSwitch;
    private Switch mPubSensorSwitch;
    private Switch mSubMotionSwitch;
    private Switch mSubHeadSwitch;

    private Switch mPubTFSwitch;
    private RealsensePublisher mRealsensePublisher;
    private TFPublisher mTFPublisher;
    private LocomotionSubscriber mLocomotionSubscriber;
    private HeadSubscriber mHeadSubscriber;

    private List<LoomoRosBridgeConsumer> mRosBridgeConsumers;

    private SensorPublisher mSensorPublisher;

    private LoomoRosBridgeNode mBridgeNode;

    private Queue<Pair<Long, Time>> mDepthRosStamps; // Stores a co-ordinated platform time and ROS time to help manage the offset

    private Queue<Long> mDepthStamps;

    private LocationManager mLocationManager;

    // Assumes that ROS master is a different machine, with a hard-coded ROS_MASTER_URI.
    // If you'd like to be able to select the URI in the app on startup, replace
    // super( , , ) with super( , ) to start a different version of RosActivity
    //public MainActivity() { super("LoomoROS", "LoomoROS");}
    public MainActivity() { super("LoomoROS", "LoomoROS", URI.create("http://192.168.42.36:11311/")); }

    @RequiresApi(api = Build.VERSION_CODES.M)
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        // Set up GUI window
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);

        // Add a button to be able to hard-kill this app (not recommended by android but whatever)
        mKillAppButton = (Button) findViewById(R.id.killapp);
        mKillAppButton.setOnClickListener(this);

        // Add some switches to turn on/off sensor publishers
        mPubRsColorSwitch = (Switch) findViewById(R.id.rscolor);
        mPubRsDepthSwitch = (Switch) findViewById(R.id.rsdepth);
        mPubFisheyeSwitch = (Switch) findViewById(R.id.fisheye);
        mPubTFSwitch = (Switch) findViewById(R.id.tf);
        mPubSensorSwitch = (Switch) findViewById(R.id.sensor);
        mSubMotionSwitch = (Switch) findViewById(R.id.motion);
        mSubHeadSwitch = (Switch) findViewById(R.id.head);

        // Add some listeners to the states of the switches
        mPubRsColorSwitch.setOnCheckedChangeListener(this);
        mPubRsDepthSwitch.setOnCheckedChangeListener(this);
        mPubFisheyeSwitch.setOnCheckedChangeListener(this);
        mPubTFSwitch.setOnCheckedChangeListener(this);
        mPubSensorSwitch.setOnCheckedChangeListener(this);
        mSubMotionSwitch.setOnCheckedChangeListener(this);
        mSubHeadSwitch.setOnCheckedChangeListener(this);

        // Keep track of timestamps when images published, so corresponding TFs can be published too
        mDepthStamps = new ConcurrentLinkedDeque<>();
        mDepthRosStamps = new ConcurrentLinkedDeque<>();

        mHeadSubscriber = new HeadSubscriber();
        mLocomotionSubscriber = new LocomotionSubscriber();
        mRealsensePublisher = new RealsensePublisher(mDepthStamps, mDepthRosStamps);
        mTFPublisher = new TFPublisher(mDepthStamps, mDepthRosStamps);
        mSensorPublisher = new SensorPublisher();

        // Add all the ROS consumers to the list so they all get initialized / de initialized together
        mRosBridgeConsumers = Arrays.asList(mHeadSubscriber, mLocomotionSubscriber, mRealsensePublisher, mTFPublisher, mSensorPublisher);

        // Start an instance of the LoomoRosBridgeNode
        mBridgeNode = new LoomoRosBridgeNode(mOnNodeStarted, mOnNodeShutdown);

        // get Vision SDK instance
        mVision = Vision.getInstance();
        mVision.bindService(this, mBindVisionListener);

        // get Sensor SDK instance
        mSensor = Sensor.getInstance();
        mSensor.bindService(this, mBindStateListener);

        // get Locomotion SDK instance
        mBase = Base.getInstance();
        mBase.bindService(this, mBindLocomotionListener);

        // get Head SDK instance
        mHead = Head.getInstance();
        mHead.bindService(this, mBindHeadListener);

        // GPS
       /* if (ContextCompat.checkSelfPermission(getApplicationContext(), Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    Activity#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for Activity#requestPermissions for more details.
            Log.e(TAG, "NO GPS PERMISSION");
            return;
        }
        else
        {
            Log.d(TAG, "GPS PERMISSION ACQUIRED");
        }

        mLocationManager = (LocationManager) getApplicationContext().getSystemService(LOCATION_SERVICE);
        List<String> providers = mLocationManager.getProviders(true);

        if (!providers.contains(LocationManager.GPS_PROVIDER))
        {
            Log.e(TAG, "No GPS provider available");
        }


        Location location = mLocationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);

        if (location != null) {
            Log.d(TAG, location.toString());
        } */
    }

    Runnable mOnNodeStarted = new Runnable() {
        @Override
        public void run() {
            // Node has started, so we can now tell publishers and subscribers that ROS has initialized
            for(LoomoRosBridgeConsumer consumer : mRosBridgeConsumers)
            {
                consumer.node_started(mBridgeNode);

                // Try a call to start listening, this may fail if the Loomo SDK is not started yet (which is fine)
                consumer.start();
            }

            // Special nodes
            // TODO: we should handle this in the generic "start()" function
            mRealsensePublisher.start_color();
            mRealsensePublisher.start_depth();
        }
    };

    // TODO: shutdown consumers correctly
    Runnable mOnNodeShutdown = new Runnable() {
        @Override
        public void run() {

        }
    };

    @Override
    protected void onRestart() {
        super.onRestart();
        Log.d(TAG, "onRestart() called");
//        mPubRsColorSwitch.setChecked(false);
//        mPubRsDepthSwitch.setChecked(false);
//        mPubTFSwitch.setChecked(false);
//        mLocomotionSubscriber.start_listening();
    }

    @Override
    protected void onResume() {
        super.onResume();
//        mPubRsColorSwitch.setChecked(false);
//        mPubRsDepthSwitch.setChecked(false);
//        mPubTFSwitch.setChecked(false);
    }

    @Override
    public void onStop() {
        super.onStop();
        Log.d(TAG, "onStop() called");
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        Log.d(TAG, "init().");
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                        getMasterUri());

        // Note: NTPd on Linux will, by default, not allow NTP queries from the local networks.
        // Add a rule like this to /etc/ntp.conf:
        //
        // restrict 192.168.86.0 mask 255.255.255.0 nomodify notrap nopeer
        //
        // Where the IP address is based on your subnet

        NtpTimeProvider ntpTimeProvider =
                new NtpTimeProvider(InetAddressFactory.newFromHostString(getMasterUri().getHost()),
                        nodeMainExecutor.getScheduledExecutorService());
        try {
            ntpTimeProvider.updateTime();
        }
        catch (Exception e){
            Log.d(TAG, "exception when updating time...");
        }
        Log.d(TAG, "master uri: " + getMasterUri().getHost());
        Log.d(TAG, "ros: " + ntpTimeProvider.getCurrentTime().toString());
        Log.d(TAG, "sys: " + Time.fromMillis(System.currentTimeMillis()));

        ntpTimeProvider.startPeriodicUpdates(1, TimeUnit.MINUTES);
        nodeConfiguration.setTimeProvider(ntpTimeProvider);

        nodeMainExecutor.execute(mBridgeNode, nodeConfiguration);
    }

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        // Someone has clicked a button - handle it here
        switch (buttonView.getId()) {
            case R.id.rscolor:
                mRealsensePublisher.mIsPubRsColor = isChecked;
                if (isChecked) {
                    mRealsensePublisher.start_color();
                } else {
                    mRealsensePublisher.stop_color();
                }
                break;
            case R.id.rsdepth:
                mRealsensePublisher.mIsPubRsDepth = isChecked;
                if (isChecked) {
                    mRealsensePublisher.start_depth();
                } else {
                    mRealsensePublisher.stop_depth();
                }
                break;
            case R.id.fisheye:
                mRealsensePublisher.mIsPubFisheye = isChecked;
                if (isChecked) {
                    mRealsensePublisher.start_fisheye();
                } else {
                    mRealsensePublisher.stop_fisheye();
                }
                break;
            case R.id.tf:
                Log.d(TAG, "TF clicked.");
                mTFPublisher.mIsPubTF = isChecked;
                if (isChecked) {
                    mTFPublisher.start();
                } else {
                    mTFPublisher.stop();
                }
                break;
            case R.id.sensor:
                Log.d(TAG, "Sensor clicked.");
                if (isChecked) {
                    mSensorPublisher.start();
                } else {
                    mSensorPublisher.stop();
                }
                break;
            case R.id.head:
                Log.d(TAG, "Head clicked.");
                if(isChecked) {
                    mHeadSubscriber.start();
                } else {
                    mHeadSubscriber.stop();
                }
            case R.id.motion:
                Log.d(TAG, "Motion clicked");
                if(isChecked) {
                    mLocomotionSubscriber.start();
                } else {
                    mLocomotionSubscriber.stop();
                }

        }
    }

    ServiceBinder.BindStateListener mBindVisionListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.i(TAG, "onBind() mBindVisionListener called");
            mRealsensePublisher.loomo_started(mVision);
            mTFPublisher.loomo_started(mVision);
            Log.d(TAG, "bindVision enabling realsense switches.");

            // All camera switches can now be toggled by the user
            mPubRsColorSwitch.setEnabled(true);
            mPubRsDepthSwitch.setEnabled(true);

            // TODO: Segway does not allow using the fishey by default. Add a test for the special unlock APK.
            mPubFisheyeSwitch.setEnabled(false);

            // Default switch states
            // TODO: move out of here!
            mPubRsColorSwitch.setChecked(true);
            mPubRsDepthSwitch.setChecked(true);
            mPubFisheyeSwitch.setChecked(false);

            mTFPublisher.start();

            if (mPubRsColorSwitch.isChecked()) {
                mRealsensePublisher.start_color();
            }

            if (mPubRsDepthSwitch.isChecked()) {
                mRealsensePublisher.start_depth();
            }

            if (mPubFisheyeSwitch.isChecked()) {
                mRealsensePublisher.start_fisheye();
            }
        }

        @Override
        public void onUnbind(String reason) {
            Log.i(TAG, "onUnbindVision: " + reason);
        }
    };

    ServiceBinder.BindStateListener mBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "onBind() mBindStateListener called");

            mTFPublisher.loomo_started(mSensor);
            mSensorPublisher.loomo_started(mSensor);

            // Enable the TF and Sensor switches in the UI
            mPubTFSwitch.setEnabled(true);
            mPubSensorSwitch.setEnabled(true);

            // Set default state of switches
            // TODO: move this out of here
            mPubTFSwitch.setChecked(true);
            mPubSensorSwitch.setChecked(true);

            // Try a call to start listening, this may fail is ROS is not started yet (which is fine)
            if(mPubTFSwitch.isChecked()) {
                mTFPublisher.start();
            }

            if(mPubSensorSwitch.isChecked()) {
                mSensorPublisher.start();
            }
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "onUnbind() called with: reason = [" + reason + "]");
        }
    };

    ServiceBinder.BindStateListener mBindLocomotionListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "mBindLocomotionListener onBind() called");
            mLocomotionSubscriber.loomo_started(mBase);
            mTFPublisher.loomo_started(mBase);

            // Try a call to start listening, this may fail is ROS is not started yet (which is fine)
            // TODO: check state of checkbox
            if(mSubMotionSwitch.isChecked()) {
                mLocomotionSubscriber.start();
            }

            if (mPubTFSwitch.isChecked()) {
                mTFPublisher.start();
            }
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "onUnbind() called with: reason = [" + reason + "]");
        }
    };

    ServiceBinder.BindStateListener mBindHeadListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            Log.d(TAG, "mBindHeadListener onBind() called");
            mHeadSubscriber.loomo_started(mHead);

            // Try a call to start listening, this may fail is ROS is not started yet (which is fine)
            // TODO: check state of checkbox
            if(mSubHeadSwitch.isChecked()) {
                mHeadSubscriber.start();
            }
        }

        @Override
        public void onUnbind(String reason) {
            Log.d(TAG, "onUnbind() called with: reason = [" + reason + "]");
        }
    };

    @Override
    public void onClick(View view) {
        Log.d(TAG, "recreating main activity to restart ROS");
        this.recreate();
    }

}
