package org.ros.android.android_loomo_ros;

import android.graphics.Bitmap;
import android.util.Log;

import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.vision.calibration.Intrinsic;
import com.segway.robot.sdk.vision.frame.Frame;
import com.segway.robot.sdk.vision.stream.StreamInfo;
import com.segway.robot.sdk.vision.stream.StreamType;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.node.topic.Publisher;

import java.io.IOException;
import java.nio.channels.Channels;
import java.nio.channels.WritableByteChannel;
import java.util.concurrent.ConcurrentLinkedDeque;

import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import java.util.Queue;


/**
 * Created by mfe on 7/17/18.
 */

public class RealsensePublisher {
    public static final String TAG = "RealsensePublisher";

    private Vision mVision;
    private LoomoRosBridgeNode mBridgeNode;

    public String RsDepthOpticalFrame = "rs_depth_optical_frame";
    public String RsColorOpticalFrame = "rs_color_optical_frame";
    public String FisheyeOpticalFrame = "fisheye_optical_frame";

    private Intrinsic mRsColorIntrinsic, mRsDepthIntrinsic, mFisheyeIntrinsic;
    private int mRsColorWidth = 640;
    private int mRsColorHeight = 480;
    private int mRsDepthWidth = 320;
    private int mRsDepthHeight = 240;
    private int mFisheyeWidth = 640;
    private int mFisheyeHeight = 480;

    private ChannelBufferOutputStream mRsColorOutStream, mRsDepthOutStream, mFisheyeOutStream;
    public Queue<Long> mDepthStamps;
    private Bitmap mRsColorBitmap, mFisheyeBitmap;


    boolean mIsPubRsColor, mIsPubRsDepth, mIsPubFisheye;

    public RealsensePublisher(Vision mVision, LoomoRosBridgeNode mBridgeNode, Queue<Long> mDepthStamps) {
        this.mVision = mVision;
        this.mBridgeNode = mBridgeNode;

        if (mBridgeNode.use_tf_prefix){
            RsDepthOpticalFrame = mBridgeNode.tf_prefix + "_" + RsDepthOpticalFrame;
            RsColorOpticalFrame = mBridgeNode.tf_prefix + "_" + RsColorOpticalFrame;
            FisheyeOpticalFrame = mBridgeNode.tf_prefix + "_" + FisheyeOpticalFrame;
        }

        this.mDepthStamps = mDepthStamps;
        mRsColorOutStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        mRsDepthOutStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        mFisheyeOutStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    public synchronized void start_all() {
        Log.d(TAG, "start_all() called");
        StreamInfo[] infos = mVision.getActivatedStreamInfo();
        for(StreamInfo info : infos) {
            switch (info.getStreamType()) {
                case StreamType.COLOR:
                    updateCameraInfo(2, mVision.getColorDepthCalibrationData().colorIntrinsic,
                            info.getWidth(), info.getHeight());
                    mVision.startListenFrame(StreamType.COLOR, mRsColorListener);
                    break;
                case StreamType.DEPTH:
                    updateCameraInfo(3, mVision.getColorDepthCalibrationData().depthIntrinsic,
                            info.getWidth(), info.getHeight());
                    mVision.startListenFrame(StreamType.DEPTH, mRsDepthListener);
                    break;
            }
        }
        Log.w(TAG, "start_all() done.");
    }

    public synchronized void stop_all() {
        Log.d(TAG, "stop_all() called");
        StreamInfo[] streamInfos = mVision.getActivatedStreamInfo();
        for (StreamInfo info : streamInfos) {
            switch (info.getStreamType()) {
                case StreamType.COLOR:
                    // Stop color listener
                    mVision.stopListenFrame(StreamType.COLOR);
                    break;
                case StreamType.DEPTH:
                    // Stop depth listener
                    mVision.stopListenFrame(StreamType.DEPTH);
                    break;
            }
        }
    }

    public synchronized void start_color(){
        Log.d(TAG, "start_color() called");
        updateCameraInfo(2, mVision.getColorDepthCalibrationData().colorIntrinsic,
                mRsColorWidth, mRsColorHeight);
        mVision.startListenFrame(StreamType.COLOR, mRsColorListener);
    }

    public synchronized void start_depth(){
        Log.d(TAG, "start_depth() called");
        updateCameraInfo(3, mVision.getColorDepthCalibrationData().depthIntrinsic,
                mRsDepthWidth, mRsDepthHeight);
        mVision.startListenFrame(StreamType.DEPTH, mRsDepthListener);
    }

    public synchronized void start_fisheye() {
        Log.d(TAG, "start_fisheye() called");
//        updateCameraInfo(1, mVision.getColorDepthCalibrationData().colorIntrinsic,
//                mFisheyeWidth, mFisheyeHeight);
        mVision.startListenFrame(StreamType.FISH_EYE, mFisheyeListener);
    }

    public synchronized void stop_color() {
        Log.d(TAG, "stop_color() called");
        mVision.stopListenFrame(StreamType.COLOR);
    }

    public synchronized void stop_depth() {
        Log.d(TAG, "stop_depth() called");
        mVision.stopListenFrame(StreamType.DEPTH);
    }

    public synchronized void stop_fisheye() {
        Log.d(TAG, "stop_fisheye() called");
        mVision.stopListenFrame(StreamType.FISH_EYE);
    }

    Vision.FrameListener mRsColorListener = new Vision.FrameListener() {
        private double lastFrameStamp = 0.d; // in millisecond
        @Override
        public void onNewFrame(int streamType, Frame frame) {
//            Log.d(TAG, "mRsColorListener onNewFrame...");
            if (!mIsPubRsColor) {
                Log.d(TAG, "mRsColorListener: !mIsPubRsColor");
                return;
            }
            if (streamType != StreamType.COLOR) {
                Log.e(TAG, "onNewFrame@mRsColorListener: stream type not COLOR! THIS IS A BUG");
                return;
            }
            if (mRsColorBitmap == null || mRsColorBitmap.getWidth() != mRsColorWidth
                    || mRsColorBitmap.getHeight() != mRsColorHeight) {
                mRsColorBitmap = Bitmap.createBitmap(mRsColorWidth, mRsColorHeight, Bitmap.Config.ARGB_8888);
            }

            mRsColorBitmap.copyPixelsFromBuffer(frame.getByteBuffer()); // copy once

            CompressedImage image = mBridgeNode.mRsColorCompressedPubr.newMessage();
            image.setFormat("jpeg");
//            image.getHeader().setStamp(currentTime);
            image.getHeader().setFrameId(RsColorOpticalFrame);

            mRsColorBitmap.compress(Bitmap.CompressFormat.JPEG, 100, mRsColorOutStream);
            image.setData(mRsColorOutStream.buffer().copy());              // copy twice

            mRsColorOutStream.buffer().clear();

            mBridgeNode.mRsColorCompressedPubr.publish(image);
            publishCameraInfo(2, image.getHeader());
        }
    };

    Vision.FrameListener mRsDepthListener = new Vision.FrameListener() {

        private double lastFrameStamp = 0.d; // in millisecond
        @Override
        public void onNewFrame(int streamType, Frame frame) {
//            Log.d(TAG, "mRsDepthListener onNewFrame...");
            if (!mIsPubRsDepth)
                return;
            if (streamType != StreamType.DEPTH) {
                Log.e(TAG, "onNewFrame@mRsDepthListener: stream type not DEPTH! THIS IS A BUG");
                return;
            }
            mDepthStamps.add(frame.getInfo().getPlatformTimeStamp());

            Image image = mBridgeNode.mRsDepthPubr.newMessage();
            image.setWidth(mRsDepthWidth);
            image.setHeight(mRsDepthHeight);
            image.setStep(mRsDepthWidth * 2);
            image.setEncoding("mono16");

//            image.getHeader().setStamp(currentTime);
            image.getHeader().setFrameId(RsDepthOpticalFrame);

            try {
                WritableByteChannel channel = Channels.newChannel(mRsDepthOutStream);
                channel.write(frame.getByteBuffer());
            } catch (IOException exception) {
                Log.e(TAG, String.format("publishRsDepth: IO Exception[%s]", exception.getMessage()));
                return;
            }
            image.setData(mRsDepthOutStream.buffer().copy());
            mRsDepthOutStream.buffer().clear();

            mBridgeNode.mRsDepthPubr.publish(image);
            publishCameraInfo(3, image.getHeader());
        }
    };

    Vision.FrameListener mFisheyeListener = new Vision.FrameListener() {
        @Override
        public void onNewFrame(int streamType, Frame frame) {
//            Log.d(TAG, "mRsColorListener onNewFrame...");
            if (!mIsPubFisheye) {
                Log.d(TAG, "mFisheyeListener: !mIsPubFisheye");
                return;
            }
            if (streamType != StreamType.FISH_EYE) {
                Log.e(TAG, "onNewFrame@mFisheyeListener: stream type not FISH_EYE! THIS IS A BUG");
                return;
            }
            if (mFisheyeBitmap == null || mFisheyeBitmap.getWidth() != mFisheyeWidth
                    || mFisheyeBitmap.getHeight() != mFisheyeHeight) {
                mFisheyeBitmap = Bitmap.createBitmap(mFisheyeWidth, mFisheyeHeight, Bitmap.Config.ALPHA_8);
            }

            mFisheyeBitmap.copyPixelsFromBuffer(frame.getByteBuffer()); // copy once

            CompressedImage image = mBridgeNode.mFisheyeCompressedPubr.newMessage();
            image.setFormat("jpeg");
//            image.getHeader().setStamp();
            image.getHeader().setFrameId(FisheyeOpticalFrame);

            mFisheyeBitmap.compress(Bitmap.CompressFormat.JPEG, 100, mFisheyeOutStream);
            image.setData(mFisheyeOutStream.buffer().copy());              // copy twice

            mFisheyeOutStream.buffer().clear();

            mBridgeNode.mFisheyeCompressedPubr.publish(image);
//            publishCameraInfo(2, image.getHeader());
        }
    };

    public void updateCameraInfo(int type, Intrinsic ins, int width, int height) {
        if (type == 1) {
            // platform camera intrinsic not supported yet
            Log.w(TAG, "updateCameraInfo: platform camera intrinsic not supported yet!");
        } else if (type == 2) {
            mRsColorIntrinsic = ins;
            mRsColorWidth = width;
            mRsColorHeight = height;
        } else {
            mRsDepthIntrinsic = ins;
            mRsDepthWidth = width;
            mRsDepthHeight = height;
        }
    }

    private synchronized void publishCameraInfo(int type, std_msgs.Header header) {
        Publisher<CameraInfo> pubr;
        CameraInfo info;
        Intrinsic intrinsic;
        int width, height;
        // type: 1 for pcam, 2 for RsColor, 3 for RsDepth
        if (type == 1) {
            // Currently does not have camera info of platform camera
            Log.d(TAG, "publishCameraInfo type==1 -> not implemented.");
            return;
        } else if (type == 2) {
            pubr = mBridgeNode.mRsColorInfoPubr;
            intrinsic = mRsColorIntrinsic;
            width = mRsColorWidth;
            height = mRsColorHeight;
        }
        else {
            pubr = mBridgeNode.mRsDepthInfoPubr;
            intrinsic = mRsDepthIntrinsic;
            width = mRsDepthWidth;
            height = mRsDepthHeight;
        }

        info = pubr.newMessage();
        double[] k = new double[9];

//        # Intrinsic camera matrix for the raw (distorted) images.
//        #     [fx  0 cx]
//        # K = [ 0 fy cy]
//        #     [ 0  0  1]
        k[0] = intrinsic.focalLength.x;
        k[4] = intrinsic.focalLength.y;
        k[2] = intrinsic.principal.x;
        k[5] = intrinsic.principal.y;
        k[8] = 1;

        info.setHeader(header);
        info.setWidth(width);
        info.setHeight(height);
        info.setK(k);

        pubr.publish(info);
    }
}
