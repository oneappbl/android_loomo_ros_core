package org.loomo.ros;

import android.graphics.Bitmap;
import android.util.Log;
import android.util.Pair;

import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.vision.calibration.Intrinsic;
import com.segway.robot.sdk.vision.frame.Frame;
import com.segway.robot.sdk.vision.frame.FrameInfo;
import com.segway.robot.sdk.vision.imu.IMUDataCallback;
import com.segway.robot.sdk.vision.stream.StreamInfo;
import com.segway.robot.sdk.vision.stream.StreamType;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.topic.Publisher;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.Channels;
import java.nio.channels.WritableByteChannel;
import java.util.Queue;

import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import std_msgs.Header;

import static org.loomo.ros.Utils.platformStampInNano;


/**
 * Created by mfe on 7/17/18.
 */

public class RealsensePublisher implements LoomoRosBridgeConsumer, IMUDataCallback {

    private enum RealsenseMetadataSource {
        DEPTH,
        COLOR,
        FISHEYE
    }

    private class RealsenseMetadata {
        int frameNum;
        Long platformStamp;
        Time rosTime;
        RealsenseMetadataSource source;
    }

    public static final String TAG = "RealsensePublisher";

    private Vision mVision = null;
    private LoomoRosBridgeNode mBridgeNode = null;

    private Intrinsic mRsColorIntrinsic, mRsDepthIntrinsic, mFisheyeIntrinsic;

    private int mRsColorWidth = 640;
    private int mRsColorHeight = 480;
    private int mRsDepthWidth = 320;
    private int mRsDepthHeight = 240;
    private int mFisheyeWidth = 640;
    private int mFisheyeHeight = 480;

    private ChannelBufferOutputStream mRsColorOutStream, mRsDepthOutStream, mFisheyeOutStream;
    private Queue<Long> mDepthStamps;
    private Queue<Pair<Long, Time>> mDepthRosStamps;

    private RealsenseMetadata mRealsenseMeta = null;

    private Bitmap mRsColorBitmap, mFisheyeBitmap;

    public boolean mIsPubRsColor, mIsPubRsDepth, mIsPubFisheye;

    private boolean mColorStarted = false;
    private boolean mDepthStarted = false;
    private boolean mFisheyeStarted = false;

    private Long mLatestDepthStamp = 0L;

    public RealsensePublisher(Queue<Long> mDepthStamps, Queue<Pair<Long, Time>> mDepthRosStamps) {
        this.mDepthStamps = mDepthStamps;
        this.mDepthRosStamps = mDepthRosStamps;

        mRsColorOutStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        mRsDepthOutStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        mFisheyeOutStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    public void node_started(LoomoRosBridgeNode mBridgeNode) {
        this.mBridgeNode = mBridgeNode;
    }

    @Override
    public void start() {
        // No generic initialization is required

        if (mBridgeNode == null || mVision == null) {
            Log.d(TAG, "Cannot start RealsensePublisher, ROS or Loomo SDK is not ready");
            return;
        }
    }

    @Override
    public void stop() {
        // No generic de-initialization is required
        // TODO: really?
    }

    public void loomo_started(Vision mVision) {
        this.mVision = mVision;

        // Get color-depth extrinsic and publish as a TF
    }

    public synchronized void start_all() {
        if (mVision == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

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
        if (mVision == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

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

        mColorStarted = false;
        mDepthStarted = false;
        mFisheyeStarted = false;
    }

    public synchronized void start_imu() {
        if (mVision == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        mVision.setIMUCallback(this);
    }

    public synchronized void start_color() {
        if (mVision == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        } else if (mColorStarted) {
            Log.d(TAG, "already started");
            return;
        }

        mColorStarted = true;

        Log.d(TAG, "start_color() called");
        updateCameraInfo(2, mVision.getColorDepthCalibrationData().colorIntrinsic,
                mRsColorWidth, mRsColorHeight);
        mVision.startListenFrame(StreamType.COLOR, mRsColorListener);
    }

    public synchronized void start_depth() {
        if (mVision == null || mBridgeNode == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        } else if (mDepthStarted) {
            Log.d(TAG, "already started");
            return;
        }

        mDepthStarted = true;

        Log.d(TAG, "start_depth() called");
        updateCameraInfo(3, mVision.getColorDepthCalibrationData().depthIntrinsic,
                mRsDepthWidth, mRsDepthHeight);
        mVision.startListenFrame(StreamType.DEPTH, mRsDepthListener);
    }

    public synchronized void start_fisheye() {
        if (mVision == null) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        mFisheyeStarted = true;

        Log.d(TAG, "start_fisheye() called");
//        updateCameraInfo(1, mVision.getColorDepthCalibrationData().colorIntrinsic,
//                mFisheyeWidth, mFisheyeHeight);
        mVision.startListenFrame(StreamType.FISH_EYE, mFisheyeListener);
    }

    public synchronized void stop_color() {
        if (mVision == null || !mColorStarted) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        Log.d(TAG, "stop_color() called");
        mVision.stopListenFrame(StreamType.COLOR);

        mColorStarted = false;
    }

    public synchronized void stop_depth() {
        if (mVision == null || !mDepthStarted) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        Log.d(TAG, "stop_depth() called");
        mVision.stopListenFrame(StreamType.DEPTH);

        mDepthStarted = false;
    }

    public synchronized void stop_fisheye() {
        if (mVision == null || !mFisheyeStarted) {
            Log.d(TAG, "Cannot start_listening yet, a required service is not ready");
            return;
        }

        Log.d(TAG, "stop_fisheye() called");
        mVision.stopListenFrame(StreamType.FISH_EYE);

        mFisheyeStarted = false;
    }

    private synchronized boolean process_metadata(RealsenseMetadataSource source, FrameInfo frameInfo, Header imageHeader)
    {
        int currentFrame = frameInfo.getFrameNum();
        Long currentPlatformStamp = frameInfo.getPlatformTimeStamp();

        // Is there currently Realsense metadata?
        if (mRealsenseMeta != null) {
            // Did we generate this metadata?
            if (mRealsenseMeta.source == source) {
                // We generated this metadata, so a new frame has arrived before the other provider was able to
                // consume it. Maybe a frame was dropped from the other camera.

                // We should update the metadata here to match our current frame

                if (mRealsenseMeta.frameNum == currentFrame) {
                    Log.d(TAG, "ERROR: Camera callback for " + source + " called twice for same frame!");
                    Log.d(TAG, "Current frame is " + currentFrame + " but metadata contains frame " + mRealsenseMeta);
                    return false;

                    // TODO: this is fatal?
                } else if (mRealsenseMeta.frameNum > currentFrame) {
                    Log.d(TAG, "ERROR: Camera callback for " + source + " called twice, with an old frame!");
                    Log.d(TAG, "Current frame is " + currentFrame + " but metadata contains frame " + mRealsenseMeta);
                    return false;

                    // TODO: this is fatal?
                } else /* if (mRealsenseMeta.frameNum < currentFrame) */ {
                    Log.d(TAG, "WARNING: Camera callback for " + source + " detected stale metadata: other source has fallen behind!");
                    Log.d(TAG, "Asked to process metadata for frame " + currentFrame + " but current metadata is for same source, frame " + mRealsenseMeta.frameNum);
                    // Fall through to the end of the function where we generate new metadata
                    // TODO: is this the right choice? what should we do about this?
                }
            } else {
                // The other camera generated the metadata. We should validate it
                // 3 possibilities:
                // - The metadata matches our frame metadata exactly
                // - Our metadata is newer
                // - Our metadata is older
                if (mRealsenseMeta.frameNum == currentFrame) {
                    // Consume the metadata and clear it
                    // Set the image header
                    imageHeader.setStamp(mRealsenseMeta.rosTime);
                    mRealsenseMeta = null;
                    return true;
                } else if (mRealsenseMeta.frameNum > currentFrame) {
                    // We have an old frame, and should drop it to try and catch up
                    Log.d(TAG, "ERROR: Camera " + source + " has fallen behind. Processing frame num " + currentFrame + " but other source metadata has already processed " + mRealsenseMeta.frameNum);
                    return false;
                } else /* implied: if (mRealsenseMeta.frameNum < currentFrame) */ {
                    // Metadata from the other camera is old. This implies that the current source skipped a frame.
                    Log.d(TAG, "WARNING: Camera " + source + " is ahead. Processing frame num " + currentFrame + " but other source published metadata data for old frame " + mRealsenseMeta.frameNum);
                    // We should create new metadata for the current frame. Fall through.
                    // TODO: is this the right choice?
                }
            }
        }

        // No metadata for this frame yet
        // Get an appropriate ROS time to match the platform time of this stamp
        Time currentRosTime = mBridgeNode.mConnectedNode.getCurrentTime();
        Time currentSystemTime = Time.fromMillis(System.currentTimeMillis());
        Duration rosToSystemTimeOffset = currentRosTime.subtract(currentSystemTime);
        Time stampTime = Time.fromNano(Utils.platformStampInNano(frameInfo.getPlatformTimeStamp()));
        Time correctedStampTime = stampTime.add(rosToSystemTimeOffset);

        // Add the platform stamp / actual ROS time pair to the list of times we want TF data for
        mDepthRosStamps.add(Pair.create(frameInfo.getPlatformTimeStamp(), correctedStampTime));

        // Create Realsense metadata for this frame
        mRealsenseMeta = new RealsenseMetadata();
        mRealsenseMeta.source = source;
        mRealsenseMeta.frameNum = currentFrame;
        mRealsenseMeta.platformStamp = currentPlatformStamp;
        mRealsenseMeta.rosTime = correctedStampTime;

        // Set the image header
        imageHeader.setStamp(correctedStampTime);

        return true;
    }

    Vision.FrameListener mRsColorListener = new Vision.FrameListener() {
        @Override
        public void onNewFrame(int streamType, Frame frame) {
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

            CompressedImage image = mBridgeNode.mRsColorCompressedPubr.newMessage();
            image.setFormat("jpeg");

            //Log.d(TAG, "COLOR FRAME NUM: " + frame.getInfo().getFrameNum());
            //Log.d(TAG, "COLOR FRAME PLATFORM STAMP: " + frame.getInfo().getPlatformTimeStamp());

            // If process_metadata doesn't want us to publish the frame, bail out now
            if(!process_metadata(RealsenseMetadataSource.COLOR, frame.getInfo(), image.getHeader())) {
                Log.d(TAG, "WARNING: Skipping Color Frame " + frame.getInfo().getFrameNum());
                return;
            }

            image.getHeader().setFrameId(mBridgeNode.RsColorOpticalFrame);

            // TODO: no more compression, it's too slow
            mRsColorBitmap.copyPixelsFromBuffer(frame.getByteBuffer()); // copy once
            mRsColorBitmap.compress(Bitmap.CompressFormat.JPEG, 75, mRsColorOutStream);
            image.setData(mRsColorOutStream.buffer().copy());              // copy twice

            mRsColorOutStream.buffer().clear();

            mBridgeNode.mRsColorCompressedPubr.publish(image);
            publishCameraInfo(2, image.getHeader());
        }
    };

    Vision.FrameListener mRsDepthListener = new Vision.FrameListener() {

        @Override
        public void onNewFrame(int streamType, Frame frame) {
            if (!mIsPubRsDepth) {
                return;
            }

            if (streamType != StreamType.DEPTH) {
                Log.e(TAG, "onNewFrame@mRsDepthListener: stream type not DEPTH! THIS IS A BUG");
                return;
            }

            //Log.d(TAG, "DEPTH FRAME NUM: " + frame.getInfo().getFrameNum());
            //Log.d(TAG, "DEPTH FRAME PLATFORM STAMP: " + frame.getInfo().getPlatformTimeStamp());

            Image image = mBridgeNode.mRsDepthPubr.newMessage();
            image.setWidth(mRsDepthWidth);
            image.setHeight(mRsDepthHeight);
            image.setStep(mRsDepthWidth * 2);
            image.setEncoding("16UC1");
            image.getHeader().setFrameId(mBridgeNode.RsDepthOpticalFrame);

            // If process_metadata doesn't want us to publish the frame, bail out now
            if(!process_metadata(RealsenseMetadataSource.DEPTH, frame.getInfo(), image.getHeader())) {
                Log.d(TAG, "WARNING: Skipping Depth Frame " + frame.getInfo().getFrameNum());
                return;
            }

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
            image.getHeader().setStamp(Time.fromNano(platformStampInNano(frame.getInfo().getPlatformTimeStamp())));
            image.getHeader().setFrameId(mBridgeNode.FisheyeOpticalFrame);

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

// # Projection/camera matrix
// #     [fx'  0  cx' Tx]
// # P = [ 0  fy' cy' Ty]
// #     [ 0   0   1   0]
        double[] p = new double[12];

        p[0] = intrinsic.focalLength.x;
        p[5] = intrinsic.focalLength.y;
        p[2] = intrinsic.principal.x;
        p[6] = intrinsic.principal.y;
        p[10] = 1;

// # Rectification matrix (stereo cameras only)
// # A rotation matrix aligning the camera coordinate system to the ideal
// # stereo image plane so that epipolar lines in both stereo images are
// # parallel.
        double[] r = new double[9];
        r[0] = 1;
        r[4] = 1;
        r[8] = 1;

        info.setHeader(header);
        info.setWidth(width);
        info.setHeight(height);
        info.setK(k);
        info.setP(p);
        info.setR(r);

        pubr.publish(info);
    }


    @Override
    public void onNewData(ByteBuffer byteBuffer, int length, int frameCount) {
        // TODO: what is this data
        Log.d(TAG, "Length: " + length);
        Log.d(TAG, "Frame Count: " + frameCount);
    }
}
