package com.carassistant.ui.activities;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.location.Location;
import android.media.ImageReader.OnImageAvailableListener;
import android.net.Uri;
import android.os.Bundle;
import android.os.SystemClock;
import android.text.SpannableString;
import android.text.style.RelativeSizeSpan;
import android.util.Log;
import android.util.Size;
import android.util.TypedValue;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.widget.SwitchCompat;
import androidx.core.content.ContextCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.carassistant.R;
import com.carassistant.di.components.DaggerScreenComponent;
import com.carassistant.managers.SharedPreferencesManager;
import com.carassistant.model.bus.MessageEventBus;
import com.carassistant.model.bus.model.EventGpsDisabled;
import com.carassistant.model.bus.model.EventUpdateLocation;
import com.carassistant.model.bus.model.EventUpdateStatus;
import com.carassistant.model.entity.Data;
import com.carassistant.model.entity.GpsStatusEntity;
import com.carassistant.model.entity.SignEntity;
import com.carassistant.tflite.classification.LaneTracking;
import com.carassistant.tflite.classification.SpeedLimitClassifier;
import com.carassistant.tflite.detection.Classifier;
import com.carassistant.tflite.detection.TFLiteObjectDetectionAPIModel;
import com.carassistant.tflite.drive.Esp32Driver;
import com.carassistant.tflite.tracking.MultiBoxTracker;
import com.carassistant.ui.adapter.SignAdapter;
import com.carassistant.utils.customview.OverlayView;
import com.carassistant.utils.env.BorderedText;
import com.carassistant.utils.env.ImageUtils;
import com.carassistant.utils.env.Logger;
import com.carassistant.utils.player.MediaPlayerHolder;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import io.reactivex.Observable;
import io.reactivex.android.schedulers.AndroidSchedulers;
import io.reactivex.disposables.CompositeDisposable;

import static com.carassistant.tflite.classification.SpeedLimitClassifier.MODEL_FILENAME;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class DetectorActivity extends CameraActivity implements OnImageAvailableListener {
    private static final Logger LOGGER = new Logger();

    private final String TAG = DetectorActivity.class.getSimpleName();

    // Configuration values for the prepackaged SSD model.
    private static final int TF_OD_API_INPUT_SIZE = 300;
    private static final boolean TF_OD_API_IS_QUANTIZED = true;
    private static final String TF_OD_API_MODEL_FILE = "detect.tflite";
    private static final String TF_OD_API_LABELS_FILE = "file:///android_asset/detect_labelmap.txt";
    //  private static final DetectorMode MODE = DetectorMode.TF_OD_API;
    // Minimum detection confidence to track a detection.
    private static float MINIMUM_CONFIDENCE_TF_OD_API = 0.7f;
    private static final boolean MAINTAIN_ASPECT = false;
    private static final Size DESIRED_PREVIEW_SIZE = new Size(640, 480);
    private static final boolean SAVE_PREVIEW_BITMAP = false;
    private static final float TEXT_SIZE_DIP = 10;
    private float steering_angle_ ;
    OverlayView trackingOverlay;
    private Esp32Driver esp32Driver;
    private Classifier detector;
    private int counterFrme;
    private ArrayList<float[]> points = new ArrayList<>();
    private long lastProcessingTimeMs;
    private Bitmap rgbFrameBitmap = null;
    private Bitmap croppedBitmap = null;
    private Bitmap cropCopyBitmap = null;
    private Mat mRgba, mGray, circles;
    private Mat mRed, mGreen, mBlue, mHue_hsv, mSat_hsv, mVal_hsv, mHue_hls, mSat_hls, mLight_hls;
    private Mat hsv, hls, rgba, gray;
    private Mat mNew, mask, mEdges, laneZoneMat;
    private org.opencv.core.Size ksize = new org.opencv.core.Size(5, 5);
    private double sigma = 3;
    private Point blurPt = new Point(3, 3);
    private Rect signRegion;
    MatOfPoint laneZone;
    private int rows, cols, left, width;
    private double top, middleX, bottomY;
    private boolean computingDetection = false;
    private double vehicleCenterX1, vehicleCenterY1, vehicleCenterX2, vehicleCenterY2, laneCenterX, laneCenterY;

    private long timestamp = 0;

    private Matrix frameToCropTransform;
    private Matrix cropToFrameTransform;

    private MultiBoxTracker tracker;

    //location
    private Data data;
    private boolean firstfix;

    private TextView currentSpeed, distance, satellite, status, accuracy, totalDistance;
    private SignAdapter adapter;

    private SwitchCompat notification, classifier;
    private double distanceValue = 0;
    private CompositeDisposable compositeDisposable;
    private MediaPlayerHolder mediaPlayerHolder;
    private RectF rectF = new RectF();
    SpeedLimitClassifier speedLimitClassifier;
    LaneTracking laneTracking;

    @Inject
    SharedPreferencesManager sharedPreferencesManager;

    private final String SIGN_LIST = "sign_list";
    private final String DISTANCE = "distance";
    private Boolean notificationSpeed = true;
    private CameraBridgeViewBase cameraBridgeViewBase;
    private BaseLoaderCallback baseLoaderCallback;
    @SuppressLint("CheckResult")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        mediaPlayerHolder = new MediaPlayerHolder(this);

        Observable.interval(30L, TimeUnit.SECONDS)
                .timeInterval()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(v -> {
                    notificationSpeed = true;
                });
        OpenCVLoader.initDebug();
        inject();
        setupLocation();
        setupRecycler();
        setupViews();
        setCallBack();
        setupClassifier();
        initializeAllMats();

    }

    private void initializeAllMats() {
        mRgba = new Mat();
        mGray = new Mat();
        mRed = new Mat();
        mGreen = new Mat();
        mBlue = new Mat();
        mHue_hsv = new Mat();
        mSat_hsv = new Mat();
        mVal_hsv = new Mat();
        mHue_hls = new Mat();
        mSat_hls = new Mat();
        mLight_hls = new Mat();
        hsv = new Mat();
        hls = new Mat();
        rgba = new Mat();
        gray = new Mat();
        mNew = new Mat();
        mask = new Mat();
        mEdges = new Mat();
        laneZoneMat = new Mat();
        laneZone = new MatOfPoint();
    }

    private BaseLoaderCallback baseCallback = new BaseLoaderCallback(this) {
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
            switch (status){
                case BaseLoaderCallback.SUCCESS:
                    break;
                default:
                    super.onManagerConnected(status);
                    break;
            }
        }
    };
    private void setupClassifier() {
        try {
            speedLimitClassifier = SpeedLimitClassifier.classifier(getAssets(), MODEL_FILENAME);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    private void setUpLaneTracking() {
        try {
            laneTracking = LaneTracking.classifier(getAssets(), MODEL_FILENAME, this);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void setCallBack() {
        compositeDisposable = new CompositeDisposable();
        compositeDisposable.add(MessageEventBus.INSTANCE
                .toObservable()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(eventModel -> {
                    if (eventModel instanceof EventUpdateLocation) {
                        refresh(((EventUpdateLocation) eventModel).getData());
                    }
                    if (eventModel instanceof EventUpdateStatus) {
                        onGpsStatusChanged(((EventUpdateStatus) eventModel).getStatus());
                    }
                    if (eventModel instanceof EventGpsDisabled) {
                        showGpsDisabledDialog();
                    }
                }));
    }

    @SuppressLint({"ResourceType", "DefaultLocale"})
    private void refresh(Data data) {
        this.data = data;

        double distanceTemp = distanceValue + data.getDistance();

        String distanceUnits;
        if (distanceTemp <= 1000.0) {
            distanceUnits = "m";
        } else {
            distanceTemp /= 1000.0;
            distanceUnits = "km";
        }

        distance.setText(String.format("%.1f %s", distanceTemp, distanceUnits).replace(',', '.'));


        if (distanceValue != data.getDistance()) {
            double distance = sharedPreferencesManager.getDistance();
            distance += (distanceTemp - distanceValue);

            data.setSessionDistanceM(distanceTemp);

            sharedPreferencesManager.setDistance((float) distance);

            distanceValue = distanceTemp;
            showTotalDistance();
        }

        if (data.getLocation().hasAccuracy()) {
            double acc = data.getLocation().getAccuracy();
            String units = "m";

            SpannableString s = new SpannableString(String.format("%.0f %s", acc, units));
            s.setSpan(new RelativeSizeSpan(0.75f), s.length() - units.length() - 1, s.length(), 0);
            accuracy.setText(s);

            if (firstfix) {
                status.setText("");
                firstfix = false;
            }
        } else {
            firstfix = true;
        }

        if (data.getLocation().hasSpeed()) {
            double speed = data.getLocation().getSpeed() * 3.6;
            if (speed > 50 && notification.isChecked() && notificationSpeed) {
                notificationSpeed = false;
                mediaPlayerHolder.loadMedia(R.raw.speed_limit_was_exceeded);
            }
            currentSpeed.setText(String.format("%.0f", speed));
        }
    }

    protected void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);
        outState.putString(SIGN_LIST, new Gson().toJson(adapter.getSigns()));
        outState.putDouble(DISTANCE, distanceValue);
    }
    // initialize all mats


    protected void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        String json = savedInstanceState.getString(SIGN_LIST);
        ArrayList<SignEntity> items = null;
        try {
            items = (new Gson()).fromJson(json, new TypeToken<ArrayList<SignEntity>>() {
            }.getType());
        } catch (Exception ignored) {
            items = new ArrayList<>();
        }
        adapter.setSigns(items);

        distanceValue = savedInstanceState.getDouble(DISTANCE);
    }

    @SuppressLint("DefaultLocale")
    private void setupViews() {
        TextView confidence = findViewById(R.id.confidence_value);
        confidence.setText(String.format("%.2f", MINIMUM_CONFIDENCE_TF_OD_API));

        SwitchCompat camera = findViewById(R.id.camera_switch);
        camera.setOnCheckedChangeListener((buttonView, isChecked) ->
                findViewById(R.id.container).setAlpha(isChecked ? 1f : 0f)
        );

        notification = findViewById(R.id.notification_switch);
        notification.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (!isChecked)
                mediaPlayerHolder.reset();
        });
        classifier = findViewById(R.id.classifier_switch);

        SeekBar confidenceSeekBar = findViewById(R.id.confidence_seek);
        confidenceSeekBar.setMax(100);
        confidenceSeekBar.setProgress((int) (MINIMUM_CONFIDENCE_TF_OD_API * 100));

        confidenceSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                MINIMUM_CONFIDENCE_TF_OD_API = progress / 100.0F;
                confidence.setText(String.format("%.2f", MINIMUM_CONFIDENCE_TF_OD_API));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        findViewById(R.id.privacyPolicy).setOnClickListener(v -> {
            Intent browserIntent = new Intent(Intent.ACTION_VIEW, Uri.parse("https://drive.google.com/open?id=1tKa1WpGUJa8JbU56JfwgOybr3or_Slgx"));
            startActivity(browserIntent);
        });

        showTotalDistance();

    }

    @Override
    public synchronized void onResume() {
        super.onResume();
    }

    @Override
    public synchronized void onPause() {
        super.onPause();
        mediaPlayerHolder.reset();
    }

    @Override
    public void onPreviewSizeChosen(final Size size, final int rotation) {
        final float textSizePx =
                TypedValue.applyDimension(
                        TypedValue.COMPLEX_UNIT_DIP, TEXT_SIZE_DIP, getResources().getDisplayMetrics());
        BorderedText borderedText = new BorderedText(textSizePx);
        borderedText.setTypeface(Typeface.MONOSPACE);

        tracker = new MultiBoxTracker(this);
        if(!OpenCVLoader.initDebug()){
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,  this, baseCallback);
        }
        else{
            baseCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        setUpLaneTracking();

        int cropSize = TF_OD_API_INPUT_SIZE;

        try {
            detector =
                    TFLiteObjectDetectionAPIModel.create(
                            getAssets(),
                            TF_OD_API_MODEL_FILE,
                            TF_OD_API_LABELS_FILE,
                            TF_OD_API_INPUT_SIZE,
                            TF_OD_API_IS_QUANTIZED);
            cropSize = TF_OD_API_INPUT_SIZE;
        } catch (final IOException e) {
            e.printStackTrace();
            LOGGER.e(e, "Exception initializing classifier!");
            Toast toast =
                    Toast.makeText(
                            getApplicationContext(), "Classifier could not be initialized", Toast.LENGTH_SHORT);
            toast.show();
            finish();
        }

        previewWidth = size.getWidth();
        previewHeight = size.getHeight();

        int sensorOrientation = rotation - getScreenOrientation();
        LOGGER.i("Camera orientation relative to screen canvas: %d", sensorOrientation);

        LOGGER.i("Initializing at size %dx%d", previewWidth, previewHeight);
        rgbFrameBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Config.ARGB_8888);
        croppedBitmap = Bitmap.createBitmap(cropSize, cropSize, Config.ARGB_8888);
        frameToCropTransform =
                ImageUtils.getTransformationMatrix(
                        previewWidth, previewHeight,
                        cropSize, cropSize,
                        sensorOrientation, MAINTAIN_ASPECT);

        cropToFrameTransform = new Matrix();
        frameToCropTransform.invert(cropToFrameTransform);
        trackingOverlay = findViewById(R.id.tracking_overlay);
        trackingOverlay.addCallback(canvas -> {
            tracker.draw(canvas);
            if (isDebug()) {
                tracker.drawDebug(canvas);
            }
        });

        tracker.setFrameConfiguration(previewWidth, previewHeight, sensorOrientation);
    }

    @Override
    protected void processImage() {
        ++timestamp;
        final long currTimestamp = timestamp;
        trackingOverlay.postInvalidate();

        // No mutex needed as this method is not reentrant.
        if (computingDetection) {
            readyForNextImage();
            return;
        }
        computingDetection = true;
        LOGGER.i("Preparing image " + currTimestamp + " for detection in bg thread.");

        rgbFrameBitmap.setPixels(getRgbBytes(), 0, previewWidth, 0, 0, previewWidth, previewHeight);

        readyForNextImage();
        Mat rgbFrame = new Mat();
        Utils.bitmapToMat(rgbFrameBitmap, rgbFrame);
        Utils.matToBitmap(rgbFrame, rgbFrameBitmap);
        final Canvas canvas = new Canvas(croppedBitmap);
        canvas.drawBitmap(rgbFrameBitmap, frameToCropTransform, null);
        // For examining the actual TF input.
        if (SAVE_PREVIEW_BITMAP) {
            ImageUtils.saveBitmap(croppedBitmap);
        }

        runInBackground(
                new Runnable() {
                    @Override
                    public void run() {
                        LOGGER.i("Running detection on image " + currTimestamp);
                        final long startTime = SystemClock.uptimeMillis();
                        final List<Classifier.Recognition> results = detector.recognizeImage(croppedBitmap);
                        lastProcessingTimeMs = SystemClock.uptimeMillis() - startTime;;
                        cropCopyBitmap = Bitmap.createBitmap(croppedBitmap);
                        Mat mat = new Mat();
                        Utils.bitmapToMat(cropCopyBitmap, mat);
                        draw_LaneLines(mat);
                        final Canvas canvas = new Canvas(cropCopyBitmap);
                        final Paint paint = new Paint();
                        // paint is used to draw the bounding box around the detected object.
                        // on the canvas.
                        paint.setColor(Color.RED);
                        paint.setStyle(Style.STROKE);
                        paint.setStrokeWidth(2.0f);


                        final List<Classifier.Recognition> mappedRecognitions =
                                new LinkedList<Classifier.Recognition>();

                        for (final Classifier.Recognition result : results) {
                            final RectF location = result.getLocation();
                            if (location != null && result.getConfidence() >= MINIMUM_CONFIDENCE_TF_OD_API) {
                                cropToFrameTransform.mapRect(location);
                                result.setLocation(location);
                                mappedRecognitions.add(result);
                                runOnUiThread(() -> updateSignList(result, croppedBitmap));
                                // draw the lines of hough transform on the canvas
//                                Mat finalMat = mat;

                            }
                        }

                        ArrayList<float[]> mappedRecognitionsPoints = new ArrayList<>();
                        // for loop to draw the lane lines on the canvas
                        for (int i = 0; i < points.size(); i++) {
                            float[] point = points.get(i);
                            cropToFrameTransform.mapPoints(point);
                            mappedRecognitionsPoints.add(point);
                         //   runOnUiThread(draw_LaneLines(mat));
                        }
                        tracker.trackResults(mappedRecognitions, currTimestamp);
                        tracker.trackLaneDetection(points, currTimestamp);
                        trackingOverlay.postInvalidate();
                        mappedRecognitionsPoints.clear();
                        points.clear();
                        computingDetection = false;

                        runOnUiThread(() -> {
                            showFrameInfo(previewWidth + "x" + previewHeight);
                            showCropInfo(cropCopyBitmap.getWidth() + "x" + cropCopyBitmap.getHeight());
                            showInference(lastProcessingTimeMs + "ms");
                        });
                    }


//
//                    private void angle_steering_draw(Mat frame) {
//                        steering_angle_ = get_steering_prediction(frame);
//                        ArrayList <int []> points = draw_LaneLines(frame);
//                        if (steering_angle_ < 0) {
//                            Imgproc.putText(
//                                    displayMat,                          // Matrix obj of the image
//                                    "turn left " + steering_angle_ * -1 + "% of the wheel",          // Text to be added
//                                    new Point(10, 50),               // point
//                                    Core.FONT_HERSHEY_SIMPLEX,      // front face
//                                    1,                               // front scale
//                                    new Scalar(255, 0, 0),             // Scalar object for color
//                                    6                                // Thickness
//                            );
//                        }
//                        if (steering_angle_ > 0) {
//                            Imgproc.putText(
//                                    displayMat,                          // Matrix obj of the image
//                                    "turn right " + steering_angle_ + "% of the wheel",          // Text to be added
//                                    new Point(10, 50),               // point
//                                    Core.FONT_HERSHEY_SIMPLEX,      // front face
//                                    1,                               // front scale
//                                    new Scalar(255, 0, 0),             // Scalar object for color
//                                    6                                // Thickness
//                            );
//                        }
//                    }
                });
    }

    private void updateSignList(Classifier.Recognition result, Bitmap bitmap) {

        SignEntity sign = getSignImage(result, bitmap);

        ArrayList<SignEntity> list = new ArrayList<>(adapter.getSigns());

        if (list.isEmpty()) {
            addSignToAdapter(sign);
            return;
        }
        if (list.contains(sign)) {
            if (isRemoveValid(sign, list.get(list.indexOf(sign)))) {
                adapter.getSigns().remove(sign);
                addSignToAdapter(sign);
            }
        } else {
            addSignToAdapter(sign);
        }

    }

    private void addSignToAdapter(SignEntity sign) {
        adapter.setSign(sign);
        if (notification.isChecked()) {
            mediaPlayerHolder.loadMedia(sign.getSoundNotification());
        }
    }

    private boolean isRemoveValid(SignEntity sign1, SignEntity sign2) {
        return isTimeDifferenceValid(sign1.getDate(), sign2.getDate())
                || isLocationDifferenceValid(sign1.getLocation(), sign2.getLocation());
    }

    private boolean isTimeDifferenceValid(Date date1, Date date2) {
        long milliseconds = date1.getTime() - date2.getTime();
        Log.i("sign", "isTimeDifferenceValid " + ((milliseconds / (1000)) > 30));
        return (int) (milliseconds / (1000)) > 30;
    }

    private boolean isLocationDifferenceValid(Location location1, Location location2) {
        if (location1 == null || location2 == null)
            return false;
        return location1.distanceTo(location2) > 50;
    }

    @SuppressLint("DefaultLocale")
    private void setupLocation() {
        satellite = findViewById(R.id.satellite_info);
        status = findViewById(R.id.gps_status_info);
        accuracy = findViewById(R.id.accuracy_info);
        distance = findViewById(R.id.distanceValueTxt);
        totalDistance = findViewById(R.id.totalDistanceValueTxt);
        currentSpeed = findViewById(R.id.currentSpeedTxt);
    }


    private void showTotalDistance() {
        double distance = sharedPreferencesManager.getDistance();
        String distanceUnits;
        if (distance <= 1000.0) {
            distanceUnits = "m";
        } else {
            distance /= 1000.0;
            distanceUnits = "km";
        }

        totalDistance.setText(
                String.format("%.1f %s", distance, distanceUnits)
                        .replace(',', '.')
                        .replace(".0", ""));
    }


    private void init_picture_attributes(int w , int h){
        rows = h;
        cols = w;
        left = rows / 8;
        width = cols - left;
        top = rows / 2.5;
        middleX = w /2;
        bottomY = h * .95;

        vehicleCenterX1 = middleX;
        vehicleCenterX2 = middleX;
        vehicleCenterY1 = bottomY-(rows/7);
        vehicleCenterY2 = bottomY-(rows/20);
        laneCenterX = 0;
        laneCenterY = (bottomY-(rows/7) + bottomY-(rows/20)) / 2;
    }
    private void draw_LaneLines(Mat frame){
        init_picture_attributes(frame.cols(), frame.rows());
        mRgba = frame.clone();
        mGray = frame.clone();
        Imgproc.cvtColor(mGray, mGray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.blur(mGray, mGray, ksize, blurPt);
        Imgproc.GaussianBlur(mRgba, mRgba, ksize, sigma);

        Mat rgbaInnerWindow = mRgba.submat((int)top, rows, left, width);
        Mat lines = new Mat();
        /* rgbaInnerWindow & mIntermediateMat = ROI Mats */
        rgbaInnerWindow.copyTo(rgba);
        Imgproc.cvtColor(rgbaInnerWindow, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(rgbaInnerWindow, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(rgbaInnerWindow, hls, Imgproc.COLOR_RGB2HLS);

        splitRGBChannels(rgba, hsv, hls);
        applyThreshold();
        org.opencv.core.Size size = new org.opencv.core.Size(5, 5);
        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, size));
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, size));
        Imgproc.Canny(mask, mEdges, 50, 150);
        Imgproc.HoughLinesP(mEdges, lines, 1, Math.PI/180, 50, 25, 85);
        if (lines.rows() > 0) {
            getAverageSlopes(lines);
            points.add(new float[]{(float)left,(float) top, (float) cols-left, (float) bottomY});

        }

        // adding the points to the list of points for the lane lines to be drawn on the image later

      // Imgproc.line(mRgba, new Point(vehicleCenterX1, vehicleCenterY1), new Point(vehicleCenterX2, vehicleCenterY2), darkGreen, 2, 8);
       // points.add(new float[]{(float)vehicleCenterX1,(float) vehicleCenterY1, (float) vehicleCenterX2, (float) vehicleCenterY2});
      //  Imgproc.rectangle(mRgba, new Point(left, top), new Point(cols-left, bottomY), darkGreen, 2);

//    return mask;

    }




    public void getAverageSlopes(Mat lines) {
        List<Double> left_slopes = new ArrayList<>();
        List<Double> right_slopes = new ArrayList<>();
        List<Double> left_y_intercept = new ArrayList<>();
        List<Double> right_y_intercept = new ArrayList<>();
        // init the imgHeight and imgWidth
        int imgHeight = mRgba.rows();
        // Threshold zone for detected lanes, lines must be within this zone
        double zoneX1 = cols-left*2.5;
        double zoneX2 = left*2.5;
       // Imgproc.line(mRgba, new Point(zoneX1, top), new Point(zoneX1, top+5), new Scalar(0, 155, 0), 2, 8);
       // Imgproc.line(mRgba, new Point(zoneX2, top), new Point(zoneX2, top+5), new Scalar(0, 155, 0), 2, 8);
        // left line float array
        float[] left_line = {(float) zoneX1, (float) top, (float) zoneX1, (float)(top + 100)};
        // right line float array
        float[] right_line = {(float) zoneX2, (float) top, (float) zoneX2, (float)(top + 100)};
       // points.add(left_line);
       // points.add(right_line);

        for (int i=0; i<lines.rows(); i++) {
            double[] points = lines.get(i, 0);
            double x1, y1, x2, y2;

            try {
                x1 = points[0];
                y1 = points[1];
                x2 = points[2];
                y2 = points[3];

                Point p1 = new Point(x1, y1);
                Point p2 = new Point(x2, y2);

                double slope = (p2.y - p1.y) / (p2.x - p1.x);
                double y_intercept;

                if (slope > 0.375 && slope < 2.6) { // Right lane
                    if (p1.x+left < zoneX1) {
                        right_slopes.add(slope);
                        y_intercept = p1.y - (p1.x * slope);
                        right_y_intercept.add(y_intercept);
                    }
                }
                else if (slope > -2.6 && slope < -0.375) { // Left lane
                    if (p2.x+left > zoneX2) {
                        left_slopes.add(slope);
                        y_intercept = p1.y - (p1.x * slope);
                        left_y_intercept.add(y_intercept);
                    }
                }
            } catch (Error e) {
                Log.e(TAG, "onCameraFrame: ", e);
            }
        }

        double avg_left_slope = 0;
        double avg_right_slope = 0;
        double avg_left_y_intercept = 0;
        double avg_right_y_intercept = 0;

        for (int i=0; i< right_slopes.size(); i++) {
            avg_right_slope += right_slopes.get(i);
            avg_right_y_intercept += right_y_intercept.get(i);
        }
        avg_left_slope /= left_slopes.size();
        avg_left_y_intercept /= left_y_intercept.size();

        // x = (y-b)/m
        // y = xm + b
        double newLeftTopX = ((-avg_left_y_intercept)/avg_left_slope) + left;
        double newRightTopX = ((0 - avg_right_y_intercept)/avg_right_slope) + left;

        Point rightLanePt = new Point((imgHeight - avg_right_y_intercept)/avg_right_slope, imgHeight);
        Point leftLanePt = new Point((0), (-left*avg_left_slope)+avg_left_y_intercept);

        Point topLeftPt = new Point(newLeftTopX, 0 + top);
        Point topRightPt = new Point(newRightTopX, 0 + top);
        Point bottomLeftPt = new Point(-500+left, ((-500*avg_left_slope)+avg_left_y_intercept)+top);
        Point bottomRightPt = new Point(rightLanePt.x + left, rightLanePt.y + top);

        if (right_slopes.size() != 0 && left_slopes.size() != 0) {
            double laneCenterX1 = (laneCenterY-top-avg_left_y_intercept)/avg_left_slope + left;
            double laneCenterX2 = (laneCenterY-top-avg_right_y_intercept)/avg_right_slope + left;
            laneCenterX = (laneCenterX1+laneCenterX2) / 2;

            laneZone = new MatOfPoint(topLeftPt, topRightPt, bottomRightPt, bottomLeftPt);
            laneZoneMat.setTo(new Scalar(0, 0, 0));
            Imgproc.fillConvexPoly(laneZoneMat, laneZone, new Scalar(255, 240, 160));
            Core.addWeighted(laneZoneMat, .5, mRgba, 1, 0, mRgba);
            laneZone.release();

        }
      //  points.add(new float[]{(float)laneCenterX,(float) laneCenterY, (float) laneCenterX, (float) laneCenterY});
      //  Imgproc.line(mRgba, new Point(vehicleCenterX1, laneCenterY), new Point(laneCenterX, laneCenterY), darkGreen, 2, 8);
      //  Imgproc.circle(mRgba, new Point(laneCenterX, laneCenterY), 4, new Scalar(0, 0, 255), 7);
        // add point to list for drawing lines
      //  points.add(new float[]{(float)laneCenterX,(float) laneCenterY});
        if (left_slopes.size() != 0) {
            Imgproc.line(mRgba, topLeftPt, bottomLeftPt, new Scalar(225, 0, 0), 8);
            // convert topLeftPt and bottomLeftPt to float array for drawing the lane lines
            float[] leftLine = {(float)topLeftPt.x, (float)topLeftPt.y , (float) bottomLeftPt.x, (float) bottomLeftPt.y};
          //  points.add(leftLine);
        }
        if (right_slopes.size() != 0) {
            Imgproc.line(mRgba, bottomRightPt, topRightPt, new Scalar(0, 0, 225), 8);
            // adding the points lines
            float[] rightLine = {(float)bottomRightPt.x, (float)bottomRightPt.y , (float) topRightPt.x, (float) topRightPt.y};
         //   points.add(rightLine);
        }
    }
    public void splitRGBChannels(Mat rgb_split, Mat hsv_split, Mat hls_split) {
        List<Mat> rgbChannels = new ArrayList<>();
        List<Mat> hsvChannels = new ArrayList<>();
        List<Mat> hlsChannels = new ArrayList<>();

        Core.split(rgb_split, rgbChannels);
        Core.split(hsv_split, hsvChannels);
        Core.split(hls_split, hlsChannels);

        rgbChannels.get(0).copyTo(mRed);
        rgbChannels.get(1).copyTo(mGreen);
        rgbChannels.get(2).copyTo(mBlue);

        hsvChannels.get(0).copyTo(mHue_hsv);
        hsvChannels.get(1).copyTo(mSat_hsv);
        hsvChannels.get(2).copyTo(mVal_hsv);

        hlsChannels.get(0).copyTo(mHue_hls);
        hlsChannels.get(1).copyTo(mSat_hls);
        hlsChannels.get(2).copyTo(mLight_hls);
//
//
        for (int i = 0; i < rgbChannels.size(); i++){
            rgbChannels.get(i).release();
        }

        for (int i = 0; i < hsvChannels.size(); i++){
            hsvChannels.get(i).release();
        }

        for (int i = 0; i < hlsChannels.size(); i++){
            hlsChannels.get(i).release();
        }
    }
    public void applyThreshold() {
        Scalar lowerThreshold = new Scalar(210), higherThreshold = new Scalar(255);

        Core.inRange(mRed, lowerThreshold, higherThreshold, mRed);
//        Core.inRange(mGreen, new Scalar(225), new Scalar(255), mGreen);
//        Core.inRange(mBlue, new Scalar(200), new Scalar(255), mBlue);

//        Core.inRange(mHue_hsv, new Scalar(200), new Scalar(255), mHue_hsv);
//        Core.inRange(mSat_hsv, new Scalar(200), new Scalar(255), mSat_hsv);
        Core.inRange(mVal_hsv, lowerThreshold, higherThreshold, mVal_hsv);

//        Core.inRange(mHue_hls, new Scalar(200), new Scalar(255), mHue_hls);
//        Core.inRange(mLight_hls, new Scalar(200), new Scalar(255), mLight_hls);
//        Core.inRange(mSat_hls, new Scalar(200), new Scalar(255), mSat_hls);

        Core.bitwise_and(mRed, mVal_hsv, mask);
    }
    private void setupRecycler() {
        adapter = new SignAdapter(this);

        RecyclerView signRecycler = findViewById(R.id.signRecycler);
        signRecycler.setAdapter(adapter);
        signRecycler.setLayoutManager(new LinearLayoutManager(this));
    }

    @Override
    protected int getLayoutId() {
        return R.layout.camera_connection_fragment_tracking;
    }

    @Override
    protected Size getDesiredPreviewFrameSize() {
        return DESIRED_PREVIEW_SIZE;
    }

    @Override
    protected void setUseNNAPI(final boolean isChecked) {
        runInBackground(() -> detector.setUseNNAPI(isChecked));
    }

    @Override
    protected void setNumThreads(final int numThreads) {
        runInBackground(() -> detector.setNumThreads(numThreads));
    }

    public void onGpsStatusChanged(GpsStatusEntity event) {
        satellite.setText(event.getSatellite());
        status.setText(event.getStatus());
        accuracy.setText(event.getAccuracy());
    }

    private void showGpsDisabledDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(getString(R.string.gps_disabled))
                .setMessage(getString(R.string.please_enable_gps))
                .setNegativeButton(android.R.string.cancel, (dialog, id) -> {
                    dialog.cancel();
                })
                .setPositiveButton(android.R.string.ok, (dialog, id) -> {
                    startActivity(new Intent("android.settings.LOCATION_SOURCE_SETTINGS"));
                });
        AlertDialog dialog = builder.create();
        dialog.setCancelable(true);
        dialog.setOnShowListener(arg -> {
                    dialog.getButton(AlertDialog.BUTTON_POSITIVE)
                            .setTextColor(ContextCompat.getColor(this, R.color.cod_gray));
                    dialog.getButton(AlertDialog.BUTTON_NEGATIVE)
                            .setTextColor(ContextCompat.getColor(this, R.color.cod_gray));
                }
        );
        dialog.show();
    }

    private SignEntity getSignImage(Classifier.Recognition result, Bitmap bitmap) {
        SignEntity sign = null;
        if ("crosswalk".equals(result.getTitle())) {
            // send zero speed to Esp32Driver to stop the car
         //   Esp32Driver.sendCommand("00000000");
            sign = new SignEntity(result.getTitle(), R.drawable.crosswalk, R.raw.crosswalk);
        } else if ("stop".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.stop, R.raw.stop);
        } else if ("main road".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.main_road, R.raw.main_road);
        } else if ("give road".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.give_road, R.raw.give_road);
        } else if ("children".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.children, R.raw.children);
        } else if ("dont stop".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.dont_stop, R.raw.dont_stop);
        } else if ("no parking".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.no_parking, R.raw.no_parking);
        } else if ("dont move".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.dont_move, R.raw.dont_move);
        } else if ("dont enter".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.dont_enter, R.raw.dont_enter);
        } else if ("dont overtake".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.no_overtake, R.raw.dont_overtake);
        } else if ("speed limit 5".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_5, R.raw.speed_limit_5);
        } else if ("speed limit 10".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_10, R.raw.speed_limit_10);
        } else if ("speed limit 20".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_20, R.raw.speed_limit_20);
        } else if ("speed limit 30".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_30, R.raw.speed_limit_30);
        } else if ("speed limit 40".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_40, R.raw.speed_limit_40);
        } else if ("speed limit 50".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_50, R.raw.speed_limit_50);
        } else if ("speed limit 60".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_60, R.raw.speed_limit_60);
        } else if ("speed limit 70".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_70, R.raw.speed_limit_70);
        } else if ("speed limit 80".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_80, R.raw.speed_limit_80);
        } else if ("speed limit 90".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_90, R.raw.speed_limit_90);
        } else if ("speed limit 100".equals(result.getTitle())) {
            sign = new SignEntity(result.getTitle(), R.drawable.speed_limit_100, R.raw.speed_limit_100);
        }

        if (sign != null) {
            sign.setConfidenceDetection(result.getConfidence());

            sign.setScreenLocation(result.getLocation());
            if (data != null) {
                sign.setLocation(data.getLocation());
            }

//            if (sign.getName().contains("speed")
//                    && sign.isValidSize(rgbFrameBitmap)
//                    && speedLimitClassifier != null
//                    && classifier.isChecked()) {
//                try {
//                    SignEntity finalSign = sign;
//
//                    runInBackground(
//                            () -> {
//                                Matrix matrix = new Matrix();
//                                matrix.postRotate(90);
//                                Bitmap crop = Bitmap.createBitmap(rgbFrameBitmap,
//                                        (int) finalSign.getScreenLocation().left,
//                                        (int) finalSign.getScreenLocation().top,
//                                        (int) finalSign.getScreenLocation().width(),
//                                        (int) finalSign.getScreenLocation().height(),
//                                        matrix,
//                                        true);
//
//                                List<ClassificationEntity> recognitions =
//                                        speedLimitClassifier.recognizeImage(prepareImageForClassification(crop));
//
////                                Toast.makeText(DetectorActivity.this, new Gson().toJson(recognitions), Toast.LENGTH_SHORT).show();
//                            });
//
//                } catch (Exception ignored) {
//                }
//            }
        }

        return sign;
    }

    @Override
    public synchronized void onDestroy() {
        super.onDestroy();
        if (compositeDisposable != null) {
            compositeDisposable.dispose();
            compositeDisposable = null;
        }
    }

    private void inject() {
        DaggerScreenComponent.builder()
                .applicationComponent(getApplicationComponent())
                .build()
                .inject(this);
    }

}
