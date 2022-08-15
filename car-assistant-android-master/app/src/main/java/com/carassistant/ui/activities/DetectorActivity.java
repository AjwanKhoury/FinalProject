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
import android.os.Build;
import android.os.Bundle;
import android.os.SystemClock;
import android.text.SpannableString;
import android.text.style.RelativeSizeSpan;
import android.util.Log;
import android.util.Pair;
import android.util.Size;
import android.util.TypedValue;
import android.view.SurfaceView;
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
import com.carassistant.managers.BTEsp32;
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
import java.lang.reflect.Array;
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

import static org.opencv.core.CvType.CV_32F;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

    private boolean computingDetection = false;

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

        inject();
        setupLocation();
        setupRecycler();
        setupViews();
        setCallBack();
        setupClassifier();

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
        this.draw_LaneLines(rgbFrame);
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
                        processResults(mat);
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
//                                runOnUiThread(() -> draw_LaneLines(finalMat));

                            }
                        }
                        ArrayList<float[]> mappedRecognitionsPoints = new ArrayList<>();
                        // for loop to draw the lane lines on the canvas
                        for (int i = 0; i < points.size(); i++) {
                            float[] point = points.get(i);
                            cropToFrameTransform.mapPoints(point);
                            mappedRecognitionsPoints.add(point);

                        }
                        tracker.trackResults(mappedRecognitions, currTimestamp);
                        tracker.trackLaneDetection(mappedRecognitionsPoints, currTimestamp);
                        trackingOverlay.postInvalidate();

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
    private Mat processResults(Mat frame) {
        steering_angle_ = get_steering_prediction(frame.clone());
        Mat displayMat = null;
        displayMat  = draw_LaneLines(frame.clone());
        counterFrme ++;
        if(steering_angle_< 0 ) {
            Imgproc.putText(
                    displayMat,                          // Matrix obj of the image
                    "turn left " + steering_angle_ * -1 + "% of the wheel",          // Text to be added
                    new Point(10, 50),               // point
                    Core.FONT_HERSHEY_SIMPLEX,      // front face
                    1,                               // front scale
                    new Scalar(255, 0, 0),             // Scalar object for color
                    6                                // Thickness
            );
        }
        if(steering_angle_ > 0 ) {
            Imgproc.putText(
                    displayMat,                          // Matrix obj of the image
                    "turn right " + steering_angle_  + "% of the wheel",          // Text to be added
                    new Point(10, 50),               // point
                    Core.FONT_HERSHEY_SIMPLEX,      // front face
                    1,                               // front scale
                    new Scalar(255, 0, 0),             // Scalar object for color
                    6                                // Thickness
            );
        }
        return displayMat;
    }
    private float get_steering_prediction(Mat frame) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(frame, frame, new org.opencv.core.Size(3, 3), 0, 0);

        Mat f = new Mat();
        Imgproc.resize(frame, f, new org.opencv.core.Size(200, 66));
        //   f = Dnn.blobFromImage(f, 0.00392, new Size(200, 66) , new Scalar(0,0 ,0), false,false);
        f.convertTo(f, CV_32F);
        StringBuilder sb = new StringBuilder();
        String s = new String();
        System.out.println("hei " + f.height() + ", wit" + f.width() + "ch " + f.channels());
        System.out.println("col " + f.cols() + ", row" + f.rows() + "ch " + f.channels());
        float[][][][] inputs = new float[1][200][66][3];
        float fs[] = new float[3];
        for (int r = 0; r < f.rows(); r++) {
            //sb.append(""+r+") ");
            for (int c = 0; c < f.cols(); c++) {
                f.get(r, c, fs);
                //sb.append( "{");
                inputs[0][c][r][0] = fs[0] / 255;
                inputs[0][c][r][1] = fs[1] / 255;
                inputs[0][c][r][2] = fs[2] / 255;

            }
        }
        return steering_angle_;
    }

    private Mat draw_LaneLines(Mat frame){
        Mat gray = frame.clone();
        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.GaussianBlur(gray,gray,new org.opencv.core.Size(5,5),0,0);
        Imgproc.Canny(gray,gray,60,140);

        int height = gray.rows();
        int width = gray.cols();
        System.out.println("hei "+ height + "wid " + width);
        Mat mask = new Mat(height,width, CvType.CV_8UC1,Scalar.all(0));
        Point[] rook_points = new Point[3];
        rook_points[0]  = new Point(200,height);
        rook_points[1]  = new Point(width/2 +50, height/2);
        rook_points[2]  = new Point(width-50,height);
        MatOfPoint matPt = new MatOfPoint();
        matPt.fromArray(rook_points);

        List<MatOfPoint> ppt = new ArrayList<MatOfPoint>();
        ppt.add(matPt);
        Imgproc.fillPoly(mask,
                ppt,
                new Scalar( 255,255,255 )
        );
        Mat after_bit = new Mat();
        Core.bitwise_and(gray,mask,after_bit);
//        lines = cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 100, np.array([]), minLineLength=40,maxLineGap=5)
        Mat result = after_bit.clone();
        Mat lines = new Mat();
        Imgproc.HoughLinesP(after_bit, lines, 2, Math.PI/180, 100, 50, 4);

        Mat frame_clone = frame.clone();

        ArrayList<Pair<Double, Double>> left_fit = new ArrayList<>();
        ArrayList<Pair<Double, Double>> right_fit = new ArrayList<>();

        float left_slope_avr = 0;
        float right_slope_avr = 0;
        float left_iterccept_avr = 0;
        float right_iterccept_avr = 0;
        ///going through lines:
        for(int j = 0; j < lines.rows(); j++) {
            for (int i = 0; i < lines.cols(); i++) {
                double[] val = lines.get(j, i);
                double x1 = val[0];                double y1 = val[1];
                double x2 = val[2];                double y2 = val[3];
                double slope = Double.MAX_VALUE;
                double intercept = 0;
                boolean flag_slope_inter =false;

                if(x1-x2!=0 ) {
                    slope = (y1 - y2) / (x1 - x2);
                    intercept = (y1 - (slope*x1)) ;
                    flag_slope_inter = true;
                }

                if ( slope < 0  && flag_slope_inter) {
                    left_fit.add(new Pair(slope, intercept));
                    left_slope_avr += slope;
                    left_iterccept_avr += intercept;
                }
                else if ( slope > 0 && flag_slope_inter) {
                    right_fit.add(new Pair(slope, intercept));
                    right_slope_avr += slope;
                    right_iterccept_avr += intercept;
                }
            }
        }
        left_slope_avr = left_slope_avr/left_fit.size();
        left_iterccept_avr = left_iterccept_avr/left_fit.size();
        right_slope_avr = right_slope_avr/right_fit.size();
        right_iterccept_avr = right_iterccept_avr/right_fit.size();

        float [] points_Left_line = make_points(left_slope_avr, left_iterccept_avr, height);
        float [] points_Right_line = make_points(right_slope_avr, right_iterccept_avr, height);
        points = new ArrayList<>();
        points.add(points_Left_line);
        points.add(points_Right_line);
        return result;
//    return mask;
    }
    private float[] make_points(double slope, double intercept, int height) {
        float [] ret = new float[4];
        float y1 = height;
        float y2 = y1*3/5;
        float x1 = (float) ((y1-intercept)/slope);
        float x2 = (float) ((y2-intercept)/slope);
        ret[0] = x1 ;
        ret[1] = y1 ;
        ret[2] = x2 ;
        ret[3] = y2 ;
        return ret;
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
            Esp32Driver.sendCommand("00000000");
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
