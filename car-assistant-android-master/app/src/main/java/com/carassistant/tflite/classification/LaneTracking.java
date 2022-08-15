package com.carassistant.tflite.classification;


import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.util.Log;

import com.carassistant.R;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.JavaCameraView;
import org.opencv.objdetect.CascadeClassifier;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.ConnectException;
import java.nio.ByteBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class LaneTracking {

    public static final String MODEL_FILENAME = "model.tflite";
    private Interpreter interpreter;
    private static CascadeClassifier detector;
    private static CascadeClassifier detectorPed;
    private static File cascadeFilePedestrain;
    private static File cascadeFileCarDet;
    public LaneTracking(Interpreter interpreter) {
        this.interpreter = interpreter;
    }
    public static LaneTracking classifier(AssetManager assetManager, String modelPath, Context context) throws IOException {
        ByteBuffer byteBuffer = loadModelFile(assetManager, modelPath);
        Interpreter interpreter = new Interpreter(byteBuffer);
        initFileCascadeCarsDet(context);
        initFileCascadePedestrains(context);
        return new LaneTracking(interpreter);
    }
    private static ByteBuffer loadModelFile(AssetManager assetManager, String modelPath) throws IOException {
        AssetFileDescriptor fileDescriptor = assetManager.openFd(modelPath);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private static void initFileCascadePedestrains(Context context) {
        try {
            System.out.println("init file Cascade");
            InputStream is = context.getResources().openRawResource(R.raw.pedestrian);
            File cascadeDir = context.getDir("cascade", Context.MODE_PRIVATE);
            cascadeFilePedestrain = new File(cascadeDir, "pedestrian.xml");
            Log.d("CarAssistant", "File pedestrian created successfully");
            FileOutputStream os = new FileOutputStream(cascadeFilePedestrain);
            byte[] buffer = new byte[4096];
            int bytesRead;
            while ((bytesRead = is.read(buffer)) != -1) {
                os.write(buffer, 0, bytesRead);
            }
            is.close();
            os.close();
            detectorPed = new CascadeClassifier(cascadeFilePedestrain.getAbsolutePath());

            if(detectorPed.empty()){
                detectorPed = null;
            }
            else{
                cascadeDir.delete();
            }
        } catch (Exception e) {
            System.out.println("something wrong!");
            e.printStackTrace();
        }
    }
    private static void initFileCascadeCarsDet(Context context) {
        try {
            System.out.println("init file Cascade");
            InputStream is = context.getResources().openRawResource(R.raw.cars_detect);
            File cascadeDir = context.getDir("cascade", Context.MODE_PRIVATE);
            cascadeFileCarDet = new File(cascadeDir, "cars_detect.xml");

            FileOutputStream os = new FileOutputStream(cascadeFileCarDet);
            byte[] buffer = new byte[4096];
            int bytesRead;
            while ((bytesRead = is.read(buffer)) != -1) {
                os.write(buffer, 0, bytesRead);
            }
            is.close();
            os.close();
            detector = new CascadeClassifier(cascadeFileCarDet.getAbsolutePath());

            if(detector.empty()){
                detector = null;
            }
            else{
                cascadeDir.delete();
            }
        } catch (Exception e) {
            System.out.println("something wrong!");
            e.printStackTrace();
        }
    }
    // get function to get interpreter
    public Interpreter getInterpreter() {
        return interpreter;
    }
    // get function to get cascade classifier
    public static CascadeClassifier getDetector() {
        return detector;
    }
}
