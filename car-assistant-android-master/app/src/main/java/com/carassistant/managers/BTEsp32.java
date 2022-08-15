package com.carassistant.managers;

import android.app.Application;
import android.os.Handler;

public class BTEsp32 extends Application{
    private final int STATUS_CHECK_INTERVAL = 500;
    private MyBtEngine mBtEngine;
    @Override
    public void onCreate() {
        super.onCreate();
        mBtEngine = new MyBtEngine();
        handlerStatusCheck.postDelayed(new Runnable() {
            @Override
            public void run() {
                onBtStatusCheckTimer();
                handlerStatusCheck.postDelayed(this, STATUS_CHECK_INTERVAL);
            }
        }, STATUS_CHECK_INTERVAL);
    }
    private final Handler handlerStatusCheck = new Handler();
    private void onBtStatusCheckTimer(){
        if(mBtEngine.getState() == MyBtEngine.BT_STATE_NONE) {
            mBtEngine.start();
        }
    }
    public boolean writeBt(byte[] buffer){ return mBtEngine.writeBt(buffer); }
}