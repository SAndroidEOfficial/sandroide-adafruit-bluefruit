package it.unibs.adafruitbluefruit;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.util.ArrayList;

import it.unibs.sandroide.lib.BLEContext;
import it.unibs.sandroide.lib.activities.SandroideBaseActivity;
import it.unibs.sandroide.lib.device.DevicesManager;
import it.unibs.sandroide.lib.item.BLEItem;
import it.unibs.sandroide.lib.item.generalIO.SandroideDevice;
import it.unibs.sandroide.lib.item.generalIO.SandroidePin;

public class MainActivity extends SandroideBaseActivity {
    protected static final String TAG = "MainActivity";

    TextView tvButton, tvLed, tvTrimmer, tvCustomMessage;
    Button btnPower;

    static boolean power=true;
    SandroideDevice bluefruitDev;
    SandroidePin pinTrimmer, pinInput, pinOutput;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        tvButton =(TextView) findViewById(R.id.tvButtonState);
        tvLed =(TextView) findViewById(R.id.tvLedState);
        tvTrimmer =(TextView) findViewById(R.id.tvAnalogValue);
        tvCustomMessage =(TextView) findViewById(R.id.tvCustomMessage);

        bluefruitDev = ((SandroideDevice) BLEContext.findViewById("blue_sandroide_device"));
        if (bluefruitDev !=null) {

            bluefruitDev.setOnDeviceConnected(new SandroideDevice.OnDeviceConnectedListener() {
                @Override
                public void onEvent(SandroideDevice device) {
                    Log.i(TAG, "OK, Bluefruit connected");
                }
            }).setCustomOnMessageReceived(new SandroideDevice.OnMessageReceivedListener() {
                @Override
                public boolean onEvent(byte[] msg) {
                    if (msg.length>0) {
                        byte cmd = msg[0]; // first byte identifies type of command
                        switch(cmd) {
                            case 'W':
                                try {
                                    final String str = new String(msg);
                                    runOnUiThread(new Runnable() {
                                                      @Override
                                                      public void run() {
                                                          tvCustomMessage.setText(str.substring(1));
                                                      }
                                                  }
                                    );
                                    Log.d(TAG,"Extra parameter: "+str.substring(1));
                                }catch(Exception ex){
                                }
                                return false; // if command is custom one, don't go further with default handling
                            default:
                                break;
                        }
                    }

                    return true; // default processing
                }
            });;

            pinTrimmer = new SandroidePin().setDevice(bluefruitDev, 9) // we connect to pin number 9
                    .setMode(SandroidePin.PIN_MODE_ANALOG_INPUT) // the pin is analog input
                    .setDeltaThreshold(10)     // notify if trimmer value changes more than 10
                    .setSamplingInterval(1000) // means: sample every 1000ms
                    .setOnValueReceived(new SandroidePin.OnValueReceivedListener() {
                        @Override
                        public void onEvent(final Object newValue, Object oldValue) {
                            // new value contains a number from 0 to 1023
                            Log.d(TAG, "trimmer-value-changed: " + newValue);
                            runOnUiThread(new Runnable() {
                                              @Override
                                              public void run() {
                                                  tvTrimmer.setText(newValue.toString());
                                              }
                                          }
                            );
                        }
                    });

            pinInput = new SandroidePin().setDevice(bluefruitDev, 12)
                    .setMode(SandroidePin.PIN_MODE_DIGITAL_INPUT)
                    .setSamplingInterval(100)
                    .setOnValueReceived(new SandroidePin.OnValueReceivedListener() {
                        @Override
                        public void onEvent(final Object newValue, Object oldValue) {
                            // new value contains a boolean because the pin is defined as digital
                            pinOutput.setValue(newValue);
                            Log.d(TAG, "button-state-changed: " + newValue);
                            runOnUiThread(new Runnable() {
                                              @Override
                                              public void run() {
                                                  tvButton.setText(newValue.toString());
                                              }
                                          }
                            );
                        }
                    });

            pinOutput = new SandroidePin().setDevice(bluefruitDev, 3)
                    .setMode(SandroidePin.PIN_MODE_DIGITAL_OUTPUT)
                    .setOnValueReceived(new SandroidePin.OnValueReceivedListener() {
                        @Override
                        public void onEvent(final Object newValue, Object oldValue) {
                            // new value contains a float ranging from 0 to 1
                            // which is the percentage of the analog pin value
                            Log.d(TAG, "led-state-changed: " + newValue);
                            runOnUiThread(new Runnable() {
                                              @Override
                                              public void run() {
                                                  tvLed.setText(newValue.toString());
                                              }
                                          }
                            );
                        }
                    });

        }

        btnPower = (Button)findViewById(R.id.btnPower);
        btnPower.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View v)
            {
                if (bluefruitDev!=null) {
                    power = !power;
                    if (!power)
                        DevicesManager.stopDeviceControl(new ArrayList<BLEItem>(), bluefruitDev);
                    else
                        DevicesManager.connectDevice(bluefruitDev, bluefruitDev.getType());
                }
            }
        });

    }
}
