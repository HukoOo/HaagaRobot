package com.dfrobot.angelo.blunobasicdemo;

import android.content.Context;
import android.os.Bundle;
import android.content.Intent;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.ScrollView;
import android.view.KeyEvent;
import android.content.DialogInterface;
import android.app.AlertDialog;
import android.widget.ImageView;

import java.util.logging.Handler;
import java.util.logging.LogRecord;

public class MainActivity  extends BlunoLibrary {
	private Button buttonScan;
	private Button buttonSerialSend;
	private EditText serialSendText;
	private TextView serialReceivedText;
	private SeekBar speedBar;
	private int motorSpeed=50;

	private Button cmdGo;
	private Button cmdBack;
	private Button cmdRight;
	private Button cmdLeft;
	private Button cmdStop;

	private boolean isBtnDown;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
        onCreateProcess();													//onCreate Process by BlunoLibrary
        serialBegin(57600);													//set the Uart Baudrate on BLE chip

		ImageView image= (ImageView) findViewById(R.id.imageView);
		image.setImageResource(R.drawable.logo_rise);
		ImageView image2= (ImageView) findViewById(R.id.imageView2);
		image2.setImageResource(R.drawable.haaga);

		TextView tv=(TextView) findViewById(R.id.speedResult);
		tv.setText("50");

		speedBar = (SeekBar) findViewById(R.id.speedBar);
		speedBar.setProgress(motorSpeed);
		speedBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
				printSelected(progress); // TextView
			}

			public void onStartTrackingTouch(SeekBar seekBar) {
				// method when touching
			}

			public void onStopTrackingTouch(SeekBar seekBar) {
				doAfterTrack(); // method after task
			}
		});


        serialReceivedText=(TextView) findViewById(R.id.serialReveicedText);	//initial the EditText of the received data
        serialSendText=(EditText) findViewById(R.id.serialSendText);			//initial the EditText of the sending data

        buttonSerialSend = (Button) findViewById(R.id.buttonSerialSend);		//initial the button for sending the data
        buttonSerialSend.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				// TODO Auto-generated method stub

				serialSend(serialSendText.getText().toString());				//send the data to the BLUNO
			}
		});

        buttonScan = (Button) findViewById(R.id.buttonScan);					//initial the button for scanning the BLE device
        buttonScan.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				// TODO Auto-generated method stub

				buttonScanOnClickProcess();										//Alert Dialog for selecting the BLE device
			}
		});

		cmdGo = (Button) findViewById(R.id.cmdGo);
		cmdGo.setOnTouchListener(new View.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				TextView tv=(TextView) findViewById(R.id.speedResult);
				int action = event.getAction();
				if (MotionEvent.ACTION_DOWN == action) {
					isBtnDown = true;
					serialSend("R"+tv.getText()+"L"+tv.getText()+"E");
				} else if (MotionEvent.ACTION_UP == action) {
					isBtnDown = false;
				}
				return false;
			}
		});
		cmdBack = (Button) findViewById(R.id.cmdBack);
		cmdBack.setOnTouchListener(new View.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				TextView tv=(TextView) findViewById(R.id.speedResult);
				int action = event.getAction();
				if(MotionEvent.ACTION_DOWN==action){
					isBtnDown = true;
					serialSend("R-"+tv.getText()+"L-"+tv.getText()+"E");
				}
				else if(MotionEvent.ACTION_UP==action)
				{
					isBtnDown = false;
				}
				return false;
			}
		});
		cmdRight = (Button) findViewById(R.id.cmdRight);
		cmdRight.setOnTouchListener(new View.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				TextView tv=(TextView) findViewById(R.id.speedResult);
				int action = event.getAction();
				if(MotionEvent.ACTION_DOWN==action){
					isBtnDown = true;
					serialSend("R-"+tv.getText()+"L"+tv.getText()+"E");
				}
				else if(MotionEvent.ACTION_UP==action)
				{
					isBtnDown = false;
				}
				return false;
			}
		});
		cmdLeft = (Button) findViewById(R.id.cmdLeft);
		cmdLeft.setOnTouchListener(new View.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				TextView tv=(TextView) findViewById(R.id.speedResult);
				int action = event.getAction();
				if(MotionEvent.ACTION_DOWN==action){
					isBtnDown = true;
					serialSend("R"+tv.getText()+"L-"+tv.getText()+"E");
				}
				else if(MotionEvent.ACTION_UP==action)
				{
					isBtnDown = false;
				}
				return false;
			}
		});
		cmdStop = (Button) findViewById(R.id.cmdStop);
		cmdStop.setOnTouchListener(new View.OnTouchListener() {

			@Override
			public boolean onTouch(View v, MotionEvent event) {
				int action = event.getAction();
				if(MotionEvent.ACTION_DOWN==action){
					isBtnDown = true;
					serialSend("R0L0E");
				}
				else if(MotionEvent.ACTION_UP==action)
				{
					isBtnDown = false;
				}
				return false;
			}
		});

		ScrollView scroll = (ScrollView)findViewById(R.id.scrollView);
		scroll.post(new Runnable()
		{
			@Override
			public void run()
			{
				ScrollView scroll = (ScrollView)findViewById(R.id.scrollView);
				scroll.fullScroll(ScrollView.FOCUS_DOWN);

			}
		});

	}

	protected void onResume(){
		super.onResume();
		System.out.println("BlUNOActivity onResume");
		onResumeProcess();														//onResume Process by BlunoLibrary
	}

	@Override
	public boolean onKeyDown(int keyCode, KeyEvent event) {
		switch (keyCode) {
			//Event
			case KeyEvent.KEYCODE_BACK:
				new AlertDialog.Builder(this)
						.setTitle("Exit Programe")
						.setMessage("Are you sure you want to exit?")
						.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
							@Override
							public void onClick(DialogInterface dialog, int which) {
								// close process
								serialSend("R0L0E");
								long saveTime = System.currentTimeMillis();
								long currTime = 0;
								while( currTime - saveTime < 500){
									currTime = System.currentTimeMillis();
								}
								serialSend("M-E");
								saveTime = System.currentTimeMillis();
								currTime = 0;
								while( currTime - saveTime < 1000){
									currTime = System.currentTimeMillis();
								}
								android.os.Process.killProcess(android.os.Process.myPid());
							}
						})
						.setNegativeButton("No", null)
						.show();
				break;
			default:
				break;
		}
		return super.onKeyDown(keyCode, event);
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		onActivityResultProcess(requestCode, resultCode, data);					//onActivityResult Process by BlunoLibrary
		super.onActivityResult(requestCode, resultCode, data);
	}

    @Override
    protected void onPause() {
        super.onPause();
        onPauseProcess();														//onPause Process by BlunoLibrary
    }

	protected void onStop() {
		super.onStop();
		onStopProcess();														//onStop Process by BlunoLibrary
	}

	@Override
    protected void onDestroy() {
        super.onDestroy();
        onDestroyProcess();														//onDestroy Process by BlunoLibrary
    }

	@Override
	public void onConectionStateChange(connectionStateEnum theConnectionState) {//Once connection state changes, this function will be called
		switch (theConnectionState) {											//Four connection state
		case isConnected:
			buttonScan.setText("Connected");
			serialSend("M+E");
			break;
		case isConnecting:
			buttonScan.setText("Connecting");
			break;
		case isToScan:
			buttonScan.setText("Scan");
			break;
		case isScanning:
			buttonScan.setText("Scanning");
			break;
		case isDisconnecting:
			buttonScan.setText("isDisconnecting");
			break;
		default:
			break;
		}
	}

	@Override
	public void onSerialReceived(String theString) {							//Once connection data received, this function will be called
		// TODO Auto-generated method stub
		serialReceivedText.append(theString+"\n");							//append the text into the EditText
		//The Serial data from the BLUNO may be sub-packaged, so using a buffer to hold the String is a good choice.
	}

	public void printSelected(int value){
		TextView tv=(TextView) findViewById(R.id.speedResult);
		tv.setText(String.valueOf(value));
	}
	private void doAfterTrack(){
		TextView tv=(TextView) findViewById(R.id.speedResult);
		tv.setText(tv.getText());
	}
}
