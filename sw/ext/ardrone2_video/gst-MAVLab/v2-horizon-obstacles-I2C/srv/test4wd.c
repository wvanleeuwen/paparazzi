void calibrate_compass(int howlong, int speed) {
    int t0; 
    int ix;
    int loop;
    
    compassxcal(9999, -9999, 9999, -9999, 1);  // reset the calibration values
    t0 = time();
    loop = 1;
    
    while (loop == 1) {
        if (time() > (t0 + howlong))
            loop = 0;
        motorx(speed, -speed);
        delay(300);
        ix = compassx();
        printf("heading = %d\r\n", ix);
        motorx(0, 0);
        delay(200);
        compassx();
        printf("heading = %d\r\n", ix);
    }
    motorx(0, 0);
    compassxcal(cxmin, cxmax, cymin, cymax, 0);  // set calibration, turn off data gathering
    printf("compass calibration finished: %d %d %d %d\r\n", cxmin, cxmax, cymin, cymax);
}

void go(int dist, int speed) {  
    int a0, a1, a2, aa;
    int b0, b1, b2, bb;
    int loop;
  
    loop = 1;
    aa = 0;  // wrap-around flags
    bb = 0;

    a0 = encoderx(1);
    a1 = a0 + dist; 
    if (a1 > 65536) {
        a1 = a1 - 65536;
        aa = 1;
    }
    b0 = encoderx(3);
    b1 = b0 + dist;
    if (b1 > 65536) {
        b1 = b1 - 65536;
        bb = 1;
    }
    
    motorx(speed, speed); 
    while (loop == 1) {
        if (analogx(3) > 150)  // check forward IR sensor
            motorx(0, 0);
        else
            motorx(speed, speed);
        a2 = encoderx(1); 
        if ((aa == 0) && (a2 > a1))
            loop = 0;      
        if ((aa == 1) && (a2 < a0) && (a2 > a1))
            loop = 0;
        b2 = encoderx(3); 
        if ((bb == 0) && (b2 > b1))
            loop = 0;      
        if ((bb == 1) && (b2 < b0) && (b2 > b1))
            loop = 0;
        printf("a2 = %d  b2 = %d\n", a2, b2);
    }
    motorx(0, 0);
}

void turn(int head, int speed) {
    int h0, h1, h2, dir, diff;
    int loop;

    h0 = compassx();
    if ((h0 <= head) && ((head - h0) < 180)) {
        dir = 0;  // spin right
        diff = head - h0;
    }
    if ((h0 <= head) && ((head - h0) >= 180)) {
        dir = 1;  // spin left
        diff = h0 - head + 360;
    }
    if ((h0 > head) && ((h0 - head) < 180)) {
        dir = 1;  // spin left
        diff = h0 - head;
    }
    if ((h0 > head) && ((h0 - head) >= 180)) {
        dir = 0;  // spin right
        diff = head - h0 + 360;
    }
    
    printf("h0=%d  head=%d  diff=%d  dir=%d\n", h0, head, diff, dir);

    if (diff < 5)  // don't rotate if heading is within 5 deg
        return;

    if (dir == 1) 
        motorx(-speed, speed);   // spin left
    else
        motorx(speed, -speed);   // spin right
    loop = 1;
    while (loop) {
        delay(50);
        h0 = compassx();
        if (abs(h0 - head) < 20) {
            motorx(0, 0);
            loop = 0;
        }
    }
}

servos(50,50);
delay(1000);
calibrate_compass(10000, 85);  // 10 seconds at 85 speed
delay(1000);
turn(90, 85);
delay(1000);
turn(90, 85);  // in case of overshoot
delay(1000);
go(2000, 75);
delay(1000);
turn(270, 85);
delay(1000);
turn(270, 85);  // in case of overshoot
delay(1000);
go(2000, 75);


