#include <stdio.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <gps.h>
#include <wiringPi.h>
#include "motor.h"
#include "mitibiki.h"
#include "xbee_at.h"
#include "ring_buffer.h"

static const int GPS_RING_LEN = 10;//gpsのリングバッファの長さ
static const double STACK_THRESHOLD = 0.000001; //stack判定するときの閾値
static const int GOAL_THRESHOLD = 3;
static const int SETPOINT = 0.0;//delta_angleの目標値
//static const double KP_VALUE = 0.65;
//static const double KI_VALUE = 0.00005;
//static const double KD_VALUE = 0;
static const int PID_LEN = 20;
static const int MAX_STACK_ROTATE_TIMES = 20;

float PGAIN = 2.0;//微調整　動作確認
float IGAIN = 0.01;

float GOAL_LAT;
float GOAL_LON;

float DestLat, DestLon, OriginLat, OriginLon;// 最新の緯度、最新の経度、現在の緯度、現在の経度
float rundata[4];
int i = 1;

double deg_to_rad(double deg) {
	(((deg) / 360) * 2 * 3.14159265358979);
}

double rad_to_deg(double rad) {
	(((rad) / 2 / 3.14159265358979) * 360);
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
	// returns course in degrees (North=0, West=270) from position 1 to position 2,
	// both specified as signed decimal-degrees latitude and longitude.
	// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
	// Courtesy of Maarten Lamers
	double dlon = (long2 - long1)*(3.14159265 / 180);
	lat1 = deg_to_rad(lat1);
	lat2 = deg_to_rad(lat2);
	double a1 = sin(dlon) * cos(lat2);
	double a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
		a2 += 6.283185307;
	}
	return rad_to_deg(a2);
}

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
	// returns distance in meters between two positions, both specified
	// as signed decimal-degrees latitude and longitude. Uses great-circle
	// distance computation for hypothetical sphere of radius 6372795 meters.
	// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
	// Courtesy of Maarten Lamers
	double delta = deg_to_rad(long1 - long2);
	double sdlong = sin(delta);
	double cdlong = cos(delta);
	lat1 = deg_to_rad(lat1);
	lat2 = deg_to_rad(lat2);
	double slat1 = sin(lat1);
	double clat1 = cos(lat1);
	double slat2 = sin(lat2);
	double clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	return delta * 6372795;
}



float PIDcontrol(float command, float current)  //command 目標地　current　現在地
{
	float controlValue;
	float error;
	static float i_error = 0.0;

	error = command - current;
	i_error += error;

	controlValue = PGAIN * error + IGAIN * i_error; //比例制御　

	return (controlValue);
}

int main()
{
	signal(SIGINT, handler);
	pwm_initialize();
	gps_init();
	xbee_init();

	double latitude;
	double longitude
	
	int GOAL_RANGE = 2;

	loc_t coord;
	gps_location(&coord);//gpsデータ取得

	OriginLat = coord.latitude;
	OriginLon = coord.longitude;

	rundata[0] = OriginLat;
	rundata[1] = OriginLon;

	unsigned long  distance = (unsigned long)distanceBetween(
		OriginLat,
		OriginLon,
		GOAL_LAT,
		GOAL_LON);

	rundata[2] = distance;
	rundata[3] = 0;
	if (distance <= GOAL_RANGE)
	{
		printf("goal");
		motor_control(0, 0);
		return 0;
	}

	motor_control(75, 75);
	delay(10000);
	motor_control(0, 0);
	while (1)
	{
		//float x, y, z;
		//float originLat, originLon;
		float angle1, angle2, angle3;
		static float angle = 0;
		//float dt;
		float controlValue;
		float error;
		unsigned long distance;

		int i;

		loc_t coord;
		gps_location(&coord);//gpsデータ取得
		if (coord.latitude == 0.0)
		{
			printf("GPS return 0 value\n");
			//RING BUFFERの更新はしない(stack判定誤作動のため)
		}
		else
		{
			enqueue(latring, coord.latitude); //緯度を格納
			enqueue(lonring, coord.longitude); //経度を格納
			printf("time:%f\nlatitude:%f\nlongitude:%f\n", coord.time, coord.latitude, coord.longitude);
			xbeePrintf("latitude:%f\r\nlongitude:%f\r\n", coord.latitude, coord.longitude);
		}
		//measure_gyro(&x, &y, &z);
		//angle += z * dt;
		//angle = DEG2RAD*AngleNormalization(angle);
		angle = 0;
		delay(2000);

		DestLat = coord.latitude;
		DestLon = coord.longitude;

		rundata[0] = DestLat;
		rundata[1] = DestLon;

		distance = (unsigned long)distanceBetween(
			OriginLat,
			OriginLon,
			GOAL_LAT,
			GOAL_LON);


		if (distance <= GOAL_RANGE)
		{
			printf("goal");
			motor_control(0, 0);
			return 0;
		}

		angle1 = courseTo(
			OriginLat, OriginLon, DestLat, DestLon);
		angle2 = courseTo(
			DestLat, DestLon, GOAL_LAT, GOAL_LON);

		angle3 = angle1 - angle2;

		rundata[3] = angle3;

		error = angle3 - angle;

		controlValue = PIDcontrol(0, error);
		if (controlValue >= 77) controlValue = 77;
		if (controlValue <= -77) controlValue = -77;
		motor_control(75 - controlValue, 75 + controlValue);
		int motorL = 75 - controlValue;
		int motorR = 75 + controlValue;
		xbeePrintf("motor_control(%d,%d)", motorL, motorR);
		printf("motor_control(%d,%d)", motorL, motorR);
		OriginLat = DestLat;
		OriginLon = DestLon;

	}
}

