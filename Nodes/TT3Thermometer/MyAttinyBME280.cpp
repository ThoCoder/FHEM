//MyAttinyBME280.cpp
//Code fuer Attiny
//Author Retian
//Version 1.0

/*
Abfrage eines Luftdruck-/Luftfeuchtesensors BME280 ueber I2C-Schnittstelle.

MyAttinyBME280 Name(i2cAdd);

Beispiel siehe unter:
http://arduino-projekte.webnode.at/meine-libraries/luftdruck-luftfeuchtesensor-bme280/

Funktionen siehe unter:
http://arduino-projekte.webnode.at/meine-libraries/luftdruck-luftfeuchtesensor-bme280/funktionen/

*/

//*************************************************************************

#include "Arduino.h"
//#include "TinyWireM.h"
#include <SoftwareI2C.h>
#include "MyAttinyBME280.h"

SoftwareI2C TinyWireM;

MyAttinyBME280::MyAttinyBME280(byte i2cAdd, byte _sda, byte _scl)
{
	_i2cAdd = i2cAdd;

	TinyWireM.begin(_sda, _scl);
	setDefaultParameters();
}

//*************************************************************************
//Setzen der Standard-Parameter
//(Interne Verwendung)

void MyAttinyBME280::setDefaultParameters()
{
	controlVal = 0;
	configVal = 0;
	controlHumVal = 0;

	//Setzen der Voreinstellungen
	mode = NORMAL_MODE;
	setMode(mode);

	osrs_p = P_OVERSAMPLING_x8; //Pressure oversampling x8
	setPressOversampling(osrs_p);

	osrs_t = T_OVERSAMPLING_x2; //Temperature oversampling x2
	setTempOversampling(osrs_t);

	osrs_h = H_OVERSAMPLING_x8; //Humidity oversampling x8
	setHumidityOversampling(osrs_h);

	t_sb = STANDBY_TIME_1000; //Standby time (NORMAL_MODE) = 1000 ms
	setStandbyTime(t_sb);

	filter = FILTER_COEFF_4; //Filter-Koeffizient = 4 (5 Stufen fuer 75%)
	setFilterCoeff(filter);
}


//*************************************************************************
//BME280 vorhanden ?

bool MyAttinyBME280::isReady()
{
	if (read8(BME280_register_chip_id) == chip_ID) return true;
	else return false;
}

//*************************************************************************
//BME280 initialisiern
void MyAttinyBME280::init()
{
	//BME280 zuruecksetzen
	reset();
	delay(300);

	//Kalibrierungskoeffizienten einlesen

	//Wenn der BME280 gerade die Koeffizienten ins interne
	//Image-Register liest, dann warte
	while (read8(BME280_register_status) & 0x01) delay(1);

	dig_T1 = (unsigned int)read16(BME280_register_dig_T1);
	dig_T2 = read16(BME280_register_dig_T2);
	dig_T3 = read16(BME280_register_dig_T3);
	dig_P1 = (unsigned int)read16(BME280_register_dig_P1);
	dig_P2 = read16(BME280_register_dig_P2);
	dig_P3 = read16(BME280_register_dig_P3);
	dig_P4 = read16(BME280_register_dig_P4);
	dig_P5 = read16(BME280_register_dig_P5);
	dig_P6 = read16(BME280_register_dig_P6);
	dig_P7 = read16(BME280_register_dig_P7);
	dig_P8 = read16(BME280_register_dig_P8);
	dig_P9 = read16(BME280_register_dig_P9);
	dig_H1 = (uint8_t)read8(BME280_register_dig_H1);
	dig_H2 = read16(BME280_register_dig_H2);
	dig_H3 = (uint8_t)read8(BME280_register_dig_H3);
	dig_H4 = (read8(BME280_register_dig_H4) << 4) |
		(read8(BME280_register_dig_H4 + 1) & 0xF);
	dig_H5 = (read8(BME280_register_dig_H5 + 1) << 4) |
		(read8(BME280_register_dig_H5) >> 4);
	dig_H6 = (uint8_t)read8(BME280_register_dig_H6);

	//Schreiben der gesetzten Parameter in die Register
	//Aenderungen beim Humidity-Controll-Register werden erst nach einer
	//Schreiboperation auf das Control-Register wirksam (lt. Datenblatt 5.4.3)
	write8(BME280_register_config, configVal);
	write8(BME280_register_humidity_control, controlHumVal);
	write8(BME280_register_control, controlVal);
}

//*************************************************************************
//Starte Einzelmessung (FORCED_MODE)

void MyAttinyBME280::startSingleMeas()
{
	controlVal &= B11111100;
	controlVal |= FORCED_MODE;
	write8(BME280_register_control, controlVal);
	while (statusMeas()); //Warte bis Messzyklus beendet
}

//*************************************************************************
//Ermittlung der Lufttemperatur

int32_t MyAttinyBME280::readTemp10()
{
	readRawData();
	if (adc_T == 0x80000)  //Rueckgabe, wenn Temperaturmesseung ausgeschaltet ist
		return -1;
	return calcTemp10();
}

//*************************************************************************
//Ermittlung des Luftdrucks

int32_t MyAttinyBME280::readPress10()
{
	readRawData();
	if (adc_P == 0x80000) //Rueckgabe, wenn Luftdruckmessung ausgeschaltet ist
		return -1;
	calcTemp10(); //calcPress() benoetigt t_fine aus calcTemp()
	return calcPress10();
}

//*************************************************************************
//Ermittlung der Luftfeuchtigkeit

int32_t MyAttinyBME280::readHumidity10()
{
	readRawData();
	if (adc_H == 0x8000) //Rueckgabe, wenn Luftfeuchtemessung ausgeschaltet ist
		return -1;
	calcTemp10(); //calcHumidity() benoetigt t_fine aus calcTemp()
	return calcHumidity10();
}

//*************************************************************************
void MyAttinyBME280::readAll(int16_t& temperature, int16_t& pressure, int16_t& humidity)
{
	readRawData();
	temperature = calcTemp10();
	humidity = calcHumidity10();
	pressure = calcPress10();
//	pressure = calcPress10_i32();
}

//*************************************************************************
//Einlesen von 8 Bytes (burst read) ueber die I2C-Schnittstelle
//(Interne Verwendung)

void MyAttinyBME280::readRawData()
{
	while (statusMeas()); //Warte bis Messzyklus beendet
	TinyWireM.beginTransmission(_i2cAdd);
	TinyWireM.write(BME280_register_data);
	TinyWireM.endTransmission();
	TinyWireM.requestFrom(_i2cAdd, (byte)8);
	adc_P = TinyWireM.read();
	adc_P <<= 8;
	adc_P |= TinyWireM.read();
	adc_P <<= 8;
	adc_P |= TinyWireM.read();
	adc_P >>= 4;

	adc_T = TinyWireM.read();
	adc_T <<= 8;
	adc_T |= TinyWireM.read();
	adc_T <<= 8;
	adc_T |= TinyWireM.read();
	adc_T >>= 4;

	adc_H = TinyWireM.read();
	adc_H <<= 8;
	adc_H |= TinyWireM.read();
}

//*************************************************************************
//Temperatur ermitteln
//(Interne Verwendung)

int32_t MyAttinyBME280::calcTemp10()
{
	int32_t var1, var2;
	int32_t temp;

	//Berechne Temperatur
	var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11;
	var2 = (((((adc_T >> 4) - (dig_T1)) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;
	t_fine = var1 + var2;
	temp = (t_fine * 5 + 128) / 2560;
	return temp;
}

//*************************************************************************
//Luftdruck ermitteln
//(Interne Verwendung)

int32_t MyAttinyBME280::calcPress10()
{
	int64_t var1, var2;
	int64_t press;

	//Berechne Luftdruck
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 + (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) return 0;
	press = 1048576 - adc_P;
	press = (((press << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (press >> 13) * (press >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * press) >> 19;
	press = ((press + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	press /= 2560;
	return press;
}

int32_t MyAttinyBME280::calcPress10_i32()
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t)t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)dig_P6);
	var2 = var2 + ((var1 * ((int32_t)dig_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)dig_P4) * 65536);
	var3 = (dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)dig_P1)) / 32768;
	/* avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t)((uint32_t)1048576) - adc_P;
		pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((uint32_t)var1);
		else
			pressure = (pressure / (uint32_t)var1) * 2;

		var1 = (((int32_t)dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(pressure / 4)) * ((int32_t)dig_P8)) / 8192;
		pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + dig_P7) / 16));

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	}
	else {
		pressure = pressure_min;
	}

	return pressure;
}

//*************************************************************************
//Luftfeuchte ermitteln
//(Interne Verwendung)

int32_t MyAttinyBME280::calcHumidity10()
{
	int32_t v_x1_u32r;
	uint32_t humidity;

	//Berechne Luftfeuchtigkeit
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
		((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
		(((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
			((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
		((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	humidity = (uint32_t)(v_x1_u32r >> 12);
	return (humidity * 5) >> 9;
}

//*************************************************************************
// Schreiben von 1 Byte ueber die I2C-Schnittstelle
//(Interne Verwendung)

void MyAttinyBME280::write8(byte add, byte val)
{
	byte _add = add;
	byte _val = val;

	TinyWireM.beginTransmission(_i2cAdd);
	TinyWireM.write(_add);
	TinyWireM.write(_val);
	TinyWireM.endTransmission();
	delay(5);
}

//*************************************************************************
// Einlesen von 1 Byte ueber die I2C-Schnittstelle
//(Interne Verwendung)

byte MyAttinyBME280::read8(byte adr)
{
	byte _adr = adr;
	byte val;

	TinyWireM.beginTransmission(_i2cAdd);
	TinyWireM.write(_adr);
	TinyWireM.endTransmission();
	TinyWireM.requestFrom(_i2cAdd, (byte)1);
	val = TinyWireM.read();
	return val;
}

//*************************************************************************
// Einlesen von 2 Bytes ueber die I2C-Schnittstelle 
//(Interne Verwendung)       

int MyAttinyBME280::read16(byte adr)
{
	byte _adr = adr;
	int val;

	TinyWireM.beginTransmission(_i2cAdd);
	TinyWireM.write(_adr);
	TinyWireM.endTransmission();
	TinyWireM.requestFrom(_i2cAdd, (byte)2);
	val = TinyWireM.read() | (TinyWireM.read() << 8);
	return val;
}

//*************************************************************************
//Setzen des Modus

void MyAttinyBME280::setMode(byte mode)
{
	byte _mode = mode;

	controlVal &= B11111100; //Loesche mode 
	controlVal |= _mode;
}

//*************************************************************************
//Setzen Temperatur-Oversampling

void MyAttinyBME280::setTempOversampling(byte osrs_t)
{
	byte _osrs_t = osrs_t;

	controlVal &= ~(B111 << 5); //Loesche Temp.oversampling
	controlVal |= (_osrs_t << 5);
}

//*************************************************************************
//Setzen Druck-Oversampling

void MyAttinyBME280::setPressOversampling(byte osrs_p)
{
	byte _osrs_p = osrs_p;

	controlVal &= ~(B111 << 2); //Loesche Press.oversampling
	controlVal |= (_osrs_p << 2);
}

//*************************************************************************
//Setzen Feuchte-Oversampling

void MyAttinyBME280::setHumidityOversampling(byte osrs_h)
{
	byte _osrs_h = osrs_h;

	controlHumVal &= ~(B111); //Loesche Humid.oversampling
	controlHumVal |= _osrs_h;
}

//*************************************************************************
//Setzen der Standby-Zeit (Normal-Modus)

void MyAttinyBME280::setStandbyTime(byte t_sb)
{
	byte _t_sb = t_sb;

	configVal &= ~(B111 << 5); //Loesche Standby Time
	configVal |= (_t_sb << 5);
}

//*************************************************************************
//Setzen des Filterkoeffizienten

void MyAttinyBME280::setFilterCoeff(byte filter)
{
	byte _filter = filter;

	configVal &= ~(B111 << 2); //Loesche Filter
	configVal |= (_filter << 2);
}

//*************************************************************************
//Abfrage des Controll-Register 0xF4

byte MyAttinyBME280::readControlReg(void)
{
	return read8(BME280_register_control);
}

//*************************************************************************
//Abfrage des Config-Register 0xF5

byte MyAttinyBME280::readConfigReg(void)
{
	return read8(BME280_register_config);
}

//*************************************************************************
//Abfrage des Humidity-Control-Register 0xF2

byte MyAttinyBME280::readHumidityControlReg(void)
{
	return read8(BME280_register_humidity_control);
}

//*************************************************************************
//Abfrage des Status des Measuring-Bit
//(Interne Verwendung)

bool MyAttinyBME280::statusMeas()
{
	byte reg;

	reg = read8(BME280_register_status);
	return (reg & B00001000) >> 3;
}

//*************************************************************************
//Software-Reset
//(Interne Verwendung)

void MyAttinyBME280::reset()
{
	write8(BME280_register_reset, RESET);
}

//*************************************************************************
//Voreinstellung fuer Weather Monitoring

void MyAttinyBME280::setWeatherMonitoring()
{
	mode = FORCED_MODE;
	setMode(mode);

	osrs_p = P_OVERSAMPLING_x1; //Pressure oversampling x1
	setPressOversampling(osrs_p);

	osrs_t = T_OVERSAMPLING_x1; //Temperature oversampling x1
	setTempOversampling(osrs_t);

	osrs_h = H_OVERSAMPLING_x1; //Humidity oversampling x1
	setHumidityOversampling(osrs_h);

	/*Keine Bedeutung bei WeatherMonitoring (Forced Mode)
	t_sb =  STANDBY_TIME_62p5;
	setStandbyTime(t_sb);
	*/

	filter = FILTER_COEFF_0; //Filter aus
	setFilterCoeff(filter);
}

//*************************************************************************
//Voreinstellung fuer Humidity Sensing (Luftfeuchtemessung)

void MyAttinyBME280::setHumiditySensing()
{
	mode = FORCED_MODE;
	setMode(mode);

	osrs_p = P_OVERSAMPLING_x0; //Pressure oversampling x0
	setPressOversampling(osrs_p);

	osrs_t = T_OVERSAMPLING_x1; //Temperature oversampling x1
	setTempOversampling(osrs_t);

	osrs_h = H_OVERSAMPLING_x1; //Humidity oversampling x1
	setHumidityOversampling(osrs_h);

	/*Keine Bedeutung bei Humidity Sensing (Forced Mode)
	t_sb =  STANDBY_TIME_62p5;
	setStandbyTime(t_sb);
	*/

	filter = FILTER_COEFF_0; //Filter aus
	setFilterCoeff(filter);
}

//*************************************************************************
//Voreinstellung fuer Indoor Navigation

void MyAttinyBME280::setIndoorNavigation()
{
	mode = NORMAL_MODE;
	setMode(mode);

	osrs_p = P_OVERSAMPLING_x16; //Pressure oversampling x16
	setPressOversampling(osrs_p);

	osrs_t = T_OVERSAMPLING_x2; //Temperature oversampling x2
	setTempOversampling(osrs_t);

	osrs_h = H_OVERSAMPLING_x1; //Humidity oversampling x1
	setHumidityOversampling(osrs_h);

	t_sb = STANDBY_TIME_0p5; //Standby Time 0,5 ms
	setStandbyTime(t_sb);

	filter = FILTER_COEFF_16; //Filter-Koeffizient = 16 (22 Stufen fuer 75%)
	setFilterCoeff(filter);
}

//*************************************************************************
//Voreinstellung fuer Gaming

void MyAttinyBME280::setGaming()
{
	mode = NORMAL_MODE;
	setMode(mode);

	osrs_p = P_OVERSAMPLING_x4; //Pressure oversampling x4
	setPressOversampling(osrs_p);

	osrs_t = T_OVERSAMPLING_x1; //Temperature oversampling x1
	setTempOversampling(osrs_t);

	osrs_h = H_OVERSAMPLING_x0; //Humidity oversampling x0
	setHumidityOversampling(osrs_h);

	t_sb = STANDBY_TIME_0p5; //Standby Time 0,5 ms
	setStandbyTime(t_sb);

	filter = FILTER_COEFF_16; //Filter-Koeffizient = 16 (22 Stufen fuer 75%)
	setFilterCoeff(filter);
}