#ifndef CMN_TYPE_H
#define CMN_TYPE_H

#include <string.h>
#include <stdint.h>
#include "lib_export.h"

#ifndef ERR_MSG_LEN
#define ERR_MSG_LEN 512
#endif // !ERR_MSG_LEN
#ifndef SENSOR_NAME_LEN
#define SENSOR_NAME_LEN 256
#endif // !SENSOR_NAME_LEN
#ifndef SENSOR_ADR_LEN
#define SENSOR_ADR_LEN 128
#endif // !SENSOR_ADR_LEN
#ifndef SENSOR_SN_LEN
#define SENSOR_SN_LEN 128
#endif // !SENSOR_SN_LEN
#ifndef SENSOR_CHANNEL_NAME_LEN
#define SENSOR_CHANNEL_NAME_LEN 8
#endif // !SENSOR_CHANNEL_NAME_LEN
#ifndef NEURO_EEG_MAX_CH_COUNT
#define NEURO_EEG_MAX_CH_COUNT 24
#endif // !NEURO_EEG_MAX_CH_COUNT
#ifndef FILE_NAME_MAX_LEN
#define FILE_NAME_MAX_LEN 64
#endif // !FILE_NAME_MAX_LEN

#ifndef NEURO_BAM_MAX_CH_COUNT
#define NEURO_BAM_MAX_CH_COUNT 8
#endif // !NEURO_BAM_MAX_CH_COUNT


typedef struct _OpStatus
{
	uint8_t Success;
	uint32_t Error;
	char ErrorMsg[ERR_MSG_LEN];
} OpStatus;

enum SensorFamily : uint8_t
{
	SensorUnknown = 0,
	SensorLECallibri = 1,
	SensorLEKolibri = 2,
	SensorLEBrainBit = 3,
	SensorLEBrainBitBlack = 4,
	SensorLEHeadPhones = 5,
	SensorLEHeadPhones2 = 6,
	SensorLESmartLeg = 7,
	SensorLENeurro = 8,
	SensorLEP300 = 9,
	SensorLEImpulse = 10,
	SensorLEHeadband = 11,
	SensorLEEarBuds = 12,
	SensorSPCompactNeuro = 13,
	SensorLENeuroEEG = 14,
	SensorLECallibriNext = 15,
	SensorLENeuroBAM = 16,
};

typedef struct _SensorVersion {
	uint32_t FwMajor;
	uint32_t FwMinor;
	uint32_t FwPatch;

	uint32_t HwMajor;
	uint32_t HwMinor;
	uint32_t HwPatch;

	uint32_t ExtMajor;
} SensorVersion;

typedef struct _SensorInfo
{
	enum SensorFamily SensFamily;
	uint8_t SensModel;
	char Name[SENSOR_NAME_LEN];
	char Address[SENSOR_ADR_LEN];
	char SerialNumber[SENSOR_SN_LEN];
	uint8_t PairingRequired;
} SensorInfo;

enum SensorFeature : int8_t
{
	FeatureSignal,
	FeatureMEMS,
	FeatureCurrentStimulator,
	FeatureRespiration,
	FeatureResist,
	FeatureFPG,
	FeatureEnvelope,
	FeaturePhotoStimulator,
	FeatureAcousticStimulator,
	FeatureFlashCard
};

enum SensorFirmwareMode : int8_t {
	ModeBootloader,
	ModeApplication
};

enum SensorCommand : int8_t
{
	CommandStartSignal,
	CommandStopSignal,
	CommandStartResist,
	CommandStopResist,
	CommandStartMEMS,
	CommandStopMEMS,
	CommandStartRespiration,
	CommandStopRespiration,
	CommandStartCurrentStimulation,
	CommandStopCurrentStimulation,
	CommandEnableMotionAssistant,
	CommandDisableMotionAssistant,
	CommandFindMe,
	CommandStartAngle,
	CommandStopAngle,
	CommandCalibrateMEMS,
	CommandResetQuaternion,
	CommandStartEnvelope,
	CommandStopEnvelope,
	CommandResetMotionCounter,
	CommandCalibrateStimulation,
	CommandIdle,
	CommandPowerDown,
	CommandStartFPG,
	CommandStopFPG,
	CommandStartSignalAndResist,
	CommandStopSignalAndResist,
	CommandStartPhotoStimulation,
	CommandStopPhotoStimulation,
	CommandStartAcousticStimulation,
	CommandStopAcousticStimulation,
	CommandFileSystemEnable,
	CommandFileSystemDisable,
	CommandFileSystemStreamClose,
	CommandStartCalibrateSignal,
	CommandStopCalibrateSignal
};

enum SensorParameter : int8_t {
	ParameterName,
	ParameterState,
	ParameterAddress,
	ParameterSerialNumber,
	ParameterHardwareFilterState,
	ParameterFirmwareMode,
	ParameterSamplingFrequency,
	ParameterGain,
	ParameterOffset,
	ParameterExternalSwitchState,
	ParameterADCInputState,
	ParameterAccelerometerSens,
	ParameterGyroscopeSens,
	ParameterStimulatorAndMAState,
	ParameterStimulatorParamPack,
	ParameterMotionAssistantParamPack,
	ParameterFirmwareVersion,
	ParameterMEMSCalibrationStatus,
	ParameterMotionCounterParamPack,
	ParameterMotionCounter,
	ParameterBattPower,
	ParameterSensorFamily,
	ParameterSensorMode,
	ParameterIrAmplitude,
	ParameterRedAmplitude,
	ParameterEnvelopeAvgWndSz,
	ParameterEnvelopeDecimation,
	ParameterSamplingFrequencyResist,
	ParameterSamplingFrequencyMEMS,
	ParameterSamplingFrequencyFPG,
	ParameterAmplifier,
	ParameterSensorChannels,
	ParameterSamplingFrequencyResp,
	ParameterSurveyId,
	ParameterFileSystemStatus,
	ParameterFileSystemDiskInfo,
	ParameterReferentsShort,
	ParameterReferentsGround,
	ParameterSamplingFrequencyEnvelope,
	ParameterChannelConfiguration,
	ParameterElectrodeState
};

enum SensorParamAccess : int8_t {
	ParamAccessRead,
	ParamAccessReadWrite,
	ParamAccessReadNotify
};

typedef struct _ParameterInfo {
	enum SensorParameter Param;
	enum SensorParamAccess ParamAccess;
} ParameterInfo;

enum SensorState : int8_t {
	StateInRange,
	StateOutOfRange
};

enum SensorSamplingFrequency : int8_t {
	FrequencyHz10,
	FrequencyHz20,
	FrequencyHz100,
	FrequencyHz125,
	FrequencyHz250,
	FrequencyHz500,
	FrequencyHz1000,
	FrequencyHz2000,
	FrequencyHz4000,
	FrequencyHz8000,
	FrequencyUnsupported
};

enum SensorGain : int8_t {
	SensorGain1,
	SensorGain2,
	SensorGain3,
	SensorGain4,
	SensorGain6,
	SensorGain8,
	SensorGain12,
	SensorGain24,
	SensorGain5,
	SensorGain2x,
	SensorGain4x,
	SensorGainUnsupported
};

enum SensorDataOffset : uint8_t {
	DataOffset0 = 0x00,
	DataOffset1 = 0x01,
	DataOffset2 = 0x02,
	DataOffset3 = 0x03,
	DataOffset4 = 0x04,
	DataOffset5 = 0x05,
	DataOffset6 = 0x06,
	DataOffset7 = 0x07,
	DataOffset8 = 0x08,
	DataOffsetUnsupported = 0xFF
};

enum SensorFilter : uint16_t {
	FilterHPFBwhLvl1CutoffFreq1Hz,
	FilterHPFBwhLvl1CutoffFreq5Hz,
	FilterBSFBwhLvl2CutoffFreq45_55Hz,
	FilterBSFBwhLvl2CutoffFreq55_65Hz,
	FilterHPFBwhLvl2CutoffFreq10Hz,
	FilterLPFBwhLvl2CutoffFreq400Hz,
	FilterHPFBwhLvl2CutoffFreq80Hz,
	FilterUnknown = 0xFF
};

enum CallibriColorType
{
	CallibriColorRed,
	CallibriColorYellow,
	CallibriColorBlue,
	CallibriColorWhite,

	CallibriColorUnknown
};

enum CallibriElectrodeState : uint8_t {
	ElStNormal,
	ElStHighResistance,
	ElStDetached
};

enum SensorExternalSwitchInput : uint8_t {
	ExtSwInElectrodesRespUSB,
	ExtSwInElectrodes,
	ExtSwInUSB,
	ExtSwInRespUSB,
	ExtSwInShort,
	ExtSwInUnknown = 0xFF
};

typedef struct _BrainBitSignalData {
	uint32_t PackNum;
	uint8_t Marker;
	double O1;
	double O2;
	double T3;
	double T4;
} BrainBitSignalData;

typedef struct _BrainBitResistData {
	double O1;
	double O2;
	double T3;
	double T4;
} BrainBitResistData;

typedef void* BrainBitSignalDataListenerHandle;
typedef void* BrainBitResistDataListenerHandle;

typedef struct _HeadbandSignalData {
	uint32_t PackNum;
	uint8_t Marker;
	double O1;
	double O2;
	double T3;
	double T4;
} HeadbandSignalData;

typedef struct _HeadbandResistData {
	uint32_t PackNum;
	double O1;
	double O2;
	double T3;
	double T4;
} HeadbandResistData;

typedef void* HeadbandSignalDataListenerHandle;
typedef void* HeadbandResistDataListenerHandle;

enum IrAmplitude : uint8_t {
	IrAmp0 = 0,
	IrAmp14 = 1,
	IrAmp28 = 2,
	IrAmp42 = 3,
	IrAmp56 = 4,
	IrAmp70 = 5,
	IrAmp84 = 6,
	IrAmp100 = 7,
	IrAmpUnsupported = 0xFF
};

enum RedAmplitude : uint8_t {
	RedAmp0 = 0,
	RedAmp14 = 1,
	RedAmp28 = 2,
	RedAmp42 = 3,
	RedAmp56 = 4,
	RedAmp70 = 5,
	RedAmp84 = 6,
	RedAmp100 = 7,
	RedAmpUnsupported = 0xFF
};

enum GenCurrent : uint8_t {
	GenCurr0nA = 0,
	GenCurr6nA = 1,
	GenCurr12nA = 2,
	GenCurr18nA = 3,
	GenCurr24nA = 4,
	GenCurr6uA = 5,
	GenCurr24uA = 6,
	Unsupported = 0xFF
};

typedef struct _HeadphonesSignalData {
	uint32_t PackNum;
	uint8_t Marker;
	double Ch1;
	double Ch2;
	double Ch3;
	double Ch4;
	double Ch5;
	double Ch6;
	double Ch7;
} HeadphonesSignalData;

typedef struct _HeadphonesResistData {
	uint32_t PackNum;
	double Ch1;
	double Ch2;
	double Ch3;
	double Ch4;
	double Ch5;
	double Ch6;
	double Ch7;
} HeadphonesResistData;

typedef struct _HeadphonesAmplifierParam {
	uint8_t ChSignalUse1;
	uint8_t ChSignalUse2;
	uint8_t ChSignalUse3;
	uint8_t ChSignalUse4;
	uint8_t ChSignalUse5;
	uint8_t ChSignalUse6;
	uint8_t ChSignalUse7;

	uint8_t ChResistUse1;
	uint8_t ChResistUse2;
	uint8_t ChResistUse3;
	uint8_t ChResistUse4;
	uint8_t ChResistUse5;
	uint8_t ChResistUse6;
	uint8_t ChResistUse7;

	SensorGain ChGain1;
	SensorGain ChGain2;
	SensorGain ChGain3;
	SensorGain ChGain4;
	SensorGain ChGain5;
	SensorGain ChGain6;
	SensorGain ChGain7;

	GenCurrent Current;
} HeadphonesAmplifierParam;

typedef void* HeadphonesSignalDataListenerHandle;
typedef void* HeadphonesResistDataListenerHandle;

typedef struct _Headphones2SignalData {
	uint32_t PackNum;
	uint8_t Marker;
	double Ch1;
	double Ch2;
	double Ch3;
	double Ch4;
} Headphones2SignalData;

typedef struct _Headphones2ResistData {
	uint32_t PackNum;
	double Ch1;
	double Ch2;
	double Ch3;
	double Ch4;
} Headphones2ResistData;

typedef struct _Headphones2AmplifierParam {
	uint8_t ChSignalUse1;
	uint8_t ChSignalUse2;
	uint8_t ChSignalUse3;
	uint8_t ChSignalUse4;

	uint8_t ChResistUse1;
	uint8_t ChResistUse2;
	uint8_t ChResistUse3;
	uint8_t ChResistUse4;

	SensorGain ChGain1;
	SensorGain ChGain2;
	SensorGain ChGain3;
	SensorGain ChGain4;

	GenCurrent Current;
} Headphones2AmplifierParam;

typedef void* Headphones2SignalDataListenerHandle;
typedef void* Headphones2ResistDataListenerHandle;

enum SensorAmpMode : uint8_t
{
	Invalid,
	PowerDown,
	Idle,
	Signal,
	Resist,
	SignalResist,
	Envelope
};

typedef void* AmpModeListenerHandle;

typedef struct _FPGData {
	uint32_t PackNum;
	double IrAmplitude;
	double RedAmplitude;
} FPGData;

typedef void* FPGDataListenerHandle;

enum SensorAccelerometerSensitivity : int8_t {
	AccSens2g,
	AccSens4g,
	AccSens8g,
	AccSens16g,
	AccSensUnsupported
};

enum SensorGyroscopeSensitivity : int8_t {
	GyroSens250Grad,
	GyroSens500Grad,
	GyroSens1000Grad,
	GyroSens2000Grad,
	GyroSensUnsupported
};

typedef struct _MEMSData {
	uint32_t PackNum;
	struct Accelerometer {
		double X;
		double Y;
		double Z;
	} Accelerometer;

	struct Gyroscope {
		double X;
		double Y;
		double Z;
	} Gyroscope;
} MEMSData;

typedef void* MEMSDataListenerHandle;

enum EEGChannelType : uint8_t
{
	EEGChTypeSingleA1,
	EEGChTypeSingleA2,
	EEGChTypeDifferential,
	EEGChTypeRef
};

enum EEGChannelId : uint8_t
{
	EEGChIdUnknown,
	EEGChIdO1,
	EEGChIdP3,
	EEGChIdC3,
	EEGChIdF3,
	EEGChIdFp1,
	EEGChIdT5,
	EEGChIdT3,
	EEGChIdF7,

	EEGChIdF8,
	EEGChIdT4,
	EEGChIdT6,
	EEGChIdFp2,
	EEGChIdF4,
	EEGChIdC4,
	EEGChIdP4,
	EEGChIdO2,

	EEGChIdD1,
	EEGChIdD2,
	EEGChIdOZ,
	EEGChIdPZ,
	EEGChIdCZ,
	EEGChIdFZ,
	EEGChIdFpZ,
	EEGChIdD3
};

typedef struct _EEGChannelInfo
{
	EEGChannelId Id;
	EEGChannelType ChType;
	char Name[SENSOR_CHANNEL_NAME_LEN];
	uint8_t Num;
} EEGChannelInfo;

enum EEGChannelMode : uint8_t {
	EEGChModeOff,
	EEGChModeShorted,
	EEGChModeSignalResist,
	EEGChModeSignal,
	EEGChModeTest
};

typedef struct _SignalChannelsData {
	uint32_t PackNum;
	uint8_t Marker;
	uint32_t SzSamples;
	double* Samples;
} SignalChannelsData;


typedef struct _CompactNeuroStimulParam
{
	double Freq;
	int32_t PulseWidthMs;
	double FillingFreq;
	uint8_t Power;
	uint16_t Count;
} CompactNeuroStimulParam;

enum CompactNeuroSignalMarker : uint8_t
{
	None = 0x00,
	PhotoStimul = 0x01,
	AcousticStimul = 0x02,
	LostFrame = 0x80
};

typedef struct _CompactNeuroSignalData
{
	uint32_t PackNum;
	CompactNeuroSignalMarker Marker;
	double O1;
	double P3;
	double C3;
	double F3;
	double Fp1;
	double T5;
	double T3;
	double F7;

	double F8;
	double T4;
	double T6;
	double Fp2;
	double F4;
	double C4;
	double P4;
	double O2;

	double D1;
	double D2;
	double OZ;
	double PZ;
	double CZ;
	double FZ;
	double FpZ;
	double D3;
} CompactNeuroSignalData;

typedef struct _CompactNeuroResistData
{
	double O1;
	double P3;
	double C3;
	double F3;
	double Fp1;
	double T5;
	double T3;
	double F7;

	double F8;
	double T4;
	double T6;
	double Fp2;
	double F4;
	double C4;
	double P4;
	double O2;

	double D1;
	double D2;
	double OZ;
	double PZ;
	double CZ;
	double FZ;
	double FpZ;
	double D3;
} CompactNeuroResistData;



typedef void* CompactNeuroSignalDataListenerHandle;
typedef void* CompactNeuroResistDataListenerHandle;

enum EEGRefMode : uint8_t
{
	RefHeadTop = 1,
	RefA1A2
};

typedef struct _NeuroEEGAmplifierParam {
	uint8_t ReferentResistMesureAllow;
	SensorSamplingFrequency Frequency;
	EEGRefMode ReferentMode;
	EEGChannelMode ChannelMode[NEURO_EEG_MAX_CH_COUNT];
	SensorGain ChannelGain[NEURO_EEG_MAX_CH_COUNT];
} NeuroEEGAmplifierParam;

enum SensorFSStatus : uint8_t
{
	FSStatusOK,
	FSStatusNoInit,
	FSStatusNoDisk,
	FSStatusProtect
};

enum SensorFSIOStatus : uint8_t
{
	FSIOStatusNoError,
	FSIOStatusIOError,
	FSIOStatusTimeout
};

enum SensorFSStreamStatus : uint8_t
{
	FSStreamStatusClosed,
	FSStreamStatusWrite,
	FSStreamStatusRead
};

typedef struct _NeuroEEGFSStatus {
	SensorFSStatus Status;
	SensorFSIOStatus IOStatus;
	SensorFSStreamStatus StreamStatus;
	uint8_t AutosaveSignal;
} NeuroEEGFSStatus;

typedef struct _SensorFileInfo {
	char FileName[FILE_NAME_MAX_LEN];
	uint32_t FileSize;
	uint16_t ModifiedYear;
	uint8_t ModifiedMonth;
	uint8_t ModifiedDayOfMonth;
	uint8_t ModifiedHour;
	uint8_t ModifiedMin;
	uint8_t ModifiedSec;
	uint8_t Attribute;
} SensorFileInfo;

typedef struct _SensorFileData {
	uint32_t OffsetStart;
	uint32_t DataAmount;
	uint32_t SzData;
	uint8_t* Data;
} SensorFileData;

typedef struct _SensorDiskInfo {
	uint64_t TotalSize;
	uint64_t FreeSize;
} SensorDiskInfo;

typedef struct _ResistChannelsData {
	uint32_t PackNum;
	double A1;
	double A2;
	double Bias;
	uint32_t SzValues;
	double* Values;
} ResistChannelsData;

typedef void* NeuroEEGSignalDataListenerHandle;
typedef void* NeuroEEGResistDataListenerHandle;
typedef void* NeuroEEGSignalResistDataListenerHandle;
typedef void* NeuroEEGSignalRawDataListenerHandle;
typedef void* NeuroEEGFileStreamDataListenerHandle;
typedef void* NeuroEEGSignalProcessParam;

typedef struct _NeuroBAMAmplifierParam {
	SensorSamplingFrequency Frequency;
	EEGChannelMode ChannelMode[NEURO_BAM_MAX_CH_COUNT];
	SensorGain ChannelGain[NEURO_BAM_MAX_CH_COUNT];
} NeuroBAMAmplifierParam;

typedef struct _NeuroBAMResistChannelsData {
	uint32_t PackNum;
	double Fp1;
	double Fp2;
	uint32_t SzValues;
	double* Values;
} NeuroBAMResistChannelsData;

typedef void* NeuroBAMSignalDataListenerHandle;
typedef void* NeuroBAMResistDataListenerHandle;
typedef void* NeuroBAMSignalResistDataListenerHandle;

enum SensorADCInput : int8_t {
	ADCInputElectrodes,
	ADCInputShort,
	ADCInputTest,
	ADCInputResistance
};

enum CallibriStimulatorState : uint8_t {
	StimStateNoParams = 0,
	StimStateDisabled = 1,
	StimStateEnabled = 2,
	StimStateUnsupported = 0xFF
};

typedef struct _CallibriStimulatorMAState {
	enum CallibriStimulatorState StimulatorState;
	enum CallibriStimulatorState MAState;
} CallibriStimulatorMAState;

/// <summary>
/// Stimulator parameters</br>
/// Limitations:</br>
/// (Current * Frequency * PulseWidth / 100) <= 2300 uA
/// </summary>
typedef struct _CallibriStimulationParams {
	/// <summary>
	/// Stimulus amplitude in  mA. 1..100
	/// </summary>
	uint8_t Current;
	/// <summary>
	/// Duration of the stimulating pulse by us. 20..460
	/// </summary>
	uint16_t PulseWidth;
	/// <summary>
	/// Frequency of stimulation impulses by Hz. 1..200.
	/// </summary>
	uint8_t Frequency;
	/// <summary>
	/// Maximum stimulation time by ms. 0...65535.</br>
	/// Zero is infinitely.
	/// </summary>
	uint16_t StimulusDuration;
} CallibriStimulationParams;

enum CallibriMotionAssistantLimb : uint8_t {
	MALimbRightLeg = 0,
	MALimbLeftLeg = 1,
	MALimbRightArm = 2,
	MALimbLeftArm = 3,
	MALimbUnsupported = 0xFF
};

typedef struct _CallibriMotionAssistantParams {
	uint8_t GyroStart;
	uint8_t GyroStop;
	enum CallibriMotionAssistantLimb Limb;
	/// <summary>
	/// multiple of 10. This means that the device is using the (MinPauseMs / 10) value.;</br>
	/// Correct values: 10, 20, 30, 40 ... 
	/// </summary>
	uint8_t MinPauseMs;
} CallibriMotionAssistantParams;

typedef struct _CallibriMotionCounterParam {
	/// <summary>
	/// Insense threshold mg. 0..500
	/// </summary>
	uint16_t InsenseThresholdMG;
	/// <summary>
	/// Algorithm insense threshold in time (in samples with the MEMS sampling rate) 0..500
	/// </summary>
	uint16_t InsenseThresholdSample;
} CallibriMotionCounterParam;

typedef struct _CallibriSignalData {
	uint32_t PackNum;
	double* Samples;
	uint32_t SzSamples;
} CallibriSignalData;

typedef struct _CallibriRespirationData {
	uint32_t PackNum;
	double* Samples;
	uint32_t SzSamples;
} CallibriRespirationData;

typedef struct _QuaternionData {
	uint32_t PackNum;
	float W;
	float X;
	float Y;
	float Z;
} QuaternionData;

typedef struct _CallibriEnvelopeData {
	uint32_t PackNum;
	double Sample;
} CallibriEnvelopeData;

enum SignalTypeCallibri : uint8_t
{
	CallibriSignalTypeEEG = 0,
	CallibriSignalTypeEMG = 1,
	CallibriSignalTypeECG = 2,
	CallibriSignalTypeEDA = 3,// GSR
	CallibriSignalTypeStrainGaugeBreathing = 4,
	CallibriSignalTypeImpedanceBreathing = 5,
	CallibriSignalTypeUnknown = 6
};

typedef void* CallibriSignalDataListenerHandle;
typedef void* CallibriRespirationDataListenerHandle;
typedef void* CallibriElectrodeStateListenerHandle;
typedef void* CallibriEnvelopeDataListenerHandle;
typedef void* QuaternionDataListenerHandle;

 
enum CallibriNextChCfg : uint8_t
{
	ChCfgRectification = 0x01,
	ChCfgTest = 0x02,
	ChCfgUnknown = 0xFF
};
typedef struct _CallibriNextChannelData {
	uint32_t PackNum;
	uint32_t SzSamples;
	double* Samples;
} CallibriNextChannelData;

typedef void* CallibriNextChannelDataListenerHandle;
typedef void* CallibriNextElectrodeStateListenerHandle;


typedef struct _SensorScanner SensorScanner;
typedef struct _Sensor Sensor;

typedef void* SensorsListenerHandle;
typedef void* BattPowerListenerHandle;
typedef void* SensorStateListenerHandle;


#endif // CMN_TYPE_H
