#ifndef SDK_API_H
#define SDK_API_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmn_type.h"

	SDK_SHARED SensorScanner* createScanner(SensorFamily* filters, int32_t szFilters, OpStatus* outStatus);
	SDK_SHARED void freeScanner(SensorScanner* ptr);
	SDK_SHARED uint8_t startScanner(SensorScanner* ptr, OpStatus* outStatus, int numOfTrying = 1);
	SDK_SHARED uint8_t stopScanner(SensorScanner* ptr, OpStatus* outStatus);
	SDK_SHARED uint8_t sensorsScanner(SensorScanner* ptr, SensorInfo* sensors, int32_t* szSensorsInOut, OpStatus* outStatus);
	SDK_SHARED uint8_t addSensorsCallbackScanner(SensorScanner* ptr, void(*callback)(SensorScanner* ptr, SensorInfo* sensors, int32_t szSensors, void* userData), SensorsListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSensorsCallbackScanner(SensorsListenerHandle handle);


	SDK_SHARED Sensor* createSensor(SensorScanner* ptr, SensorInfo sensor, OpStatus* outStatus);
	SDK_SHARED void freeSensor(Sensor* ptr);

	SDK_SHARED uint8_t connectSensor(Sensor* ptr, OpStatus* outStatus);
	SDK_SHARED uint8_t disconnectSensor(Sensor* ptr, OpStatus* outStatus);

	SDK_SHARED int32_t getFeaturesCountSensor(Sensor* ptr);
	SDK_SHARED uint8_t getFeaturesSensor(Sensor* ptr, SensorFeature* sensorFeatures, int32_t* szSensorFeatureInOut, OpStatus* outStatus);
	SDK_SHARED int8_t isSupportedFeatureSensor(Sensor* ptr, SensorFeature sensorFeature);

	SDK_SHARED int32_t getCommandsCountSensor(Sensor* ptr);
	SDK_SHARED uint8_t getCommandsSensor(Sensor* ptr, SensorCommand* sensorCommands, int32_t* szSensorCommandsInOut, OpStatus* outStatus);
	SDK_SHARED int8_t isSupportedCommandSensor(Sensor* ptr, SensorCommand sensorCommand);

	SDK_SHARED int32_t getParametersCountSensor(Sensor* ptr);
	SDK_SHARED uint8_t getParametersSensor(Sensor* ptr, ParameterInfo* sensorParameters, int32_t* szSensorParametersInOut, OpStatus* outStatus);
	SDK_SHARED int8_t isSupportedParameterSensor(Sensor* ptr, SensorParameter sensorParameter);

	SDK_SHARED int32_t getChannelsCountSensor(Sensor* ptr);

	SDK_SHARED uint8_t execCommandSensor(Sensor* ptr, SensorCommand sensorCommand, OpStatus* outStatus);

	SDK_SHARED SensorFamily getFamilySensor(Sensor* ptr);

	SDK_SHARED uint8_t readNameSensor(Sensor* ptr, char* nameOut, int32_t szNameIn, OpStatus* outStatus);
	SDK_SHARED uint8_t writeNameSensor(Sensor* ptr, char* name, int32_t szName, OpStatus* outStatus);
	SDK_SHARED uint8_t readStateSensor(Sensor* ptr, SensorState* stateOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readAddressSensor(Sensor* ptr, char* addressOut, int32_t szAddressIn, OpStatus* outStatus);
	SDK_SHARED uint8_t readSerialNumberSensor(Sensor* ptr, char* serialNumberOut, int32_t szSerialNumberIn, OpStatus* outStatus);
	SDK_SHARED uint8_t writeSerialNumberSensor(Sensor* ptr, char* serialNumber, int32_t szSerialNumber, OpStatus* outStatus);
	SDK_SHARED uint8_t readBattPowerSensor(Sensor* ptr, int32_t* battPowerOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readSamplingFrequencySensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readGainSensor(Sensor* ptr, SensorGain* gainOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readDataOffsetSensor(Sensor* ptr, SensorDataOffset* dataOffsetOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readFirmwareModeSensor(Sensor* ptr, SensorFirmwareMode* modeOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readVersionSensor(Sensor* ptr, SensorVersion* versionOut, OpStatus* outStatus);

	SDK_SHARED uint8_t readHardwareFiltersSensor(Sensor* ptr, SensorFilter* filtersOut, int32_t* szFiltersInOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeHardwareFiltersSensor(Sensor* ptr, SensorFilter* filters, int32_t szFilters, OpStatus* outStatus);
	SDK_SHARED uint8_t writeGainSensor(Sensor* ptr, SensorGain gain, OpStatus* outStatus);
	SDK_SHARED uint8_t readExternalSwitchSensor(Sensor* ptr, SensorExternalSwitchInput* extSwInputOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeExternalSwitchSensor(Sensor* ptr, SensorExternalSwitchInput extSwInput, OpStatus* outStatus);
	SDK_SHARED uint8_t readColorCallibri(Sensor* ptr, CallibriColorType* callibriColorOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readElectrodeStateCallibri(Sensor* ptr, CallibriElectrodeState* electrodeStateOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readSamplingFrequencyEnvelopeSensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);

	SDK_SHARED int32_t getSupportedFiltersCountSensor(Sensor* ptr);
	SDK_SHARED uint8_t getSupportedFiltersSensor(Sensor* ptr, SensorFilter* sensorFilters, int32_t* szSensorFiltersInOut, OpStatus* outStatus);
	SDK_SHARED int8_t isSupportedFilterSensor(Sensor* ptr, SensorFilter filter);
	SDK_SHARED void readColorInfo(SensorInfo sensorInfo, CallibriColorType* callibriColorOut);

	SDK_SHARED uint8_t readChannelCfgCallibriNext(Sensor* ptr, CallibriNextChCfg* chCfgsOut, int32_t* szchCfgsInOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeChannelCfgCallibriNext(Sensor* ptr, CallibriNextChCfg* chCfgs, int32_t szchCfgs, OpStatus* outStatus);
	SDK_SHARED uint8_t addSignalCallbackCallibriNext(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriNextChannelData* data, int32_t szData, void* userData), CallibriNextChannelDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalCallbackCallibriNext(CallibriNextChannelDataListenerHandle handle);
	SDK_SHARED uint8_t addEnvelopeCallbackCallibriNext(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriNextChannelData* data, int32_t szData, void* userData), CallibriNextChannelDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeEnvelopeCallbackCallibriNext(CallibriNextChannelDataListenerHandle handle);
	SDK_SHARED uint8_t addElectrodeStateCallbackCallibriNext(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriElectrodeState state, void* userData), CallibriNextElectrodeStateListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeElectrodeStateCallbackCallibriNext(CallibriNextElectrodeStateListenerHandle handle);

	SDK_SHARED uint8_t writeFirmwareModeSensor(Sensor* ptr, SensorFirmwareMode mode, OpStatus* outStatus);
	SDK_SHARED uint8_t writeSamplingFrequencySensor(Sensor* ptr, SensorSamplingFrequency samplingFrequency, OpStatus* outStatus);
	SDK_SHARED uint8_t writeDataOffsetSensor(Sensor* ptr, SensorDataOffset dataOffset, OpStatus* outStatus);
	SDK_SHARED uint8_t readADCInputSensor(Sensor* ptr, SensorADCInput* adcInputOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeADCInputSensor(Sensor* ptr, SensorADCInput adcInput, OpStatus* outStatus);

	SDK_SHARED uint8_t readStimulatorAndMAStateCallibri(Sensor* ptr, CallibriStimulatorMAState* stimulatorMAStateOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readStimulatorParamCallibri(Sensor* ptr, CallibriStimulationParams* stimulationParamsOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeStimulatorParamCallibri(Sensor* ptr, CallibriStimulationParams stimulationParams, OpStatus* outStatus);
	SDK_SHARED uint8_t readMotionAssistantParamCallibri(Sensor* ptr, CallibriMotionAssistantParams* motionAssistantParamsOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeMotionAssistantParamCallibri(Sensor* ptr, CallibriMotionAssistantParams motionAssistantParams, OpStatus* outStatus);
	SDK_SHARED uint8_t readMotionCounterParamCallibri(Sensor* ptr, CallibriMotionCounterParam* motionCounterParamOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeMotionCounterParamCallibri(Sensor* ptr, CallibriMotionCounterParam motionCounterParam, OpStatus* outStatus);
	SDK_SHARED uint8_t readMotionCounterCallibri(Sensor* ptr, uint32_t* motionCounterOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readMEMSCalibrateStateCallibri(Sensor* ptr, uint8_t* state, OpStatus* outStatus);
	SDK_SHARED uint8_t readSamplingFrequencyRespSensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);

	SDK_SHARED uint8_t setSignalSettingsCallibri(Sensor* ptr, SignalTypeCallibri signalType, OpStatus* outStatus);
	SDK_SHARED uint8_t getSignalSettingsCallibri(Sensor* ptr, SignalTypeCallibri* signalType, OpStatus* outStatus);

	SDK_SHARED uint8_t addSignalCallbackCallibri(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriSignalData* data, int32_t szData, void* userData), CallibriSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalCallbackCallibri(CallibriSignalDataListenerHandle handle);
	SDK_SHARED uint8_t addRespirationCallbackCallibri(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriRespirationData* data, int32_t szData, void* userData), CallibriRespirationDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeRespirationCallbackCallibri(CallibriRespirationDataListenerHandle handle);
	SDK_SHARED uint8_t addElectrodeStateCallbackCallibri(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriElectrodeState state, void* userData), CallibriElectrodeStateListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeElectrodeStateCallbackCallibri(CallibriElectrodeStateListenerHandle handle);
	SDK_SHARED uint8_t addEnvelopeDataCallbackCallibri(Sensor* ptr, void(*callback)(Sensor* ptr, CallibriEnvelopeData* data, int32_t szData, void* userData), CallibriEnvelopeDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeEnvelopeDataCallbackCallibri(CallibriEnvelopeDataListenerHandle handle);

	SDK_SHARED uint8_t addQuaternionDataCallback(Sensor* ptr, void(*callback)(Sensor* ptr, QuaternionData* data, int32_t szData, void* userData), QuaternionDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeQuaternionDataCallback(QuaternionDataListenerHandle handle);
	SDK_SHARED uint8_t readSamplingFrequencyFPGSensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readIrAmplitudeFPGSensor(Sensor* ptr, IrAmplitude* amplitudeOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeIrAmplitudeFPGSensor(Sensor* ptr, IrAmplitude amplitude, OpStatus* outStatus);
	SDK_SHARED uint8_t readRedAmplitudeFPGSensor(Sensor* ptr, RedAmplitude* amplitudeOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeRedAmplitudeFPGSensor(Sensor* ptr, RedAmplitude amplitude, OpStatus* outStatus);
	SDK_SHARED uint8_t addFPGDataCallback(Sensor* ptr, void(*callback)(Sensor* ptr, FPGData* data, int32_t szData, void* userData), FPGDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeFPGDataCallback(FPGDataListenerHandle handle);
	SDK_SHARED uint8_t readSamplingFrequencyResistSensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readAmplifierParamHeadphones(Sensor* ptr, HeadphonesAmplifierParam* ampParamOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeAmplifierParamHeadphones(Sensor* ptr, HeadphonesAmplifierParam ampParam, OpStatus* outStatus);

	SDK_SHARED uint8_t addResistCallbackHeadphones(Sensor* ptr, void(*callback)(Sensor* ptr, HeadphonesResistData* data, int32_t szData, void* userData), HeadphonesResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackHeadphones(HeadphonesResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalDataCallbackHeadphones(Sensor* ptr, void(*callback)(Sensor* ptr, HeadphonesSignalData* data, int32_t szData, void* userData), HeadphonesSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalDataCallbackHeadphones(HeadphonesSignalDataListenerHandle handle);
	SDK_SHARED uint8_t readAmplifierParamHeadphones2(Sensor* ptr, Headphones2AmplifierParam* ampParamOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeAmplifierParamHeadphones2(Sensor* ptr, Headphones2AmplifierParam ampParam, OpStatus* outStatus);

	SDK_SHARED uint8_t addResistCallbackHeadphones2(Sensor* ptr, void(*callback)(Sensor* ptr, Headphones2ResistData* data, int32_t szData, void* userData), Headphones2ResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackHeadphones2(Headphones2ResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalDataCallbackHeadphones2(Sensor* ptr, void(*callback)(Sensor* ptr, Headphones2SignalData* data, int32_t szData, void* userData), Headphones2SignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalDataCallbackHeadphones2(Headphones2SignalDataListenerHandle handle);
	SDK_SHARED uint8_t readStimulatorParamCompactNeuro(Sensor* ptr, CompactNeuroStimulParam* stimulationParamsOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeStimulatorParamCompactNeuro(Sensor* ptr, CompactNeuroStimulParam stimulationParams, OpStatus* outStatus);
	SDK_SHARED uint8_t readSupportedChannelsCompactNeuro(Sensor* ptr, EEGChannelInfo* channelsOut, int32_t* szchannelsInOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readReferentsShortCompactNeuro(Sensor* ptr, uint8_t* refShortOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeReferentsShortCompactNeuro(Sensor* ptr, uint8_t refShort, OpStatus* outStatus);
	SDK_SHARED uint8_t readReferentsGroundCompactNeuro(Sensor* ptr, uint8_t* refGndOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeReferentsGroundCompactNeuro(Sensor* ptr, uint8_t refGnd, OpStatus* outStatus);

	SDK_SHARED uint8_t addResistCallbackCompactNeuro(Sensor* ptr, void(*callback)(Sensor* ptr, CompactNeuroResistData* data, int32_t szData, void* userData), CompactNeuroResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackCompactNeuro(CompactNeuroResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalDataCallbackCompactNeuro(Sensor* ptr, void(*callback)(Sensor* ptr, CompactNeuroSignalData* data, int32_t szData, void* userData), CompactNeuroSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalDataCallbackCompactNeuro(CompactNeuroSignalDataListenerHandle handle);
	SDK_SHARED uint8_t pingNeuroSmart(Sensor* ptr, uint8_t marker, OpStatus* outStatus);
	SDK_SHARED uint8_t readAmpMode(Sensor* ptr, SensorAmpMode* modeOut, OpStatus* outStatus);

	SDK_SHARED uint8_t addAmpModeCallback(Sensor* ptr, void(*callback)(Sensor* ptr, SensorAmpMode mode, void* userData), AmpModeListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeAmpModeCallback(AmpModeListenerHandle handle);
	SDK_SHARED uint8_t addResistCallbackHeadband(Sensor* ptr, void(*callback)(Sensor* ptr, HeadbandResistData data, void* userData), HeadbandResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackHeadband(HeadbandResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalDataCallbackHeadband(Sensor* ptr, void(*callback)(Sensor* ptr, HeadbandSignalData* data, int32_t szData, void* userData), HeadbandSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalDataCallbackHeadband(HeadbandSignalDataListenerHandle handle);

	SDK_SHARED uint8_t readSurveyIdNeuroEEG(Sensor* ptr, uint32_t* surveyIdOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeSurveyIdNeuroEEG(Sensor* ptr, uint32_t surveyId, OpStatus* outStatus);
	SDK_SHARED uint8_t readSupportedChannelsNeuroEEG(Sensor* ptr, EEGChannelInfo* channelsOut, int32_t* szchannelsInOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readAmplifierParamNeuroEEG(Sensor* ptr, NeuroEEGAmplifierParam* ampParamOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeAmplifierParamNeuroEEG(Sensor* ptr, NeuroEEGAmplifierParam ampParam, OpStatus* outStatus);
	SDK_SHARED uint8_t readFilesystemStatusNeuroEEG(Sensor* ptr, NeuroEEGFSStatus* filesystemStatusOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readFileSystemDiskInfoNeuroEEG(Sensor* ptr, SensorDiskInfo* diskInfoOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readFileInfoNeuroEEG(Sensor* ptr, const char* fileName, SensorFileInfo* fileInfoOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readFileInfoAllNeuroEEG(Sensor* ptr, SensorFileInfo* filesInfoOut, uint32_t* szFilesInfoOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeFileNeuroEEG(Sensor* ptr, const char* fileName, uint8_t* data, uint32_t szData, uint32_t offsetStart, OpStatus* outStatus);
	SDK_SHARED uint8_t readFileNeuroEEG(Sensor* ptr, const char* fileName, uint8_t* data, uint32_t* szDataInOut, uint32_t offsetStart, OpStatus* outStatus);
	SDK_SHARED uint8_t deleteFileNeuroEEG(Sensor* ptr, const char* fileName, OpStatus* outStatus);
	SDK_SHARED uint8_t deleteAllFilesNeuroEEG(Sensor* ptr, const char* fileExt, OpStatus* outStatus);
	SDK_SHARED uint8_t readFileCRC32NeuroEEG(Sensor* ptr, const char* fileName, uint32_t totalSize, uint32_t offsetStart, uint32_t* crc32Out, OpStatus* outStatus);
	SDK_SHARED uint8_t fileStreamAutosaveNeuroEEG(Sensor* ptr, const char* fileName, OpStatus* outStatus);
	SDK_SHARED uint8_t fileStreamReadNeuroEEG(Sensor* ptr, const char* fileName, uint32_t totalSize, uint32_t offsetStart, OpStatus* outStatus);
	SDK_SHARED void calcCRC32(uint8_t* data, uint32_t szData, uint32_t* crc32Out);

	SDK_SHARED uint8_t addSignalCallbackNeuroEEG(Sensor* ptr, void(*callback)(Sensor* ptr, SignalChannelsData* data, int32_t szData, void* userData), NeuroEEGSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalCallbackNeuroEEG(NeuroEEGSignalDataListenerHandle handle);
	SDK_SHARED uint8_t addResistCallbackNeuroEEG(Sensor* ptr, void(*callback)(Sensor* ptr, ResistChannelsData* data, int32_t szData, void* userData), NeuroEEGResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackNeuroEEG(NeuroEEGResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalResistCallbackNeuroEEG(Sensor* ptr, void(*callback)(Sensor* ptr, SignalChannelsData* signalData, int32_t szSignalData, ResistChannelsData* resistData, int32_t szResistData, void* userData), NeuroEEGSignalResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalResistCallbackNeuroEEG(NeuroEEGSignalResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalRawCallbackNeuroEEG(Sensor* ptr, void(*callback)(Sensor* ptr, uint8_t* data, int32_t szData, void* userData), NeuroEEGSignalRawDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalRawCallbackNeuroEEG(NeuroEEGSignalRawDataListenerHandle handle);
	SDK_SHARED uint8_t addFileStreamReadCallbackNeuroEEG(Sensor* ptr, void(*callback)(Sensor* ptr, SensorFileData* data, int32_t szData, void* userData), NeuroEEGFileStreamDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeFileStreamReadCallbackNeuroEEG(NeuroEEGFileStreamDataListenerHandle handle);

	SDK_SHARED uint8_t createSignalProcessParamNeuroEEG(NeuroEEGAmplifierParam ampParam, NeuroEEGSignalProcessParam* paramOut, OpStatus* outStatus);
	SDK_SHARED void removeSignalProcessParamNeuroEEG(NeuroEEGSignalProcessParam param);
	// signalOut.Samples and resistOut.Values - Required created manual! Actual size signalOut.SzSamples and resistOut.SzValues required set! Recommended channel size - NEURO_EEG_MAX_CH_COUNT. signalOut.SzSamples and resistOut.SzValues after invoke automatically updated
	SDK_SHARED uint8_t parseRawSignalNeuroEEG(uint8_t* data, uint32_t* szDataInOut, NeuroEEGSignalProcessParam processParam, SignalChannelsData* signalOut, uint32_t* szSignalInOut, ResistChannelsData* resistOut, uint32_t* szResistInOut, OpStatus* outStatus);

	SDK_SHARED uint8_t readSurveyIdNeuroBAM(Sensor* ptr, uint32_t* surveyIdOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeSurveyIdNeuroBAM(Sensor* ptr, uint32_t surveyId, OpStatus* outStatus);
	SDK_SHARED uint8_t readSupportedChannelsNeuroBAM(Sensor* ptr, EEGChannelInfo* channelsOut, int32_t* szchannelsInOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readAmplifierParamNeuroBAM(Sensor* ptr, NeuroBAMAmplifierParam* ampParamOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeAmplifierParamNeuroBAM(Sensor* ptr, NeuroBAMAmplifierParam ampParam, OpStatus* outStatus);

	SDK_SHARED uint8_t addSignalCallbackNeuroBAM(Sensor* ptr, void(*callback)(Sensor* ptr, SignalChannelsData* data, int32_t szData, void* userData), NeuroBAMSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalCallbackNeuroBAM(NeuroBAMSignalDataListenerHandle handle);
	SDK_SHARED uint8_t addResistCallbackNeuroBAM(Sensor* ptr, void(*callback)(Sensor* ptr, NeuroBAMResistChannelsData* data, int32_t szData, void* userData), NeuroBAMResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackNeuroBAM(NeuroBAMResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalResistCallbackNeuroBAM(Sensor* ptr, void(*callback)(Sensor* ptr, SignalChannelsData* signalData, int32_t szSignalData, NeuroBAMResistChannelsData* resistData, int32_t szResistData, void* userData), NeuroBAMSignalResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalResistCallbackNeuroBAM(NeuroBAMSignalResistDataListenerHandle handle);


	SDK_SHARED uint8_t addBatteryCallback(Sensor* ptr, void(*callback)(Sensor* ptr, int32_t battPower, void* userData), BattPowerListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeBatteryCallback(BattPowerListenerHandle handle);
	SDK_SHARED uint8_t addConnectionStateCallback(Sensor* ptr, void(*callback)(Sensor* ptr, SensorState state, void* userData), SensorStateListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeConnectionStateCallback(SensorStateListenerHandle handle);

	SDK_SHARED uint8_t addResistCallbackBrainBit(Sensor* ptr, void(*callback)(Sensor* ptr, BrainBitResistData data, void* userData), BrainBitResistDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeResistCallbackBrainBit(BrainBitResistDataListenerHandle handle);
	SDK_SHARED uint8_t addSignalDataCallbackBrainBit(Sensor* ptr, void(*callback)(Sensor* ptr, BrainBitSignalData* data, int32_t szData, void* userData), BrainBitSignalDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeSignalDataCallbackBrainBit(BrainBitSignalDataListenerHandle handle);
	SDK_SHARED uint8_t readSamplingFrequencyMEMSSensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readAccelerometerSensSensor(Sensor* ptr, SensorAccelerometerSensitivity* accSensOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeAccelerometerSensSensor(Sensor* ptr, SensorAccelerometerSensitivity accSens, OpStatus* outStatus);
	SDK_SHARED uint8_t readGyroscopeSensSensor(Sensor* ptr, SensorGyroscopeSensitivity* gyroSensOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeGyroscopeSensSensor(Sensor* ptr, SensorGyroscopeSensitivity gyroSens, OpStatus* outStatus);

	SDK_SHARED uint8_t addMEMSDataCallback(Sensor* ptr, void(*callback)(Sensor* ptr, MEMSData* data, int32_t szData, void* userData), MEMSDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeMEMSDataCallback(MEMSDataListenerHandle handle);
	SDK_SHARED uint8_t readSamplingFrequencyFPGSensor(Sensor* ptr, SensorSamplingFrequency* samplingFrequencyOut, OpStatus* outStatus);
	SDK_SHARED uint8_t readIrAmplitudeFPGSensor(Sensor* ptr, IrAmplitude* amplitudeOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeIrAmplitudeFPGSensor(Sensor* ptr, IrAmplitude amplitude, OpStatus* outStatus);
	SDK_SHARED uint8_t readRedAmplitudeFPGSensor(Sensor* ptr, RedAmplitude* amplitudeOut, OpStatus* outStatus);
	SDK_SHARED uint8_t writeRedAmplitudeFPGSensor(Sensor* ptr, RedAmplitude amplitude, OpStatus* outStatus);
	SDK_SHARED uint8_t addFPGDataCallback(Sensor* ptr, void(*callback)(Sensor* ptr, FPGData* data, int32_t szData, void* userData), FPGDataListenerHandle* handleOut, void* userData, OpStatus* outStatus);
	SDK_SHARED void removeFPGDataCallback(FPGDataListenerHandle handle);


#ifdef __cplusplus
}
#endif

#endif // SDK_API_H
