#include "myBlue.h"

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/services/UARTService.h"

#include "Buffer.h"
#include "CurrentTimeService.h"
#include "main.h"
#include "XMODEMReceiver.h"
#include <events/mbed_events.h>

#include "math.h"



static const char blue_device_name[] = "MyCalc";
static EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

uint16_t floatAUUID = 0xA000;
uint16_t floatBUUID = 0xA001;
uint16_t operatorUUID = 0xA002;
uint16_t resultUUID = 0xA003;
uint16_t serviceUUID = 0xA004;

static const uint16_t uuid16_list[]        = {floatAUUID, floatBUUID, operatorUUID, resultUUID};

// Set Up custom Characteristics
static float *A;
static float *B;
static uint8_t *operatorAB;
static float *result;

static float _result;
static uint8_t resultBytes[sizeof(float)];

 ReadWriteGattCharacteristic<float> readWriteA(floatAUUID, A);
 ReadWriteGattCharacteristic<float> readWriteB(floatBUUID, B);

 WriteOnlyGattCharacteristic<uint8_t> writeOperator(operatorUUID, operatorAB);

 ReadOnlyGattCharacteristic<float> readResult(resultUUID, result);

 
static void blue_connected(const Gap::ConnectionCallbackParams_t *params)
{
    usb.printf("[blue] connected\r\n");

}

static void blue_characteristic_written(const GattWriteCallbackParams *params)
 {

    //If A is written
    if (params->handle == readWriteA.getValueHandle())
     {  
        usb.printf("[blue] A value written\r\n");
    }

    //If B is written
    if(params->handle == readWriteB.getValueHandle())
    {
        usb.printf("[blue] B value written\r\n");
    }

    //If operator is written
    if(params-> handle == writeOperator.getValueHandle())
    {
        usb.printf("[blue] operator value written\r\n");
    }


    //operatorAB table
    //0 +
    //1 -
    //2 *
    //3 /
    //4 ^^
    //5 sqrt

    if (*operatorAB == 0)
    {
        _result = *A+*B;
    }
    
    if(*operatorAB == 1)
    {
        _result = *A-*B;
    }

    if(*operatorAB == 2)
    {
        _result = *A * *B;
    }

    if(*operatorAB == 3)
    {
        if(*B == 0.0f){
            BLE::Instance().shutdown();
        }
        else
        {
            _result = *A / *B;
        }
    }

    if(*operatorAB == 4)
    {
        _result = pow(*A, *B); 
    }

    if(*operatorAB == 5)
    {
        if(*B == 0.0f)
           BLE::Instance().shutdown();
        _result = pow(*A, 1.0f/ *B);
    }
         

    *(float*)(resultBytes) = _result;
    BLE::Instance().gattServer().write(readResult.getValueHandle(), resultBytes,sizeof(resultBytes));


}

static void blue_disconnected(const Gap::DisconnectionCallbackParams_t *params)
{
    usb.printf("[blue] disconnected\r\n");

    BLE::Instance().gap().startAdvertising();
}

static void blue_process_events(BLE::OnEventsToProcessCallbackContext* context)
{
    queue.call(Callback<void()>(&BLE::Instance(), &BLE::processEvents));
}


static void blue_init_completed(BLE::InitializationCompleteCallbackContext *params)
{
    BLE& ble = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, do sg meaningful */
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE)
        return;

    usb.printf("[blue] registering connection callbacks\r\n");
    ble.gap().onConnection(blue_connected);
    ble.gap().onDisconnection(blue_disconnected);

    /* Setup services */
    usb.printf("[blue] initializing calculator service\r\n");
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list)); 

   

    // Set up custom service
  GattCharacteristic *characteristics[] = {&readWriteA, &readWriteB, &writeOperator, &readResult};
 GattService   calcService(serviceUUID, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *));
    ble.onDataWritten(blue_characteristic_written);
    ble.addService(calcService);
   

    /* Setup advertising */
    usb.printf("[blue] configuring and starting advertising\r\n");

    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS,
                                           (const uint8_t *)uuid16_list, sizeof(uuid16_list));
    // NOTE: the name might not fit in the advertising payload anymore, so put
    //       it into the scan response
    // NOTE: don't try to split the payload in a different way, keep all 128 and
    //       16 bit service UUIDS in the advertising payload, BluefruitLE/OSX
    //       may not be able to see all of them (Android BLE stack handles that
    //       case correctly, too)
    ble.gap().accumulateScanResponse(GapAdvertisingData::COMPLETE_LOCAL_NAME,
                                     (const uint8_t *)blue_device_name, sizeof(blue_device_name));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* multiples of 0.625ms */
    ble.gap().startAdvertising();
}

void myblue_init(void)
{

    usb.printf("[blue] initializing ble\r\n");
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(blue_process_events);
    ble.init(blue_init_completed);

}