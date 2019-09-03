/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** test advertising on LED coded PHY  
 *  scan on LE CODED PHY
 *  scan on 1M phy in parallel
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "gap/Gap.h"
#include "gap/AdvertisingDataParser.h"
#include "pretty_printer.h"
#include "BatteryService.h"

using namespace ble;
using namespace mbed;

static const char DEVICE_NAME[] = "DualPHYScan";

class DualPHYScan : private mbed::NonCopyable<DualPHYScan>, public ble::Gap::EventHandler
{
public:

    static const uint16_t MAX_ADVERTISING_PAYLOAD_SIZE = 59;

    DualPHYScan(BLE& ble, events::EventQueue& event_queue) :
        _ble(ble),
        _gap(ble.gap()),
        _event_queue(event_queue),
        _led1(LED1, 0),
        _battery_uuid(GattService::UUID_BATTERY_SERVICE),
        _battery_level(100),
        _battery_service(ble, _battery_level),
        _adv_data_builder(_adv_buffer),
        _adv_handle(ble::INVALID_ADVERTISING_HANDLE)
    {
    }

    ~DualPHYScan()
    {
        if (_ble.hasInitialized()) {
            _ble.shutdown();
        }
    }

    void run();

private:
    /** This is called when BLE interface is initialised and starts the first mode */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *event);

    void startScanForever(void);

    void advertise(void);

    void update_payload(void);

    /** Set up and start scanning 1M phy*/
    void scan_1M_phy(void);

    /** Set up and start scanning coded phy*/
    void scan_coded_phy(void);

    void update_sensor_value(void);

    /** Blink LED to show we're running */
    void blink(void);

private: //following are virtual functions

    /* Gap::EventHandler */

    /** Look at scan payload and parse the packet **/
    virtual void onAdvertisingReport(
    const ble::AdvertisingReportEvent &event
    ) {
        
        ble::AdvertisingDataParser adv_parser(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_parser.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_parser.next();

            if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME &&
                field.value.size() == strlen(DEVICE_NAME) &&
                (memcmp(field.value.data(), DEVICE_NAME, field.value.size()) == 0)) {
                    printf("found peer device on %s:%s \r\n", 
                            phy_to_string(event.getPrimaryPhy()), phy_to_string(event.getSecondaryPhy()));
            }
            else if (field.type == ble::adv_data_type_t::MANUFACTURER_SPECIFIC_DATA &&
                field.value.size() > 2)
            {
                    printf("found device with manuf data on %s:%s \r\n",
                            phy_to_string(event.getPrimaryPhy()), phy_to_string(event.getSecondaryPhy()));
            }
        }
    }

    virtual void onAdvertisingEnd(
        const ble::AdvertisingEndEvent &event
    ) {
        if (event.isConnected()) {
            printf("Stopped advertising due to connection\r\n");
        }
        else
        {
            printf("Advertising ended \r\n");
        }
    }
    virtual void onScanTimeout(
        const ble::ScanTimeoutEvent&
    ) {
    }

    /** This is called by Gap to notify the application we connected */
    virtual void onConnectionComplete(
        const ble::ConnectionCompleteEvent &event
    ) {
    }

    /** This is called by Gap to notify the application we disconnected */
    virtual void onDisconnectionComplete(
        const ble::DisconnectionCompleteEvent &event
    ) {

        /* node disconnected as slave or master  */
    }

private:
    BLE                &_ble;
    ble::Gap           &_gap;
    events::EventQueue &_event_queue;

    DigitalOut _led1;

    UUID            _battery_uuid;
    uint8_t         _battery_level;
    BatteryService  _battery_service;
    uint8_t _adv_buffer[MAX_ADVERTISING_PAYLOAD_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;

    ble::advertising_handle_t _adv_handle;
};

/** Start BLE interface initialisation */
void DualPHYScan::run(void)
{
    if (_ble.hasInitialized()) {
        printf("Ble instance already initialised.\r\n");
        return;
    }

    /* handle gap events */
    _gap.setEventHandler(this);

    ble_error_t error = _ble.init(this, &DualPHYScan::on_init_complete);
    if (error) {
        print_error(error, "Error returned by BLE::init\r\n");
        return;
    }

    /* to show we're running we'll blink every 500ms */
    _event_queue.call_every(1000, this, &DualPHYScan::blink);

    /* this will not return until shutdown */
    _event_queue.dispatch_forever();
}

/** This is called when BLE interface is initialised and starts the first mode */
void DualPHYScan::on_init_complete(BLE::InitializationCompleteCallbackContext *event)
{
    if (event->error) {
        print_error(event->error, "Error during the initialisation\r\n");
        return;
    }

    if (!_gap.isFeatureSupported(ble::controller_supported_features_t::LE_EXTENDED_ADVERTISING) ||
            !_gap.isFeatureSupported(ble::controller_supported_features_t::LE_PERIODIC_ADVERTISING)) {
        printf("Periodic advertising not supported, cannot run example.\r\n");
        return;
    }

    print_mac_address();

    /* all calls are serialised on the user thread through the event queue */
    _event_queue.call(this, &DualPHYScan::advertise);

    _event_queue.call_in(200, this, &DualPHYScan::scan_1M_phy);

    _event_queue.call_in(10, this, &DualPHYScan::scan_coded_phy);
}

/** Set up and start advertising */
void DualPHYScan::advertise(void)
{
    ble_error_t error;
    bool adv_params_UseLegacyPDU;

    ble::AdvertisingParameters adv_params
        (
         ble::advertising_type_t::NON_CONNECTABLE_UNDIRECTED,
         ble::adv_interval_t(0x0320), //500ms
         ble::adv_interval_t(0x0320), //500ms
         false);

    adv_params.setOwnAddressType(ble::own_address_type_t::RANDOM);
    adv_params.setPhy(ble::phy_t::LE_CODED, ble::phy_t::LE_CODED);

    /* create the advertising set with its parameter if we haven't yet */
    if (_adv_handle == ble::INVALID_ADVERTISING_HANDLE) {
        error = _gap.createAdvertisingSet(
                &_adv_handle,
                adv_params
                );

        if (error) {
            print_error(error, "Gap::createAdvertisingSet() failed\r\n");
            return;
        }
    }

    _adv_data_builder.clear();

    _adv_data_builder.setFlags();

    _adv_data_builder.setName(DEVICE_NAME);

    error = _gap.setAdvertisingParameters(_adv_handle, adv_params);
    if (error) {
        print_error(error, "setAdvertisingParameters() failed \r\n");
        return;
    }

    /* Set payload for the set */
    error = _gap.setAdvertisingPayload(
            _adv_handle,
            _adv_data_builder.getAdvertisingData()
            );

    if (error) {
        print_error(error, "Gap::setAdvertisingPayload() failed\r\n");
        return;
    }

    error = _ble.gap().startAdvertising(
            _adv_handle,
            ble::adv_duration_t::forever()
            );

    if (error) {
        print_error(error, "Gap::startAdvertising() failed\r\n");
        return;
    }

    printf("Advertising started on CODED PHY forever\r\n");
}

void DualPHYScan::update_payload(void)
{
    /* advertising payload will have the battery level which we will update */
    ble_error_t error = _adv_data_builder.setServiceData(
            _battery_uuid,
            mbed::make_Span(&_battery_level, 1)
            );

    if (error) {
        print_error(error, "AdvertisingDataBuilder::setServiceData() failed\r\n");
        return;
    }

    /* the data in the local host buffer has been updated but now
     * we have to update the data in the controller */
    error = _gap.setPeriodicAdvertisingPayload(
            _adv_handle,
            _adv_data_builder.getAdvertisingData()
            );

    if (error) {
        print_error(error, "Gap::setPeriodicAdvertisingPayload() failed\r\n");
        return;
    }
}

/** Set up and start scanning 1M phy*/
void DualPHYScan::scan_1M_phy(void)
{
    ble::ScanParameters scan_params(
            ble::phy_t::LE_1M,
            ble::scan_interval_t(480),// scan_interval = scan_interval_t::min(),
            ble::scan_window_t(80),   // scan_window = scan_window_t::min(),
            false,
            ble::own_address_type_t::RANDOM,
            ble::scanning_filter_policy_t::NO_FILTER);

    ble_error_t error = _gap.setScanParameters(scan_params);

    if (error) {
        print_error(error, "Error caused by Gap::setScanParameters\r\n");
        return;
    }

    error = _gap.startScan(ble::scan_duration_t(0));

    if (error) {
        print_error(error, "Error caused by Gap::startScan\r\n");
        return;
    }
    printf("started scanning 1M PHY forever \r\n");
}

/** Set up and start scanning coded phy*/
void DualPHYScan::scan_coded_phy(void)
{
    /* two configurations should be ready duration initialisation */
    /* set1mPhyConfiguration */
    ble::ScanParameters scan_params(
            ble::phy_t::LE_CODED,
            ble::scan_interval_t(480),// scan_interval = scan_interval_t::min(),
            ble::scan_window_t(80),   // scan_window = scan_window_t::min(),
            false,
            ble::own_address_type_t::RANDOM,
            ble::scanning_filter_policy_t::NO_FILTER);

    ble_error_t error = _gap.setScanParameters(scan_params);

    if (error) {
        print_error(error, "Error caused by Gap::setScanParameters\r\n");
        return;
    }

    error = _gap.startScan(ble::scan_duration_t(0));

    if (error) {
        print_error(error, "Error caused by Gap::startScan\r\n");
        return;
    }
    printf("Scanning on LE coded PHY started\r\n");

    //printf("Scanning for periodic advertising started\r\n");
}

/** Blink LED to show we're running */
void DualPHYScan::blink(void)
{
    _led1 = !_led1;
}

events::EventQueue event_queue;

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();

    /* this will inform us off all events so we can schedule their handling
     * using our event queue */
    ble.onEventsToProcess(schedule_ble_events);

    /* look for other device and then settle on a role and sync periodic advertising */
    DualPHYScan DualPhyScan(ble, event_queue);

    DualPhyScan.run();

    return 0;
}
