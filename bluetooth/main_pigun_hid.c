/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

//#define BTSTACK_FILE__ "hid_joystick_demo.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>

#include "btstack.h"
#include "pigun_bt.h" // this is mine!



// from USB HID Specification 1.1, Appendix B.1
// this is custom made joystick with 8 buttons, and two 16-bit axis
const uint8_t hid_descriptor_joystick_mode[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)
    0x16, 0x01, 0x80,  //   Logical Minimum 0x8001 (-32767)  
    0x26, 0xFF, 0x7F,  //   Logical Maximum 0x7FFF (32767)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x75, 0x10,        //     Report Size (16) --- 22 bytes
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection   --- 27 bytes
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x08,        //   Usage Maximum (0x08)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1) --- 37 bytes
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection --- 44 bytes
};



// STATE

static uint8_t hid_service_buffer[250];
//static uint8_t device_id_sdp_service_buffer[100];
static const char hid_device_name[] = "HID PiGun";
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint16_t hid_cid;
static uint8_t hid_boot_device = 0;


static enum {
    APP_BOOTING,
    APP_NOT_CONNECTED,
    APP_CONNECTING,
    APP_CONNECTED
} app_state = APP_BOOTING;


int nServers = 0;
bd_addr_t servers[3];

void* pigun_autoconnect(void* nullargs);
pthread_t pigun_autoconnect_thr;
static btstack_timer_source_t heartbeat;
static void heartbeat_handler(btstack_timer_source_t* ts);



int compare_servers(bd_addr_t a, bd_addr_t b) {
    for (int i = 0; i < 6; i++) {
        if (a[i] != b[i]) return 0;
    }
    return 1;
}

// HID Report sending
static void send_report() {

    // this is the report to send
    // I do now know that the first byte is there for?!
    uint8_t hid_report[] = { 0xa1, 0, 0, 0, 0, 0 };

    hid_report[1] = (global_pigun_report.x) & 0xff;
    hid_report[2] = (global_pigun_report.x >> 8) & 0xff;
    hid_report[3] = (global_pigun_report.y) & 0xff;
    hid_report[4] = (global_pigun_report.y >> 8) & 0xff;
    hid_report[5] = global_pigun_report.buttons;

    //printf("sending x=%i (%i %i) y=%i (%i %i) \n", global_pigun_report.x, hid_report[1], hid_report[2], global_pigun_report.y, hid_report[3], hid_report[4]);
    hid_device_send_interrupt_message(hid_cid, &hid_report[0], 6); // 6 = sizeof(hid_report)
}


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t packet_size){
    UNUSED(channel);
    UNUSED(packet_size);
    uint8_t status;
    
    //printf("got packet type: %i \n", packet_type);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        app_state = APP_NOT_CONNECTED;
        break;

    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
        // ssp: inform about user confirmation request
        log_info("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", hci_event_user_confirmation_request_get_numeric_value(packet));
        log_info("SSP User Confirmation Auto accept\n");
        break;

    // *** HID META EVENTS ***
    case HCI_EVENT_HID_META:
        switch (hci_event_hid_meta_get_subevent_code(packet)) {

        case HID_SUBEVENT_CONNECTION_OPENED:
            status = hid_subevent_connection_opened_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) {
                // outgoing connection failed
                printf("Connection failed, status 0x%x\n", status);
                app_state = APP_NOT_CONNECTED;
                hid_cid = 0;

                // re-register timer
                btstack_run_loop_set_timer(&heartbeat, 1000);
                btstack_run_loop_add_timer(&heartbeat);
                return;
            }
            app_state = APP_CONNECTED;
            hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
            bd_addr_t host_addr;
            hid_subevent_connection_opened_get_bd_addr(packet, host_addr);

            // save the server address - if not already there
            // rewrite the past servers list, putting the current one on top
            // only write 3 of them
            int isThere = 0;
            for (int i = 0; i < nServers; i++) {
                if (compare_servers(host_addr, servers[i])) {
                    isThere = 1;
                    break;
                }
            }
            int ns = nServers;
            if (!isThere) ns++;
            if (ns > 3) ns = 3;
            int written = 1;
            bd_addr_t newlist[3];

            FILE* fout = fopen("servers.bin", "wb");
            fwrite(&ns, sizeof(int), 1, fout);
            fwrite(host_addr, sizeof(bd_addr_t), 1, fout); memcpy(newlist[0], host_addr, sizeof(bd_addr_t));

            for (int i = 0; i < nServers; i++) {
                if (!compare_servers(host_addr, servers[i])) {
                    fwrite(servers[i], sizeof(bd_addr_t), 1, fout); memcpy(newlist[written], servers[i], sizeof(bd_addr_t));
                    written++;
                    if (written == 3) break;
                }
            }
            fclose(fout);
            memcpy(servers[0], newlist[0], sizeof(bd_addr_t)*3);
            nServers = ns;

            printf("HID connected to %s, pigunning now...\n", bd_addr_to_str(host_addr));
            hid_device_request_can_send_now_event(hid_cid); // request a sendnow
            break;
        case HID_SUBEVENT_CONNECTION_CLOSED:
            printf("HID Disconnected\n");
            app_state = APP_NOT_CONNECTED;
            hid_cid = 0;
            break;
        case HID_SUBEVENT_CAN_SEND_NOW:
            // when the stack raises can_send_now event, we send a report... because we can!
            send_report(); // uses the global variable with gun data

            // and then we request another can_send_now because we are greedy! need to send moooar!
            hid_device_request_can_send_now_event(hid_cid);
            break;
        default:
            break;
        }
        break;


    default: //any other type of event
        break;
    }
}



/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code. 
 * To run a HID Device service you need to initialize the SDP, and to create and register HID Device record with it. 
 * At the end the Bluetooth stack is started.
 */

/* LISTING_START(MainConfiguration): Setup HID Device */

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){
    (void)argc;
    (void)argv;

    // allow to get found by inquiry
    gap_discoverable_control(1);
    // use Limited Discoverable Mode; Peripheral; Keyboard as CoD
    gap_set_class_of_device(0x2504);
    // set local name to be identified - zeroes will be replaced by actual BD ADDR
    gap_set_local_name("PiGun 1d"); // ("PiGun 00:00:00:00:00:00");
    // allow for role switch in general and sniff mode
    gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE );
    // allow for role switch on outgoing connections - this allow HID Host to become master when we re-connect to it
    gap_set_allow_role_switch(true);

    // L2CAP
    l2cap_init();

    // SDP Server
    sdp_init();
    memset(hid_service_buffer, 0, sizeof(hid_service_buffer));

    uint8_t hid_virtual_cable = 0;
    uint8_t hid_remote_wake = 0;
    uint8_t hid_reconnect_initiate = 1;
    uint8_t hid_normally_connectable = 1;

    hid_sdp_record_t hid_params = {
        // hid sevice subclass 0x2504 joystick, hid counntry code 33 US
        0x2504, 33, // should be the joystick code
        hid_virtual_cable, hid_remote_wake, 
        hid_reconnect_initiate, hid_normally_connectable,
        hid_boot_device, 
        0xFFFF, 0xFFFF, 3200,
        hid_descriptor_joystick_mode,
        sizeof(hid_descriptor_joystick_mode), 
        hid_device_name
    };
    
    hid_create_sdp_record(hid_service_buffer, 0x10001, &hid_params);

    printf("HID service record size: %u\n", de_get_len(hid_service_buffer));
    sdp_register_service(hid_service_buffer);

    // See https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers if you don't have a USB Vendor ID and need a Bluetooth Vendor ID
    // device info: BlueKitchen GmbH, product 1, version 1
   // device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10003, DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 2, 1);
    //printf("Device ID SDP service record size: %u\n", de_get_len((uint8_t*)device_id_sdp_service_buffer));
    //sdp_register_service(device_id_sdp_service_buffer);

    // HID Device
    hid_device_init(hid_boot_device, sizeof(hid_descriptor_joystick_mode), hid_descriptor_joystick_mode);
       
    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for HID events
    hid_device_register_packet_handler(&packet_handler);
    
    // turn on!
    hci_power_control(HCI_POWER_ON);

    // read the previous server addresses
    nServers = 0;
    FILE* fin = fopen("servers.bin", "rb");
    if (fin == NULL) {
        fin = fopen("servers.bin", "wb");
        fwrite(&nServers, sizeof(int), 1, fin);
        fclose(fin);
        fin = fopen("servers.bin", "rb");
    }
    fread(&nServers, sizeof(int), 1, fin);
    
    printf("HID previous hosts: %i\n", nServers);
    for (int i = 0; i < nServers; i++) {
        fread(servers[i], sizeof(bd_addr_t), 1, fin);
        printf("previous host[%i]: %s\n", i, bd_addr_to_str(servers[i]));
    }
    fclose(fin);
    

    //pthread_create(&pigun_autoconnect_thr, NULL, pigun_autoconnect, NULL);

    // set one-shot timer
    heartbeat.process = &heartbeat_handler;
    if (nServers != 0) {
        btstack_run_loop_set_timer(&heartbeat, 5000);
        btstack_run_loop_add_timer(&heartbeat);
    }

    return 0;
}
/* LISTING_END */
/* EXAMPLE_END */


static void heartbeat_handler(btstack_timer_source_t* ts) {
    UNUSED(ts);

    // increment counter
    static int snum = 0;

    if (app_state == APP_NOT_CONNECTED && nServers != 0) {

        // try connecting to a server
        printf("trying to connect to %s...\n", bd_addr_to_str(servers[snum]));
        hid_device_connect(servers[snum], &hid_cid);

        snum++; if (snum == nServers)snum = 0;
    }
}

