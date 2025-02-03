/******************************************************************************
 * read_tags_best_window.c
 *
 * Demonstrates a continuous, framed inventory with CAEN "Light" library,
 * but only "captures" data for a specific 3-second window each time the user
 * presses Enter. Afterwards, it picks the single best tag (highest RSSI).
 *
 * Steps:
 *  1) Connect at 921600 baud, set read cycles = 0 (unlimited).
 *  2) Add "Ant0" to "Source_0" (if needed), set protocol to EPC_C1G2.
 *  3) Start continuous + framed inventory with RSSI included.
 *  4) In the main loop:
 *       - We *do* call GetFramedTag(...) in the background, but ignore results
 *         until user presses Enter.
 *       - On Enter, we do a separate 3s "capture window" where we store
 *         all tags' RSSI and IDs.
 *       - After 3 seconds, pick the best (highest RSSI) tag from that window,
 *         and print it.
 *       - Then go back to waiting for next Enter.
 *  5) Ctrl+C at any time => gracefully abort and disconnect.
 *
 * Expand the logic in pickBestTag() if you want to also measure how "constant"
 * the RSSI is (less variance => more stable, etc.).
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

#include "CAENRFIDLib_Light.h"
#include "host.h"

/* -----------
   Global / Constants
   -----------
*/
static volatile bool g_stopReading = false;  // set to true on Ctrl+C

#define CAPTURE_WINDOW_MS 1000  // 1 seconds
#define MAX_SNAP_TAGS     200   // how many tag reads to store during the capture window

/* A simple structure to store each reading in our 3s window. */
typedef struct {
    char    epc[2 * MAX_ID_LENGTH + 1]; // Tag ID as hex
    int16_t rssi;
} TagReading;

/* -----------
   Helpers
   -----------
*/

/* Called on Ctrl+C (SIGINT) to end the loop. */
static void sigintHandler(int signum) {
    (void)signum; // unused
    fprintf(stderr, "\nSIGINT caught. Requesting to stop.\n");
    g_stopReading = true;
}

/* Return the current time in milliseconds. 
   This is identical or similar to get_ms_timestamp() in host.c, 
   but we'll replicate it here. */
static uint64_t getCurrentTimeMs(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
}

/* Non-blocking check for user pressing ENTER. */
static bool checkIfEnterPressed(void) {
    /* Use select() or just see if data is available on stdin. */
    struct timeval tv = {0, 0};
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    int ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    if (ret > 0) {
        /* There's something to read. Check if it's an actual ENTER. */
        char c = getchar();  // read one char
        return (c == '\n');  // we only care if user typed ENTER
    }
    return false;
}

/* Convert the CAEN tag ID into hex string. outHex must have enough space. */
static void convertTagIDtoHex(const CAENRFIDTag* tag, char* outHex) {
    for (int i = 0; i < tag->Length; i++) {
        sprintf(&outHex[2*i], "%02X", tag->ID[i]);
    }
    outHex[2 * tag->Length] = '\0';
}

/* -----------
   main logic
   -----------
*/
int main(void) {
    /* 1) set up SIGINT so we can gracefully quit on Ctrl+C */
    signal(SIGINT, sigintHandler);

    /* 2) Prepare the CAENRFIDReader structure with your host function pointers */
    CAENRFIDReader reader = {
        .connect       = _connect,
        .disconnect    = _disconnect,
        .tx            = _tx,
        .rx            = _rx,
        .clear_rx_data = _clear_rx_data,
        .enable_irqs   = _enable_irqs,
        .disable_irqs  = _disable_irqs
    };

    /* Connect parameters for your device. */
    RS232_params port_params = {
        .com         = "/dev/ttyACM1", // adjust as needed
        .baudrate    = 921600,
        .dataBits    = 8,
        .stopBits    = 1,
        .parity      = 0,
        .flowControl = 0
    };

    /* Connect at 921600 baud, or your device's requirement. */
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader, CAENRFID_USB, &port_params);
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "Error connecting: %d\n", ec);
        return -1;
    }
    printf("Connected.\n");

    /* Set unlimited read cycles => 0, so continuous inventory won't end by itself */
    ec = CAENRFID_SetSourceConfiguration(&reader, "Source_0", CONFIG_READCYCLE, 0);
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "Warning: can't set infinite cycles: %d\n", ec);
    }

    /* (optional) Add read point "Ant0" to "Source_0" if needed */
    ec = CAENRFID_AddReadPoint(&reader, "Source_0", "Ant0");
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "AddReadPoint returned %d (may be okay)\n", ec);
    }

    /* Set the protocol to EPC_C1G2 */
    ec = CAENRFID_SetProtocol(&reader, CAENRFID_EPC_C1G2);
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "Error setting protocol: %d\n", ec);
        CAENRFID_Disconnect(&reader);
        return -1;
    }

    /* Start continuous + framed inventory,
       also enable RSSI to get Tag->RSSI */
    uint16_t flags = RSSI | CONTINUOS | FRAMED; // = 0x01 + 0x04 + 0x02 = 0x07
    ec = CAENRFID_InventoryTag(
        &reader,
        "Source_0", 
        0,0,0, NULL,0, 
        flags,
        NULL, 
        NULL
    );
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "Error starting continuous inventory: %d\n", ec);
        CAENRFID_Disconnect(&reader);
        return -1;
    }
    printf("Continuous inventory started (RSSI enabled).\n");
    printf("Press ENTER to do a 3-second 'best tag' measurement, or Ctrl+C to exit.\n");

    /* Make stdin non-blocking so we can detect Enter without pausing. */
    int oldFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags | O_NONBLOCK);

    /* 3) Main loop: 
          We keep calling GetFramedTag(...) so the reader sees us as "active."
          We ignore those results unless we are in a capture window.
          If user presses ENTER, we do a 3s capturing. */

    bool capturing = false;       // Are we in the 3-second capture window?
    uint64_t captureStart = 0;    // when capture began
    TagReading snapData[MAX_SNAP_TAGS];
    int snapCount = 0;

    while (!g_stopReading) {
        /* Check for new framed tag. Always call it to keep reading. */
        bool has_tag = false;
        bool has_result_code = false;
        CAENRFIDTag tag;
        memset(&tag, 0, sizeof(tag));

        ec = CAENRFID_GetFramedTag(&reader, &has_tag, &tag, &has_result_code);
        if (has_result_code) {
            // The inventory ended on its own
            printf("Inventory ended, code: %d\n", ec);
            break;
        }
        if (ec != CAENRFID_StatusOK && !has_tag) {
            // Possibly a minor read error
            // Not necessarily fatal
            // fprintf(stderr, "GetFramedTag error: %d\n", ec);
        }

        if (has_tag && capturing) {
            /* We are in the 3-second capture window. Store this read. */
            if (snapCount < MAX_SNAP_TAGS) {
                /* Convert to hex, store with RSSI. */
                char epcHex[2*MAX_ID_LENGTH + 1];
                convertTagIDtoHex(&tag, epcHex);

                strcpy(snapData[snapCount].epc, epcHex);
                snapData[snapCount].rssi = tag.RSSI;
                snapCount++;
            }
        }

        /* If we are capturing, check if 3s have passed. */
        if (capturing) {
            uint64_t now = getCurrentTimeMs();
            if (now - captureStart >= CAPTURE_WINDOW_MS) {
                /* Time's up: pick the best tag. */
                if (snapCount == 0) {
                    printf("No tags captured in these 3 seconds.\n");
                } else {
                    /* find highest RSSI in snapData */
                    int bestIndex = 0;
                    int16_t bestRSSI = snapData[0].rssi;
                    for (int i=1; i < snapCount; i++) {
                        if (snapData[i].rssi > bestRSSI) {
                            bestIndex = i;
                            bestRSSI = snapData[i].rssi;
                        }
                    }
                    printf("BEST TAG from this 3s window:\n");
                    printf("   EPC=%s, RSSI=%d\n", snapData[bestIndex].epc, bestRSSI);
                }
                /* end capturing mode */
                capturing = false;
            }
        } 
        else {
            /* Not currently capturing -> check if user just pressed ENTER to start. */
            if (checkIfEnterPressed()) {
                printf("Starting a 3s capture window...\n");
                capturing = true;
                captureStart = getCurrentTimeMs();
                snapCount = 0;
            }
        }

        usleep(10000);  // 10ms to reduce CPU usage
    }

    /* 4) If we exit the loop, we do an abort and then disconnect. */
    printf("Aborting inventory...\n");
    ec = CAENRFID_InventoryAbort(&reader);
    if (ec == CAENRFID_StatusOK) {
        // We must read leftover frames until we see result_code
        while (true) {
            bool has_tag = false, has_rc = false;
            CAENRFIDTag tag;
            memset(&tag, 0, sizeof(tag));

            ec = CAENRFID_GetFramedTag(&reader, &has_tag, &tag, &has_rc);
            if (has_rc) {
                printf("Final code after abort: %d\n", ec);
                break;
            }
            if (ec != CAENRFID_StatusOK && !has_tag) {
                // error or no data, presumably done
                break;
            }
            // if has_tag => leftover frames
        }
    } else {
        fprintf(stderr, "Error aborting: %d\n", ec);
    }

    /* Finally, disconnect. */
    ec = CAENRFID_Disconnect(&reader);
    if (ec == CAENRFID_StatusOK) {
        printf("Disconnected.\n");
    } else {
        fprintf(stderr, "Error disconnecting: %d\n", ec);
    }

    return 0;
}
