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
 *  5) Ctrl+C or SIGTERM at any time => gracefully abort and disconnect.
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
static volatile bool g_stopReading = false;  // set to true on SIGINT/SIGTERM

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

/* Called on SIGINT or SIGTERM to end the loop. */
static void sigintHandler(int signum) {
    (void)signum; // unused
    fprintf(stderr, "\nSignal caught. Requesting to stop.\n");
    g_stopReading = true;
}

/* Return the current time in milliseconds. */
static uint64_t getCurrentTimeMs(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
}

/* Non-blocking check for user pressing ENTER. */
static bool checkIfEnterPressed(void) {
    struct timeval tv = {0, 0};
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    int ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    if (ret > 0) {
        char c = getchar();
        return (c == '\n');
    }
    return false;
}

/* Convert the CAEN tag ID into hex string. */
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
    /* 1) set up SIGINT and SIGTERM so we can gracefully quit */
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);

    /* 2) Prepare the CAENRFIDReader structure with host function pointers */
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

    /* Connect at 921600 baud. */
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader, CAENRFID_USB, &port_params);
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "Error connecting: %d\n", ec);
        return -1;
    }
    printf("Connected.\n");

    /* Set unlimited read cycles => 0 */
    ec = CAENRFID_SetSourceConfiguration(&reader, "Source_0", CONFIG_READCYCLE, 0);
    if (ec != CAENRFID_StatusOK) {
        fprintf(stderr, "Warning: can't set infinite cycles: %d\n", ec);
    }

    /* (optional) Add read point "Ant0" to "Source_0" */
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

    /* Start continuous + framed inventory with RSSI enabled. */
    uint16_t flags = RSSI | CONTINUOS | FRAMED;
    ec = CAENRFID_InventoryTag(
        &reader,
        "Source_0",
        0, 0, 0, NULL, 0,
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

    /* Make stdin non-blocking so we can detect ENTER without pausing. */
    int oldFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags | O_NONBLOCK);

    /* 3) Main loop: poll for tags and capture a 3s window on ENTER. */
    bool capturing = false;
    uint64_t captureStart = 0;
    TagReading snapData[MAX_SNAP_TAGS];
    int snapCount = 0;

    while (!g_stopReading) {
        bool has_tag = false;
        bool has_result_code = false;
        CAENRFIDTag tag;
        memset(&tag, 0, sizeof(tag));

        ec = CAENRFID_GetFramedTag(&reader, &has_tag, &tag, &has_result_code);
        if (has_result_code) {
            printf("Inventory ended, code: %d\n", ec);
            break;
        }
        if (ec != CAENRFID_StatusOK && !has_tag) {
            /* Minor read error; ignore */
        }

        if (has_tag && capturing) {
            if (snapCount < MAX_SNAP_TAGS) {
                char epcHex[2*MAX_ID_LENGTH + 1];
                convertTagIDtoHex(&tag, epcHex);
                strcpy(snapData[snapCount].epc, epcHex);
                snapData[snapCount].rssi = tag.RSSI;
                snapCount++;
            }
        }

        if (capturing) {
            uint64_t now = getCurrentTimeMs();
            if (now - captureStart >= CAPTURE_WINDOW_MS) {
                if (snapCount == 0) {
                    printf("No tags captured in these 3 seconds.\n");
                } else {
                    int bestIndex = 0;
                    int16_t bestRSSI = snapData[0].rssi;
                    for (int i = 1; i < snapCount; i++) {
                        if (snapData[i].rssi > bestRSSI) {
                            bestIndex = i;
                            bestRSSI = snapData[i].rssi;
                        }
                    }
                    printf("BEST TAG from this 3s window:\n");
                    printf("   EPC=%s, RSSI=%d\n", snapData[bestIndex].epc, bestRSSI);
                }
                capturing = false;
            }
        } else {
            if (checkIfEnterPressed()) {
                printf("Starting a 3s capture window...\n");
                capturing = true;
                captureStart = getCurrentTimeMs();
                snapCount = 0;
            }
        }

        usleep(10000);  // 10ms delay
    }

    /* 4) Shutdown procedure:
       - Abort the inventory.
       - Wait for a full second.
       - Clear any pending data.
       - Then disconnect. */
    printf("Aborting inventory...\n");
    ec = CAENRFID_InventoryAbort(&reader);
    if (ec == CAENRFID_StatusOK) {
        sleep(1);  // Wait 1 second for abort to complete
        reader.clear_rx_data(reader._port_handle);
    } else {
        fprintf(stderr, "Error aborting: %d\n", ec);
    }

    ec = CAENRFID_Disconnect(&reader);
    if (ec == CAENRFID_StatusOK) {
        printf("Disconnected.\n");
    } else {
        fprintf(stderr, "Error disconnecting: %d\n", ec);
    }

    return 0;
}
