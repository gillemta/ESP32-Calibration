#include "dw3000.h"
#include <WiFi.h>
#include <chrono>
#include <map>
#include <vector>

// extern SPISettings _fastSPI;

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

// Delays
#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define RESP_RX_TIMEOUT_UUS 1000
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define POLL_RX_TO_RESP_TX_DLY_UUS 250

// Indexes
#define ALL_MSG_SN_IDX 2
#define RECEIVER_ID_IDX 18
#define SENDER_ID_IDX 19
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14

// Other
#define NUM_OF_ANCHORS 5
#define RESP_MSG_TS_LEN 4
#define ALL_MSG_COMMON_LEN 10


/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

int anchorIDs[] = {132, 28, 24, 10, 30};
int deviceID;

static uint8_t msg_poll[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Poll message array (used when initiating a distance measurement)
static uint8_t msg_resp[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'E', 'V', 'A', 'W', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Response message array (used when responding to a poll)
static uint8_t rx_buffer[22]; // Buffer for receiving messages
static uint8_t frame_seq_nb = 0;
static uint32_t status_reg = 0;
static uint64_t poll_ts;
static uint64_t resp_ts;
static double tof;      // Time of Flight
static double distance; // Calculated distance based on the Time of Flight
extern dwt_txconfig_t txconfig_options;

std::map<int, int> deviceMappings = {
    {132, 0},
    {28, 1},
    {24, 2},
    {184, 3},
    {196, 4},
};

std::map<int, std::vector<double>> finalDistances;
std::map<int, int> distanceCounts;
std::vector<std::vector<bool>> distanceStored(NUM_OF_ANCHORS, std::vector<bool>(NUM_OF_ANCHORS, false));

// ** Add values to dictionary **
// finalDistances[1].push_back(10);
// finalDistances[1].push_back(20);

// Function to retrieve device ID based on the last byte of MAC address
void getID() {
  byte mac[6];
  WiFi.macAddress(mac);
  int lastByteOfMac = mac[5]; // Use the last byte of the MAC address

  if (deviceMappings.find(lastByteOfMac) != deviceMappings.end()) {
    deviceID = deviceMappings[lastByteOfMac];
    Serial.printf("Device %d has ID %d\n", deviceID, lastByteOfMac);
  } else {
    Serial.printf("Device %d hasn't been configured\n", lastByteOfMac);
  }
}

void setup()
{
  UART_init();

  // Start serial communication for debugging or data output
  Serial.begin(115200);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }


  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);


  delay(1000);
  getID();
  // Initialize finalDistances map
  for (int i = 0; i < NUM_OF_ANCHORS; ++i) {
    finalDistances[i] = std::vector<double>(NUM_OF_ANCHORS - i - 1, 0.0);
    distanceCounts[i] = 0;
  }

  
  Serial.println("Setup over........");
}

void loop() {
  if (isMyTimeSlot()) {
    getDistances();
  } else {
    listenForPollsAndRespond();
  }
}

uint64_t timeSinceEpochMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool isMyTimeSlot() {
  uint64_t currentTime = timeSinceEpochMillisec();

  unsigned long timeSlotLength = 4000; // 4 seconds for each slot
  unsigned long cycleDuration = NUM_OF_ANCHORS * timeSlotLength;

  unsigned long timeSlotStart = deviceID * timeSlotLength;
  unsigned long nextTimeSlotStart = timeSlotStart + timeSlotLength;

  if (deviceID == NUM_OF_ANCHORS - 1) {
    nextTimeSlotStart = cycleDuration;
  }

  if (currentTime % cycleDuration >= timeSlotStart && currentTime % cycleDuration < nextTimeSlotStart) {
    return true;
  } else {
    return false;
  }
}

void getDistances() {
  for (int i = deviceID + 1; i < NUM_OF_ANCHORS; i++) {
    if(pollDevice(i)) {
      responseReceived();
    }
    delay(300);
  }
}

bool pollDevice(int targetDeviceID) {
  // Send a poll message to the specified device
  // Implement the actual UWB poll message sending logic
  msg_poll[RECEIVER_ID_IDX] = targetDeviceID;
  msg_poll[SENDER_ID_IDX] = deviceID;
  msg_poll[ALL_MSG_SN_IDX] = frame_seq_nb;
  
  // Serial.printf("Device %d --> %d\n", deviceID, targetDeviceID);
  // Serial.println();

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(msg_poll), msg_poll, 0);
  dwt_writetxfctrl(sizeof(msg_poll), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

  frame_seq_nb++;

  return (status_reg & SYS_STATUS_RXFCG_BIT_MASK) != 0;
}

void responseReceived() {
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

  if (frame_len <= sizeof(rx_buffer)) {
    dwt_readrxdata(rx_buffer, frame_len, 0);
  } else {
    return;
  }

  rx_buffer[ALL_MSG_SN_IDX] = 0;
  msg_resp[ALL_MSG_SN_IDX] = 0;

  if (memcmp(rx_buffer, msg_resp, ALL_MSG_COMMON_LEN) == 0) {
    int targetID = rx_buffer[SENDER_ID_IDX];
    printf("Successful communication between Devices %d --> %d\n", rx_buffer[RECEIVER_ID_IDX], rx_buffer[SENDER_ID_IDX]);
    calculateDistances(targetID);
    // calculateDistances(,, targetID);
  } else {
    // Serial.println("Response message and buffer don't match");
    // Serial.println("Received buffer:");
    // printBuffer(rx_buffer, sizeof(rx_buffer));
    // Serial.println("Expected response message:");
    // printBuffer(msg_resp, sizeof(msg_resp));
    // delay(1000);
  }
}

void listenForPollsAndRespond() {
  if (pollReceived()) {
    respondToPoll();
  }
}

bool pollReceived() {
  // Implement logic to check if a poll message is received
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)){
      dwt_readrxdata(rx_buffer, frame_len, 0);

      uint8_t targetDeviceID = rx_buffer[RECEIVER_ID_IDX];
      if (targetDeviceID == deviceID) {
        return true;
      }
    }
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }

  return false;
}

// // Implement logic to check if poll message is recieved
// bool pollReceived(){
//   while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
//       // Loop until frame is received, a timeout occurs, or an error is detected
//   };


//     // Checks status of message 
//     if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

//       // Successfully revieved frame so will now check contents
//       uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

//       // Checks length of response
//       if (frame_len == sizeof(msg_resp)) {

//           dwt_readrxdata(rx_buffer, frame_len, 0);
//           dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
//           uint8_t targetDeviceID = rx_buffer[RECEIVER_ID_IDX];
//           // Checks if device is the one being targetted, if so return true, else false
//           if (targetDeviceID == deviceID){
//               return true;
//           } else{
//               return false;
//           }

//       }
//     } else {

//         // No valid poll recieved
//         dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//     }
// }

void respondToPoll() {
  // Implement logic to respond to the received poll message
  rx_buffer[ALL_MSG_SN_IDX] = 0;
  msg_poll[ALL_MSG_SN_IDX] = 0;  
  
  uint32_t resp_time;
  int ret;

  poll_ts = get_rx_timestamp_u64();

  resp_time = (poll_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

  // Serial.printf("poll_ts - %d\n", poll_ts);
  // Serial.printf("resp_time - %d\n", resp_time);
  // delay(5000);
  dwt_setdelayedtrxtime(resp_time);


  resp_ts = (((uint64_t)(resp_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  msg_resp[RECEIVER_ID_IDX] = rx_buffer[SENDER_ID_IDX];
  msg_resp[SENDER_ID_IDX] = rx_buffer[RECEIVER_ID_IDX];

  resp_msg_set_ts(&msg_resp[RESP_MSG_POLL_RX_TS_IDX], poll_ts);
  resp_msg_set_ts(&msg_resp[RESP_MSG_RESP_TX_TS_IDX], resp_ts);
  
  msg_resp[ALL_MSG_SN_IDX] = frame_seq_nb;

  // Serial.println("Sending Response...");
  // printBuffer(msg_resp, sizeof(msg_resp));
  
  dwt_writetxdata(sizeof(msg_resp), msg_resp, 0);
  dwt_writetxfctrl(sizeof(msg_resp), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  if (ret == DWT_SUCCESS) {
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    frame_seq_nb++;
  }
}

// void calculateDistances(double pollTimeStamp, double responseTimeStamp, int targetDeviceID) {
//   double totalTimeOfFlight, distance;
//   totalTimeOfFlight = abs(pollTimeStamp - responseTimeStamp);

//   distance = totalTimeOfFlight / (128 * 499.2); // to turn time into clock ticks divide the TOF by 128 * 499.2MHz

//   logDistanceData(distance, targetDeviceID);
// }

void calculateDistances(int targetDeviceID) {
  uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
  int32_t rtd_init, rtd_resp;
  float clockOffsetRatio;

  poll_tx_ts = dwt_readtxtimestamplo32();
  resp_rx_ts = dwt_readrxtimestamplo32();

  clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

  resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
  resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

  rtd_init = resp_rx_ts - poll_tx_ts;
  rtd_resp = resp_tx_ts - poll_rx_ts;

  tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
  distance = tof * SPEED_OF_LIGHT;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println( "meters");
  // delay(3000);

  logDistanceData(distance, targetDeviceID);
}

void logDistanceData(double distance, int targetDeviceID) {
  int index = targetDeviceID - deviceID - 1;

  if (!distanceStored[deviceID][index]) {
    finalDistances[deviceID][index] = distance;

    distanceStored[deviceID][index] = true;

    distanceCounts[deviceID]++;

    if (distanceCounts[deviceID] == (NUM_OF_ANCHORS - deviceID -1)) {
      Serial.print("Distances for Device ");
      Serial.print(deviceID);
      Serial.println(":");
      

      for (size_t i = 0; i < finalDistances[deviceID].size(); i++) {
        Serial.print("To Device ");
        Serial.print(deviceID + i + 1);
        Serial.print(": ");
        Serial.print(finalDistances[deviceID][i]);
        Serial.println(" meters");
      }

      delay(10000);
    }
  }
}


// ----------------------------------------------


void printBuffer(const uint8_t* buffer, size_t bufferSize) {
    Serial.print("Buffer Content: ");
    for (size_t i = 0; i < bufferSize; i++) {
        if (buffer[i] < 0x10) {
            Serial.print("0"); // Print leading zero for single hex digit
        }
        Serial.print(buffer[i], HEX); // Print each byte in hexadecimal format
        Serial.print(" ");
    }
    Serial.println(); // New line after printing the entire buffer
}