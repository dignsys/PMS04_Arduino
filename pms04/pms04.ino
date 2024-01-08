/* 
 * PMS[01/04] Main 
 * Author : DIGNSYS Inc.
 */

#include <Arduino.h>
#include <SC16IS752Serial.h>
#include <PZEM004Tv30-pms.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include <SPI.h>
#include <Ethernet_Generic.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#define DISABLE_IR_FUNCTION
#ifndef DISABLE_IR_FUNCTION
#include <IRremoteESP8266.h>
#include <IRsend.h>
#endif

#define VERSION_PMS_FW  "20240108"

#define SYS_PMS01     1
#define SYS_PMS04     4
//#define SYS_PMS_HW    SYS_PMS01
#define SYS_PMS_HW   SYS_PMS04

SC16IS752Serial serial0 = SC16IS752Serial(0);
SC16IS752Serial serial1 = SC16IS752Serial(1);
#if (SYS_PMS_HW == SYS_PMS04)
SC16IS752Serial serial2 = SC16IS752Serial(2);
SC16IS752Serial serial3 = SC16IS752Serial(3);
#endif

PZEM004Tv30 pzem0(serial0);
PZEM004Tv30 pzem1(serial1);
#if (SYS_PMS_HW == SYS_PMS04)
PZEM004Tv30 pzem2(serial2);
PZEM004Tv30 pzem3(serial3);
#endif

uint8_t rx_buf[SC_RX_BUF_SIZE];
uint8_t tx_buf[SC_TX_BUF_SIZE];

#define PZEM004_SUB_TEST

#ifdef PZEM004_SUB_TEST
#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42

#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

bool checkCRC(const uint8_t *buf, uint16_t len);
void setCRC(uint8_t *buf, uint16_t len);
uint16_t CRC16(const uint8_t *data, uint16_t len);
#endif

#define PIN_I2C_SDA       23
#define PIN_I2C_SCL       22
#define PIN_RTC_I2C_SDA   13  //25
#define PIN_RTC_I2C_SCL   14  //33
#define PIN_RS232_TXD     26  //27
#define PIN_RS232_RXD     25  //26
#define PIN_RS485_TXD     16  //17
#define PIN_RS485_RXD     4   //16
#define PIN_ETH_CS        17  //5
#define PIN_ETH_INT       38  // SENSOR_VP
#define PIN_ETH_MISO      18  //19
#define PIN_ETH_SCLK      5   //18
#define PIN_ETH_MOSI      19  //23

#define PIN_PCM_CONTROL   27

#define PIN_GPIO          32
#define PIN_IR            33

#ifndef DISABLE_IR_FUNCTION
IRsend IrSender(PIN_IR);
#endif

#define I2C_ADDR_RTC    0x68

uint8_t rtc_tx_buf[16];
uint8_t rtc_rx_buf[16];
uint8_t str_cur_date[9] = {0,};
uint8_t str_cur_time[9] = {0,};

#define I2C_ADDR_IO     0x20

SoftWire sw(PIN_RTC_I2C_SDA, PIN_RTC_I2C_SCL);
void printTwoDigit(int n);
void readTime(void);

#define PIN_W5500_RST       34  // GPIO Expander
#define PIN_SC16IS752_RST   15

uint8_t nc_mac[] = {
  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
enum {
  IP_TYPE_DHCP,
  IP_TYPE_STATIC
};

IPAddress nc_ip(192, 168, 1, 35);
IPAddress nc_dns(8, 8, 8, 8);
IPAddress nc_gateway(192, 168, 1, 1);
IPAddress nc_subnet(255, 255, 255, 0);
IPAddress nc_server(192, 168, 1, 200);
uint8_t nc_ip_type = IP_TYPE_STATIC;

SPIClass* pspi;
DhcpClass* dhcp = new DhcpClass();

#define PIN_MAX485_DE       21    //4
#define MAX485_DIR_SEND     LOW   //HIGH
#define MAX485_DIR_RECEIVE  HIGH  //LOW

void setRS485Dir(bool dir);
int setAddress(char c, uint8_t* paddr, uint8_t* pch);

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_read_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);

#define LED_ON      1
#define LED_OFF     0
#define ETH_RST_ON  1
#define ETH_RST_OFF 0
#define SWITCH_ON   1
#define SWITCH_OFF  0

void gpio_exp_conf(void);
void gpio_exp_read(uint16_t* pdata);
void gpio_exp_write(uint16_t data);

void led_pm(uint8_t num, uint8_t on);
void led_tx(uint8_t on);
void led_rx(uint8_t on);
void led_fail(uint8_t on);

void eth_rst(void);

int get_swtich_val(uint8_t num);

void dual_uart_led_init(void);
void dual_uart_led_set(uint8_t num, uint8_t on);

#define EEPROM_SIZE           512
#define EEPROM_ADDR_SN        32

const char* na_str = "***";

char gv_ssid[32] = {0,};
char gv_passwd[32] = {0,};

WebServer server(80);
bool fsFound = false;

//holds the current upload
File fsUploadFile;
String FileName;

String HTML = "<form method='post' action='/upload' enctype='multipart/form-data'><input type='file' name='upload'><input type='submit' value='Upload'></form>";  

#define FILEBUFSIZ 4096

const char* host = "PMSMGR";
String logStr = "Session start:\n";
char tempBuf[256];

// main page JS scripts 
const char edit_script[] PROGMEM = R"rawliteral(
<script>
  var fName; 
  function doEdit(item) 
  {
    console.log('clicked', item); 
    fName = item;
    var fe = document.getElementById("nameSave");
    fe.value = fName;
  } 
  function saveFile()
  {
    console.log('Save', fName);
  }
  function scrollToBottom(el) // log window
  {
    //console.log("scrolling", el);
    var element = document.getElementById(el);
    element.scrollTop = element.scrollHeight;   
  }
</script>
)rawliteral";

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void writeFile2(fs::FS &fs, const char * path, const char * message);
void deleteFile2(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);
size_t LittleFSFilesize(const char* filename);
std::string ReadFileToString(const char* filename);
String formatBytes(size_t bytes);
void handleFileSysFormat(void);
String getContentType(String filename);
bool exists(String path);
bool handleFileRead(String path);
void handleFileUpload(void);
void handleFileDelete(void);
void handleFileCreate(void);
void handleMain(void);
bool initFS(bool format, bool force);
void lfs_log(char* pmsg);

void init_ethernet(void);
void update_settings(void);
void load_settings(void);
void update_serial_number(char* pstr);
void load_serial_number(char* pstr);

void sub_test_a(uint8_t ireg);
void sub_test_b(void);
void sub_test_c(void);
int32_t sub_test_c_01(uint32_t f, int32_t *bestErr);
void sub_test_d(void);
void sub_test_e(void);
void sub_test_f(void);
void sub_test_g(void);
void sub_test_h(void);
void sub_test_i(uint8_t iaddr);
void sub_test_l(void);
void sub_test_m(void);
void sub_test_n(void);
void sub_test_o(void);
void sub_test_p(void);
void sub_test_q(void);
void sub_test_r(void);
void sub_test_s(void);
void sub_test_t(void);
void sub_test_v(void);
void sub_test_y(void);
void sub_test_z(void);
void sub_test_loop(void);

// Run this once at power on or reset.
void setup() {

  // RTC Initialization
  sw.setTxBuffer(rtc_tx_buf, sizeof(rtc_tx_buf));
  sw.setRxBuffer(rtc_rx_buf, sizeof(rtc_rx_buf));
  sw.setDelay_us(5);
  sw.setTimeout(1000);
  sw.begin();

  // GPIO Expander
  gpio_exp_conf();

  eth_rst();

  pspi = new SPIClass(VSPI);
  pspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);

  // Reset SC16IS752
  pinMode(PIN_SC16IS752_RST, OUTPUT);
  digitalWrite(PIN_SC16IS752_RST, LOW);
  delay(100);
  digitalWrite(PIN_SC16IS752_RST, HIGH);
  delay(100);

  // GPIO
  pinMode(PIN_GPIO, OUTPUT);
  digitalWrite(PIN_GPIO, LOW);

  // IR
  pinMode(PIN_IR, OUTPUT);
  digitalWrite(PIN_IR, LOW);

  // PCM Control (LOW: AC Power-OFF, HIGH: AC Power-ON)
  pinMode(PIN_PCM_CONTROL, OUTPUT);
  digitalWrite(PIN_PCM_CONTROL, HIGH);

  // Arduino USART setup.
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RS232_RXD, PIN_RS232_TXD);  // RS232
  Serial2.begin(115200, SERIAL_8N1, PIN_RS485_RXD, PIN_RS485_TXD);  // RS485
  pinMode(PIN_MAX485_DE, OUTPUT);
  digitalWrite(PIN_MAX485_DE, MAX485_DIR_SEND);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.setClock(400000);  // 400kHz (For SC16IS752)

  // SC16IS752 UART setup. (begin is excuted here instead of pzem creation)
  serial0.begin(SC_REF_BAUDRATE);
  serial1.begin(SC_REF_BAUDRATE);
#if (SYS_PMS_HW == SYS_PMS04)
  serial2.begin(SC_REF_BAUDRATE);
  serial3.begin(SC_REF_BAUDRATE);
#endif

  // PZEM Initialization
  pzem0.init(&serial0, false, PZEM_DEFAULT_ADDR);
  pzem1.init(&serial1, false, PZEM_DEFAULT_ADDR);
#if (SYS_PMS_HW == SYS_PMS04)
  pzem2.init(&serial2, false, PZEM_DEFAULT_ADDR);
  pzem3.init(&serial3, false, PZEM_DEFAULT_ADDR);
#endif

  // GPIO in SC16IS752 (need to init after pzem ?)
  dual_uart_led_init();

#ifndef DISABLE_IR_FUNCTION
  IrSender.begin();
#endif

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  fsFound = initFS(false, false); // is an FS already in place?
  if(fsFound){
    LittleFS.exists("/init.txt");
    load_settings();
  }

  esp_read_mac(nc_mac, ESP_MAC_ETH);

}

// Process this loop whenever we see a serial event or interrupt from the SC16IS752
void loop() {

  Serial.println();
  Serial.println("PMS[01/04] Main Loop");
  Serial.println("(C) 2023 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_PMS_FW);

  char c = 0;
  int idx=0;
  int log_idx=0;
  uint8_t addr;
  PZEM004Tv30* ppzem[4];
  char log_msg[256] = {0,};
  uint8_t led_toggle = 0;

#if (SYS_PMS_HW == SYS_PMS04)
  ppzem[0] = &pzem3;
  ppzem[1] = &pzem2;
  ppzem[2] = &pzem1;
  ppzem[3] = &pzem0;
#elif (SYS_PMS_HW == SYS_PMS01)
  ppzem[0] = &pzem0;
#endif

  while(1) {

    if(Serial.available()) {
      c = Serial.read();
    }
    if(c == '#'){
      Serial.println("Go to Sub Test!!!");
      sub_test_loop();
      Serial.println("Return to Main Loop!!!");
      c = 0;
    }

    // PZEM Address Check
    addr = ppzem[idx]->readAddress();
    Serial.print("Custom Address[PM-"); Serial.print(idx+1); Serial.print("]: ");
    Serial.println(addr, HEX);

    if(!log_idx){
      memset(log_msg, 0x00, sizeof(log_msg));
      sprintf(log_msg, "Custom Address[PM-%d]: %02x", idx+1, addr);
      lfs_log(log_msg);
    }

    if(addr) {

      led_pm(idx+1, LED_ON);

      // Read the data from the sensor
      float voltage = ppzem[idx]->voltage();
      float current = ppzem[idx]->current();
      float power = ppzem[idx]->power();
      float energy = ppzem[idx]->energy();
      float frequency = ppzem[idx]->frequency();
      float pf = ppzem[idx]->pf();

      // Check if the data is valid
      if(isnan(voltage)){
          Serial.println("Error reading voltage");
      } else if (isnan(current)) {
          Serial.println("Error reading current");
      } else if (isnan(power)) {
          Serial.println("Error reading power");
      } else if (isnan(energy)) {
          Serial.println("Error reading energy");
      } else if (isnan(frequency)) {
          Serial.println("Error reading frequency");
      } else if (isnan(pf)) {
          Serial.println("Error reading power factor");
      } else {

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);

      }
      led_pm(idx+1, LED_OFF);
    }
    else{
      led_pm(idx+1, LED_OFF);
    }

    Serial.println();

#if (SYS_PMS_HW == SYS_PMS04)
    if(++idx > 3){
      idx = 0;
      if(++log_idx > 10){
        log_idx = 0;
      }
    }
#elif (SYS_PMS_HW == SYS_PMS01)
    if(++log_idx > 40){
      log_idx = 0;
    }
#endif
    if(led_toggle){
      led_toggle = 0;
      dual_uart_led_set(2, LED_ON);
      dual_uart_led_set(3, LED_OFF);
    } else {
      led_toggle = 1;
      dual_uart_led_set(2, LED_OFF);
      dual_uart_led_set(3, LED_ON);
    }
    delay(1000);

  }
  Serial.println("loop exit!");

}

void sub_test_loop() {

  Serial.println();
  Serial.println("PMS[01/04] Board Sub-testing.");
  Serial.println("(C) 2023 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_PMS_FW);

  char c;
  while(c != 'x') {
    Serial.printf("Input Command: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)){
          break;
        }
      }
      delay(100);
    }
    Serial.printf("%c", c);
    Serial.println();

    switch(c) {
      case 'a': 
        sub_test_a(0);
        break;
      case 'b':
        sub_test_b();
        break;
      case 'c':
        sub_test_c();
        break;
      case 'd':
        sub_test_d();
        break;
      case 'e':
        sub_test_e();
        break;
      case 'f':
        sub_test_f();
        break;
      case 'g':
        sub_test_g();
        break;
      case 'h':
        sub_test_h();
        break;
      case 'i':
        sub_test_i(0);
        break;
      case 'j':
        for(int i = 1; i < 0xf8; i++){
          sub_test_i((uint8_t)i);
        }
        break;
      case 'l':
        sub_test_l();
        break;
      case 'm':
        sub_test_m();
        break;
      case 'n':
        sub_test_n();
        break;
      case 'o':
        sub_test_o();
        break;
      case 'p':
        sub_test_p();
        break;
      case 'q':
        sub_test_q();
        break;
      case 'r':
        sub_test_r();
        break;
      case 's':
        sub_test_s();
        break;
      case 't':
        sub_test_t();
        break;
      case 'v':
        sub_test_v();
        break;
      case 'y':
        sub_test_y();
        break;
      case 'z':
        sub_test_z();
        break;
      default:
        break;
    }
  }
  Serial.println("loop exit!");

}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(addr == 0x10){
    Wire.endTransmission(false);
  } else {
    Wire.endTransmission();
  }
  
  if(addr == 0x10) {
      Wire.requestFrom(addr, (size_t)dlen, (bool) false);
  } else {
    Wire.requestFrom(addr, (uint8_t)dlen);
  }
  if (Wire.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = Wire.read();
    }
  }

  return ret;
}

int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  for(int i = 0; i < dlen; i++) {
    Wire.write(pdata[i]);
  }
  Wire.endTransmission();

  return ret;
}

int i2c_read_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw.beginTransmission(addr);
  sw.write(reg);
  sw.endTransmission();
  
  sw.requestFrom(addr, (uint8_t)dlen);

  if (sw.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = sw.read();
    }
  }

  return ret;
}

int i2c_write_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw.beginTransmission(addr);
  sw.write(reg);
  for(int i = 0; i < dlen; i++) {
    sw.write(pdata[i]);
  }
  sw.endTransmission();

  return ret;
}

void gpio_exp_conf(void){

  uint8_t data[3] = {0,};

  data[0] = 0xff;
  data[1] = 0x00;

  // IN/OUT set
  i2c_write_sw(I2C_ADDR_IO, 0x06, data, 2);

}

void gpio_exp_read(uint16_t* pdata){

  uint8_t rdata[3] = {0,};

  i2c_read_sw(I2C_ADDR_IO, 0x00, rdata, 2);

  *pdata = rdata[1] * 0x100 + rdata[0];

}

void gpio_exp_write(uint16_t data){

  uint8_t wdata[3] = {0,};

  wdata[0] = (uint8_t) (data & 0x00ff);
  wdata[1] = (uint8_t) ((data>>8) & 0x00ff);

  i2c_write_sw(I2C_ADDR_IO, 0x02, wdata, 2);

}

void led_pm(uint8_t num, uint8_t on){

  uint16_t num_bit;
  uint16_t data = 0;

  if(!num || (num > 4)){
    Serial.print("Invalid PM LED[1~4] Set:"); Serial.println(num);
    return;
  }

  gpio_exp_read(&data);

  if(on == LED_OFF){
    num_bit = 0x1000 << (num-1);
    data |= num_bit;
  } else {
    num_bit = ~(0x1000 << (num-1));
    data &= num_bit;
  }
  
  gpio_exp_write(data);

}

void led_tx(uint8_t on){

  uint16_t data = 0;

  gpio_exp_read(&data);

  if(on == LED_OFF){
    data |= 0x200;
  } else {
    data &= 0xfdff;
  }
  
  gpio_exp_write(data);

}

void led_rx(uint8_t on){

  uint16_t data = 0;

  gpio_exp_read(&data);

  if(on == LED_OFF){
    data |= 0x400;
  } else {
    data &= 0xfbff;
  }
  
  gpio_exp_write(data);

}

void led_fail(uint8_t on){

  uint16_t data = 0;

  gpio_exp_read(&data);

  if(on == LED_OFF){
    data |= 0x100;
  } else {
    data &= 0xfeff;
  }
  
  gpio_exp_write(data);

}

void eth_rst(void){

  uint16_t data = 0;

  gpio_exp_read(&data);

  data &= 0xf7ff;
  gpio_exp_write(data);

  delay(20);

  data |= 0x800;
  gpio_exp_write(data);

}

int get_swtich_val(uint8_t num){

  int ret = 0;
  uint16_t num_bit;
  uint16_t data = 0;

  if(!num || (num > 8)){
    Serial.print("Invalid Switch[1~8] Set:"); Serial.println(num);
    return ret;
  }

  num_bit = 0x01 << (num-1);
  gpio_exp_read(&data);

  if(data & num_bit) {
    ret = SWITCH_OFF;
  } else {
    ret = SWITCH_ON;
  }

  return ret;
}

void dual_uart_led_init(void){

  uint8_t data[2] = {0,};

  // device 0
  // IO Control
  i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOCONTROL_REG<<3, data, 1);
  data[0] &= 0xfb;
  i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOCONTROL_REG<<3, data, 1);

  // IO Direction
  i2c_read(SC16IS752_SADDR0, SC16IS7XX_IODIR_REG<<3, data, 1);
  data[0] |= 0x03;
  i2c_write(SC16IS752_SADDR0, SC16IS7XX_IODIR_REG<<3, data, 1);

  // IO State
  i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
  data[0] &= 0xfc;
  i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);

#if (SYS_PMS_HW == SYS_PMS04)
  // device 1
  // IO Control
  i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOCONTROL_REG<<3, data, 1);
  data[0] &= 0xfb;
  i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOCONTROL_REG<<3, data, 1);

  // IO Direction
  i2c_read(SC16IS752_SADDR1, SC16IS7XX_IODIR_REG<<3, data, 1);
  data[0] |= 0x03;
  i2c_write(SC16IS752_SADDR1, SC16IS7XX_IODIR_REG<<3, data, 1);

  // IO State
  i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
  data[0] &= 0xfc;
  i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
#endif

}

void dual_uart_led_set(uint8_t num, uint8_t on){

  uint8_t data[2] = {0,};

#if (SYS_PMS_HW == SYS_PMS04)
  if((num < 2) || (num > 5)){
    Serial.print("Invalid Dual Uart LED[2~5] Set:"); Serial.println(num);
    return;
  }
#elif (SYS_PMS_HW == SYS_PMS01)
  if((num < 2) || (num > 3)){
    Serial.print("Invalid Dual Uart LED[2~3] Set:"); Serial.println(num);
    return;
  }
#endif

  if(num == 2){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x01;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfe;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  } else if(num == 3){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x02;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfd;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  } else if(num == 4){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x01;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfe;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  } else if(num == 5){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x02;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfd;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  }
}

// Print with leading zero, as expected for time
void printTwoDigit(int n)
{
  if (n < 10) {
    Serial.print('0');
  }
  Serial.print(n);
}

void readTime(void)
{
  // Ensure register address is valid
  sw.beginTransmission(I2C_ADDR_RTC);
  sw.write(uint8_t(0)); // Access the first register
  sw.endTransmission();

  uint8_t registers[7]; // There are 7 registers we need to read from to get the date and time.
  int numBytes = sw.requestFrom(I2C_ADDR_RTC, (uint8_t)7);
  for (int i = 0; i < numBytes; ++i) {
    registers[i] = sw.read();
  }
  if (numBytes != 7) {
    Serial.print("Read wrong number of bytes: ");
    Serial.println((int)numBytes);
    return;
  }

  int tenYear = (registers[6] & 0xf0) >> 4;
  int unitYear = registers[6] & 0x0f;
  int year = (10 * tenYear) + unitYear;

  int tenMonth = (registers[5] & 0x10) >> 4;
  int unitMonth = registers[5] & 0x0f;
  int month = (10 * tenMonth) + unitMonth;

  int tenDateOfMonth = (registers[4] & 0x30) >> 4;
  int unitDateOfMonth = registers[4] & 0x0f;
  int dateOfMonth = (10 * tenDateOfMonth) + unitDateOfMonth;

  // Reading the hour is messy. See the datasheet for register details!
  bool twelveHour = registers[2] & 0x40;
  bool pm = false;
  int unitHour;
  int tenHour;
  if (twelveHour) {
    pm = registers[2] & 0x20;
    tenHour = (registers[2] & 0x10) >> 4;
  } else {
    tenHour = (registers[2] & 0x30) >> 4;
  }
  unitHour = registers[2] & 0x0f;
  int hour = (10 * tenHour) + unitHour;
  if (twelveHour) {
    // 12h clock? Convert to 24h.
    hour += 12;
  }

  int tenMinute = (registers[1] & 0xf0) >> 4;
  int unitMinute = registers[1] & 0x0f;
  int minute = (10 * tenMinute) + unitMinute;

  int tenSecond = (registers[0] & 0xf0) >> 4;
  int unitSecond = registers[0] & 0x0f;
  int second = (10 * tenSecond) + unitSecond;

  // ISO8601 is the only sensible time format
  Serial.print("Time: ");
  Serial.print(year);
  Serial.print('-');
  printTwoDigit(month);
  Serial.print('-');
  printTwoDigit(dateOfMonth);
  Serial.print('T');
  printTwoDigit(hour);
  Serial.print(':');
  printTwoDigit(minute);
  Serial.print(':');
  printTwoDigit(second);
  Serial.println();

  sprintf((char*) str_cur_date, "%02d-%02d-%02d", year, month, dateOfMonth);
  sprintf((char*) str_cur_time, "%02d:%02d:%02d", hour, minute, second);
  Serial.printf("cur_date: %s\r\n", str_cur_date);
  Serial.printf("cur_time: %s\r\n", str_cur_time);
}

void setRS485Dir(bool dir) {

  if(dir == MAX485_DIR_SEND) {
    digitalWrite(PIN_MAX485_DE, MAX485_DIR_SEND);
  } else if(dir == MAX485_DIR_RECEIVE) {
    digitalWrite(PIN_MAX485_DE, MAX485_DIR_RECEIVE);
  } else {
    Serial.println("Invalid set of MAX485");
  }
}

int setAddress(char c, uint8_t* paddr, uint8_t* pch) {

  int ret = 0;

  if(c == '0') {
    *paddr = SC16IS752_SADDR0; *pch = SC16IS752_CHANNEL_A;
  } else if (c == '1') {
    *paddr = SC16IS752_SADDR0; *pch = SC16IS752_CHANNEL_B;
  } else if (c == '2') {
    *paddr = SC16IS752_SADDR1; *pch = SC16IS752_CHANNEL_A;
  } else if (c == '3') {
    *paddr = SC16IS752_SADDR1; *pch = SC16IS752_CHANNEL_B;
  } else {
    ret = 1;
  }

  return ret;
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("- failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.path(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("- failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\r\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("- file renamed");
  } else {
    Serial.println("- rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\r\n", path);
  if(fs.remove(path)){
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

// SPIFFS-like write and delete file, better use #define CONFIG_LITTLEFS_SPIFFS_COMPAT 1
void writeFile2(fs::FS &fs, const char * path, const char * message){
  if(!fs.exists(path)){
		if (strchr(path, '/')) {
            Serial.printf("Create missing folders of: %s\r\n", path);
			char *pathStr = strdup(path);
			if (pathStr) {
				char *ptr = strchr(pathStr, '/');
				while (ptr) {
					*ptr = 0;
					fs.mkdir(pathStr);
					*ptr = '/';
					ptr = strchr(ptr+1, '/');
				}
			}
			free(pathStr);
		}
  }

  Serial.printf("Writing file to: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void deleteFile2(fs::FS &fs, const char * path){
  Serial.printf("Deleting file and empty folders on path: %s\r\n", path);

  if(fs.remove(path)){
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }

  char *pathStr = strdup(path);
  if (pathStr) {
    char *ptr = strrchr(pathStr, '/');
    if (ptr) {
      Serial.printf("Removing all empty folders on path: %s\r\n", path);
    }
    while (ptr) {
      *ptr = 0;
      fs.rmdir(pathStr);
      ptr = strrchr(pathStr, '/');
    }
    free(pathStr);
  }
}

void testFileIO(fs::FS &fs, const char * path){
  Serial.printf("Testing file I/O with %s\r\n", path);

  static uint8_t buf[512];
  size_t len = 0;
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }

  size_t i;
  Serial.print("- writing" );
  uint32_t start = millis();
  for(i=0; i<2048; i++){
    if ((i & 0x001F) == 0x001F){
      Serial.print(".");
    }
    file.write(buf, 512);
  }
  Serial.println("");
  uint32_t end = millis() - start;
  Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
  file.close();

  file = fs.open(path);
  start = millis();
  end = start;
  i = 0;
  if(file && !file.isDirectory()){
    len = file.size();
    size_t flen = len;
    start = millis();
    Serial.print("- reading" );
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      if ((i++ & 0x001F) == 0x001F){
        Serial.print(".");
      }
      len -= toRead;
    }
    Serial.println("");
    end = millis() - start;
    Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
    file.close();
  } else {
    Serial.println("- failed to open file for reading");
  }
}

size_t LittleFSFilesize(const char* filename) {
  auto file = LittleFS.open(filename, "r");
  size_t filesize = file.size();
  // Don't forget to clean up!
  file.close();
  return filesize;
}

std::string ReadFileToString(const char* filename) {
  auto file = LittleFS.open(filename, "r");
  size_t filesize = file.size();
  // Read into temporary Arduino String
  String data = file.readString();
  // Don't forget to clean up!
  file.close();
  return std::string(data.c_str(), data.length());
}

//format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

void handleFileSysFormat() {
	LittleFS.format();
	server.send(200, "text/json", "format complete");
}

String getContentType(String filename) {
  if (server.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  } else if (filename.endsWith(".png")) {
    return "image/wav";
  }
  return "text/plain";
}

bool exists(String path){
  bool yes = false;  
  File file = LittleFS.open(path, "r");
  if(!file.isDirectory()){
    yes = true;
  }
  file.close();
  return yes;
}

bool handleFileRead(String path) {
  Serial.printf_P(PSTR("handleFileRead: %s\r\n"), path.c_str());
  if(path.endsWith("/")) path += "";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(LittleFS.exists(pathWithGz) || LittleFS.exists(path))
	{
    if(LittleFS.exists(pathWithGz))
      path += ".gz";
    File file = LittleFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
		Serial.println("Read OK");
    return true;
  }
	Serial.printf("Read failed '%s', type '%s'\n", path.c_str(), contentType.c_str()) ;
  return false;
}

void handleFileUpload() {  
  bool OK = false;
  if(server.uri() != "/edit") return;
 
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START)
  {
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.printf_P(PSTR("handleFileUpload Name: %s\r\n"), filename.c_str());
    fsUploadFile = LittleFS.open(filename, "w");
    filename = String();
  } 
  else if(upload.status == UPLOAD_FILE_WRITE)
  {
    Serial.printf_P(PSTR("handleFileUpload Data: %d\r\n"), upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } 
  else if(upload.status == UPLOAD_FILE_END)
  {
    if(fsUploadFile)
      fsUploadFile.close();    
    OK = true;
    Serial.printf_P(PSTR("handleFileUpload Size: %d\r\n"), upload.totalSize);
    sprintf(tempBuf,"File upload [%s] %s\n", upload.filename.c_str(), (OK)? "OK" : "failed");
    logStr += tempBuf;
  }
}

void handleFileDelete() {
	if(!server.hasArg("file")) {server.send(500, "text/html", "<meta http-equiv='refresh' content='1;url=/main'>Bad arguments. <a href=/main>Back to list</a>"); return;}
  //if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");  
	String path = server.arg("file");
	//String path = server.arg(0);
  Serial.printf_P(PSTR("handleFileDelete: '%s'\r\n"),path.c_str());
  if(path == "/")
    return server.send(500, "text/html", "<meta http-equiv='refresh' content='1;url=/main>Can't delete root directory. <a href=/main>Back to list</a>");
  if(!LittleFS.exists(path))
    return server.send(200, "text/html", "<meta http-equiv='refresh' content='1;url=/main'>File not found. <a href=/main>Back to list</a>");
  LittleFS.remove(path);
  server.send(200, "text/html", "<meta http-equiv='refresh' content='1;url=/main'>File deleted. <a href=/main>Back to list</a>");
  logStr += "Deleted ";
  logStr += path;
  logStr +="\n";
  path = String();
}

void handleFileCreate() {
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.printf_P(PSTR("handleFileCreate: %s\r\n"),path.c_str());
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(LittleFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  if(!path.startsWith("/")) path = "/"+path;    // is this needed for LittleFS?
  File file = LittleFS.open(path, "w");
  if(file)
	{
    file.close();
		Serial.printf("Created file [%s]\n",path.c_str());
	}
  else
	{
		Serial.printf("Create file [%s] failed\n",path.c_str());
    return server.send(500, "text/plain", "CREATE FAILED");
	}
  server.send(200, "text/html", "<meta http-equiv='refresh' content='1;url=/main'>File created. <a href=/main>Back to list</a>");
  logStr += "Created ";
  logStr += path;
  logStr +="\n";
  path = String();
}

void handleMain() {
  bool foundText = false;
  bool foundName = false;
  bool foundMode = false;
  bool foundSaveBut = false;
  char filebuf[FILEBUFSIZ];
  char fileName[128];
  File file;
	String path = "", bText = "", bName = "", bMode ="";
  String output = "";
  // check arguments
  if(server.hasArg("mode"))
  {
    bMode = server.arg("mode");
    if(bMode.length() > 0) 
      foundMode = true;
    Serial.printf("Mode %s\n",bMode.c_str());
  }
  if(server.hasArg("dir"))// {server.send(500, "text/plain", "BAD ARGS"); return;}  
		path = server.arg("dir");
	else
		path ="/";
  if(server.hasArg("editBlock"))
  {
    bText = server.arg("editBlock");
    if(bText.length() > 0) 
      foundText = true;
  }
  
  if(server.hasArg("nameSave"))
  {
    bName = server.arg("nameSave");
    if(bName.length() > 0) 
      foundName = true;
    if(!bName.startsWith("/")) bName = "/"+ bName;    // is this needed for LittleFS?   
  }
  
  if(server.hasArg("saveBut"))
  {
    foundSaveBut = true;
  }

  // write
  if(foundName && foundText && bMode == "save")
  {
    Serial.println("something to save");
    file = LittleFS.open(bName, "w");
    if(file)
    {
      file.write((uint8_t *)bText.c_str(), bText.length());
      file.close();
      logStr += "Saved ";
      logStr += bName;
      logStr +="\n";
    }  
  }
  Serial.printf_P(PSTR("handleMain: path [%s]\r\n"), path.c_str());
  Serial.printf_P(PSTR("fname:[%s], %i\r\n"), bName.c_str(), foundName);
  Serial.printf_P(PSTR("text [%s], %i\r\n"),  bText.c_str(), foundText);
  Serial.printf_P(PSTR("mode [%s], %i\r\n"),  bMode.c_str(), foundMode);

  File dir = LittleFS.open(path.c_str());
  if(!dir)
	  Serial.printf("Directory [%s]not found", path.c_str());
	else if(!dir.isDirectory())
        Serial.println(" - not a directory");
	//path = String();

  // Create HTML page
  output = "<html><head>";
  output += "</head><body onload='scrollToBottom(\"log\")'>\n";
  output += "<span style='text-align: center;'><h2>PMS File Management</H2></span>";
  output += "<table style='margin-left: auto;  margin-right: auto;border-collapse: collapse'>\n"; // style='border: 1px solid silver; border-collapse: collapse;'
  
  // FS format 
  if(bMode == "format" && foundMode)
  {
    fsFound = initFS(true, true);
    Serial.println("main: Done formatting");
    logStr += "Formatted FS ";
    logStr += "LittleFS";
    logStr +="\n";      
  }
  output += "<tr><td style='background-color: #fff0ff; border: 1px solid silver;  padding: 5px;vertical-align:top;' colspan='2'><h3>";
  output += "LittleFS";
  output += " file system</h3>";
  output += "LIttleFS";
  output += " filesystem ";
  if (!fsFound)
    output += "not ";
  output += "found.";
  // format form
  output += "<span style='text-align: right;'><form action='/main?mode=format' method='get' enctype='multipart/form-data'>"; 
  output += "<input type='hidden' name='mode' value='format'>";
  output += "<button>Format FS</button>";
  output += "</form></span>";  
  output += "</td></tr>\n";
   
  //file upload 
  output += "<tr><td style='background-color: #e4ffe4; border: 1px solid silver;  padding: 5px;vertical-align:top;' colspan='2'><h3>";
  output += "<h3>Upload files</h3><form action='/edit' method='post' enctype='multipart/form-data'><BR>"; // use post for PUT /edit server handler
  output += "Name: <input type='file' name='data' required>";
  output += "Path: <input type='text' name='path' value='/'>";
  output += "<input type='hidden' name='mode' value='upload'>";
  output += "<button>Upload</button>";
  output += "</form>";
  output += "</td></tr>\n";
  
  // file list and edit   
  output += "<tr style='background-color: #ffffde; border: 1px solid silver;'><td style='padding:5px; vertical-align:top;'>";
	output += "<h3>Files in directory '" + path + "'</h3>";
	output += "<a href=/main>Back to root</a><br>"; // LittleFS only

	File entry;
  while(entry = dir.openNextFile())
  {  
	  bool isDir = entry.isDirectory(); // single level for SPIFFS, possibly not true for LittleFS
    // output += (isDir) ? "dir:" : "file: ";
    // output += "\t";
		if(isDir) 
		{
			output += "<a href=/main?dir=/" ;
			output +=  entry.name(); 
			output +=  ">";
		}
    strcpy(fileName, entry.name());
    output += String(entry.name());
		if(isDir) 
		  output += "</a>";
		output += " (";
    output += String(entry.size());
		output += ")&nbsp;&nbsp;";
    // edit
    output += "<a href=/main?mode=edit&nameSave="; 
    if(fileName[0] != '\\' && fileName[0] != '/') // avoid double \ or / in filename (on some OS)
   	 output += path;
    output += String(entry.name());
    output += ">Edit</a>&nbsp;&nbsp;";
    // delete
    output += "<a href=/delete?file="; 
    if(fileName[0] != '\\' && fileName[0] != '/') // avoid double \ or / in filename (on some OS)
    output += path;
    output += String(entry.name());
    output += ">Delete</a><BR>";
    entry.close();
  }	
  output += "</td>\n";  
  // edit form - text, filename and submit
  output += "<td style='padding: 5px;'><form action='/main' method='get'><textarea name=editBlock id=editBlock rows='30' cols='60'>";
  
  if(bMode = "edit")
  {
    // read file and insert content
    file = LittleFS.open(bName.c_str(), "r");
    if(file)
    {
      Serial.printf("File read avail %i, ",file.available());
      int readlen = (file.available() < FILEBUFSIZ) ? file.available() : FILEBUFSIZ;   
      file.read((uint8_t *)filebuf, readlen);
      file.close();        
      filebuf[readlen] = '\0';
      output += filebuf;
      //Serial.printf("read len %i, text [%s]\n",readlen, filebuf);
      logStr += (foundSaveBut) ? "Saving " : "Editing ";
      logStr += bName;
      logStr +="\n";   
    }
  }
  output += "</textarea><BR>\n";
  output += "<input type=hidden name='mode' value='save'>";
  output += "Filename: <input type=text value='";
  if(bMode = "edit")
    output += bName;        
  output += "' name=nameSave id=nameSave> <input type=submit name=saveBut onclick=saveFile() value='Save'></form></td></tr>\n";
  output += "<tr style='background-color: #ececff; border: 1px solid silver; vertical-align:top;'><td colspan=2 style='padding:5px; vertical-align:top;'>";
  output += "Log: <form><textarea  name=log id=log rows='5' cols='85'>";
  output += logStr;
  output += "</textarea></form>";
  output += "<BR><a href='/main'>Reload page</a>";
  output += "</td></tr></table></body>";
  output += edit_script;
  output += "</html>";
  //server.send(200, "text/json", output);
  server.send(200, "text/html", output);
}

bool initFS(bool format = false, bool force = false) 
{
  bool fsFound = LittleFS.begin();
  if(!fsFound) 
    Serial.println(F("No file system found. Please format it."));
     
  if(!format)
    return fsFound;
  
  // format
	if(!fsFound || force)
	{
    Serial.println(F("Formatting FS."));
	  if(LittleFS.format())
    {
		  if(LittleFS.begin())
      {
        Serial.println(F("Format complete."));
        return true;
      }
    }      
    Serial.println(F("Format failed."));
    return false;              
	}
	//fsList();
  return false; // shouldn't get here
}

void lfs_log(char* pmsg){

  if(!fsFound) return;

  readTime();

  if(LittleFS.exists("/log_today.txt")){
    // Check file date
    File file = LittleFS.open("/log_today.txt");
    String str_tmp = file.readStringUntil(0x0d);
    file.close();
    if(str_tmp.compareTo(String((char*) str_cur_date))){
      Serial.printf("Diff: %s, %s\r\n", str_tmp.c_str(), str_cur_date);
      if(LittleFS.exists("/log_past.txt")){
        LittleFS.remove("/log_past.txt");
      }
      LittleFS.rename("/log_today.txt", "/log_past.txt");
    } else {
      Serial.printf("Same: %s, %s\r\n", str_tmp.c_str(), str_cur_date);
    }
  }

  if(!LittleFS.exists("/log_today.txt")){
    writeFile(LittleFS, "/log_today.txt", (String((char*) str_cur_date)+ "\r\n").c_str());
  }
  appendFile(LittleFS, "/log_today.txt", (String((char*) str_cur_time)+" - "+String(pmsg)+"\r\n").c_str());

}

void update_settings(void){

  uint8_t mdata[6] = {0,};
  uint8_t sdata[32] = {0,};
  uint8_t str_tmp[128] = {0,};

  writeFile(LittleFS, "/init.txt", "=== PMS Settings ===\r\n");

  memset(str_tmp, 0x00, sizeof(str_tmp));
  if(gv_ssid[0]){
    sprintf((char*) str_tmp, "[WLAN/SSID]: %s", gv_ssid);
  } else {
    sprintf((char*) str_tmp, "[WLAN/SSID]: ***");
  }
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  if(gv_passwd[0]){
    sprintf((char*) str_tmp, "[WLAN/PASSWD]: %s", gv_passwd);
  } else {
    sprintf((char*) str_tmp, "[WLAN/PASSWD]: ***");
  }
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  if(nc_ip_type == IP_TYPE_STATIC){
    sprintf((char*) str_tmp, "[LAN/Static|DHCP]: Static");
  } else {
    sprintf((char*) str_tmp, "[LAN/Static|DHCP]: DHCP");
  }
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  sprintf((char*) str_tmp, "[LAN/IP]: %s", (nc_ip.toString()).c_str());
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  sprintf((char*) str_tmp, "[LAN/Subnet]: %s", (nc_subnet.toString()).c_str());
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  sprintf((char*) str_tmp, "[LAN/Gateway]: %s", (nc_gateway.toString()).c_str());
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  sprintf((char*) str_tmp, "[LAN/DNS]: %s", (nc_dns.toString()).c_str());
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  sprintf((char*) str_tmp, "[Server/IP]: %s", (nc_server.toString()).c_str());
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  esp_read_mac(mdata, ESP_MAC_WIFI_STA);
  memset(str_tmp, 0x00, sizeof(str_tmp));
  sprintf((char*) str_tmp, "[System/ID]: %02x:%02x:%02x:%02x:%02x:%02x", mdata[0], mdata[1], mdata[2], mdata[3], mdata[4], mdata[5]);
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

  memset(str_tmp, 0x00, sizeof(str_tmp));
  load_serial_number((char*) sdata);
  sprintf((char*) str_tmp, "[System/SN]: %s", sdata);
  appendFile(LittleFS, "/init.txt", (String((char*) str_tmp)+ "\r\n").c_str());

}

void load_settings(void){

  int slen = 0;
  uint8_t str_tmp[128] = {0,};

  File file = LittleFS.open("/init.txt");
  if(!file || file.isDirectory()){
    Serial.println("Fail to open init.txt");
  }

  file.find("[WLAN/SSID]:");
  String str_temp = file.readStringUntil('\n');
  str_temp.trim();

  if(0 != str_temp.compareTo(na_str)){
    memset(gv_ssid, 0x00, sizeof(gv_ssid));
    strcpy(gv_ssid, str_temp.c_str());
  } else {
    Serial.println("SSID is not valid");
  }

  file.find("[WLAN/PASSWD]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();

  if(0 != str_temp.compareTo(na_str)){
    memset(gv_passwd, 0x00, sizeof(gv_passwd));
    strcpy(gv_passwd, str_temp.c_str());
  } else {
    Serial.println("PASSWD is not valid");
  }

  file.find("[LAN/Static|DHCP]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();
  str_temp.toLowerCase();
  if(0 == str_temp.compareTo("static")){
    nc_ip_type = IP_TYPE_STATIC;
  } else if(0 == str_temp.compareTo("dhcp")){
    nc_ip_type = IP_TYPE_DHCP;
  } else {
    nc_ip_type = IP_TYPE_STATIC;
  }

  file.find("[LAN/IP]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();
  nc_ip.fromString(str_temp);

  file.find("[LAN/Subnet]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();
  nc_subnet.fromString(str_temp);

  file.find("[LAN/Gateway]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();
  nc_gateway.fromString(str_temp);

  file.find("[LAN/DNS]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();
  nc_dns.fromString(str_temp);

  file.find("[Server/IP]:");
  str_temp = file.readStringUntil('\n');
  str_temp.trim();
  nc_server.fromString(str_temp);

  file.close();
}

void update_serial_number(char* pstr){

  int slen;

  slen = strlen(pstr);
  
  for(int i = 0; i < 16; i++){
    if(i < slen){
      EEPROM.write(EEPROM_ADDR_SN+i, pstr[i]);
    } else {
      EEPROM.write(EEPROM_ADDR_SN+i, 0x00);
    }
    Serial.printf("%d - STR_IN: %02x, EEPROM: %02x\r\n", i, pstr[i], EEPROM.read(EEPROM_ADDR_SN+i));
  }
  EEPROM.commit();

}

void load_serial_number(char* pstr){

  for(int i = 0; i < 16; i++){
    pstr[i] = EEPROM.read(EEPROM_ADDR_SN+i);
    Serial.printf("EEPROM[%d]: %02x\r\n", i, pstr[i]);
  }
}

void sub_test_a(uint8_t ireg) {

  uint8_t address = SC16IS752_SADDR0;
  uint8_t channel;
  uint8_t reg = 0;
  uint8_t out = 0;
  char c;
  Serial.println("Sub-test A - Read I2C");

  if(ireg) {
    if(ireg > 0x25) {
      Serial.println("Invalid Register Address");
      return;
    }
    reg = ireg;
  }
  else {
    Serial.print("Input Port (0~3): ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    if(setAddress(c, &address, &channel)) {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);
  }

  // Considering SC16IS752
  reg = (reg<<3)|(channel<<1);
  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) {
    out = Wire.read();
  }
  Serial.print("I2C Read: address: "); Serial.print(address, HEX);
  Serial.print(", register: "); Serial.print(reg, HEX);
  Serial.print(", value: "); Serial.println(out, HEX);

}

void sub_test_b(void) {

  uint8_t address = SC16IS752_SADDR0;
  uint8_t channel;
  uint8_t reg = 0;
  uint8_t val = 0;
  char c;
  Serial.println("Sub-test B - Write I2C");

  Serial.print("Input Port (0~3): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }

  if(setAddress(c, &address, &channel)) {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Register Address: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    reg = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    reg |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Value to write: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    val = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'f')) {
    val = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    val |= (c - '0');
  } else if ((c >= 'a') && (c <= 'f')) {
    val |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  // Considering SC16IS752
  reg = (reg<<3)|(channel<<1);
  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();

  Serial.print("I2C Write: address: "); Serial.print(address, HEX);
  Serial.print(", register: "); Serial.print(reg, HEX);
  Serial.print(", value: "); Serial.println(val, HEX);

}

int32_t sub_test_c_01(uint32_t f, int32_t *bestErr) {

  // Use baudRate 115200 for calculate error
  int32_t err = f % (460800 * 16);

  if ((*bestErr < 0) || (*bestErr > err)) {
    *bestErr = err;
    return 0;
  }

  return 1;
}

void sub_test_c(void) {

  uint8_t address;
  uint32_t freq = SC_REF_FREQ;
  bool xtal;
  uint32_t div, clksrc, pllcfg = 0;
  int32_t bestErr = -1;
  uint32_t fdiv, fmul, bestfreq = freq;

  Serial.println("Sub-test C - Select Clock Setting");
  // Reset Port
#if 0  // MAX14830 related code
  // First, update error without PLL
  sub_test_c_01(freq, &bestErr);

  // Try all possible PLL dividers
  for (div = 1; (div <= 63) && bestErr; div++) {
    fdiv = DIV_ROUND_CLOSEST(freq, div);
    Serial.print("fdiv: "); Serial.print(fdiv, DEC);
    Serial.print(", freq: "); Serial.print(freq, DEC);
    Serial.print(", div: "); Serial.println(div, DEC);

    // Try multiplier 6
    fmul = fdiv * 6;
    if ((fdiv >= 500000) && (fdiv <= 800000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (0 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 48 
    fmul = fdiv * 48;
    if ((fdiv >= 850000) && (fdiv <= 1200000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (1 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 96
    fmul = fdiv * 96;
    if ((fdiv >= 425000) && (fdiv <= 1000000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (2 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 144
    fmul = fdiv * 144;
    if ((fdiv >= 390000) && (fdiv <= 667000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (3 << 6) | div;
        bestfreq = fmul;
      }
      
    Serial.print("bestfreq: "); Serial.print(bestfreq, DEC);
    Serial.print(", bestErr: "); Serial.println(bestErr, DEC);
  }
  Serial.print("pllcfg: "); Serial.println(pllcfg, HEX);
#endif
}

void sub_test_d(void) {

  uint8_t addr;
  Serial.println("Sub-test D - Access PZEM");

  addr = pzem0.getAddress();

  Serial.print("address: "); Serial.println(addr, HEX);
}

#ifdef PZEM004_SUB_TEST
bool checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}

void setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};

uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

#endif

void sub_test_e(void) {

  uint8_t addr = SC16IS752_SADDR0;

  uint8_t tbuf[SC_TX_BUF_SIZE] = {0,};
  uint8_t rbuf[SC_RX_BUF_SIZE] = {0,};
  uint16_t tlength = 8;
  uint16_t rlength = 8;
  uint8_t index = 0;
  Serial.println("Sub-test E - PZEM Command #1");

  //serial0.fifoClear();
#if 0
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ADDR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
#elif 0
  // read power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x02;
#elif 1
  // read data
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RIR;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tbuf[4] = 0x00;
  tbuf[5] = 0x0a;
  rlength = 25;
#else
  // set power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_WSR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x55;
  tbuf[5] = 0xaa;
#endif
  setCRC(tbuf, tlength);
  
  while(tlength--) {
    Wire.beginTransmission(addr);
    Wire.write((SC16IS7XX_THR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
    Wire.write(tbuf[index]);
    Wire.endTransmission();
    index++;
  }

  delay(600);

  index = 0;
  Wire.beginTransmission(addr);
  Wire.write((SC16IS7XX_RHR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) rlength);

  while(rlength) {
    if (Wire.available()) {
      rbuf[index] = Wire.read();
      Serial.print("rbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(rbuf[index], HEX);
      index++;
      rlength--;
    }
  }

}

void sub_test_f(void) {

  uint8_t addr = SC16IS752_SADDR0;
  uint8_t tbuf[SC_TX_BUF_SIZE] = {0,};
  uint16_t length = 8;
  uint8_t index = 0;
  char c;
  Serial.println("Sub-test F - writeData");

  Serial.print("Input Transfer Length (1 dec): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length = (c - '0');
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  for(int i=0; i <(length-2); i++){

    Serial.print("Input Buffer Data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      tbuf[i] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      tbuf[i] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      tbuf[i] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      tbuf[i] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

  }

  setCRC(tbuf, length);
  
  while(length--) {
    Wire.beginTransmission(addr);
    Wire.write((SC16IS7XX_THR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
    Wire.write(tbuf[index]);
    Serial.print("tbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(tbuf[index], HEX);
    Wire.endTransmission();
    index++;
  }

}

void sub_test_g(void) {

  uint8_t addr = SC16IS752_SADDR0;
  uint8_t buf[SC_RX_BUF_SIZE];
  uint16_t length = 8;
  uint8_t index = 0;
  char c;
  Serial.println("Sub-test G - readData");

  Serial.print("Input Total Length (2 dec): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length = (c - '0')*10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length += (c - '0');
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Wire.beginTransmission(addr);
  Wire.write((SC16IS7XX_RHR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) length);

  while(length) {
    if (Wire.available()) {
      buf[index] = Wire.read();
      Serial.print("buf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(buf[index], HEX);
      index++;
      length--;
    }
  }

}

void sub_test_h(void) {

  uint8_t addr = SC16IS752_SADDR0;
  uint8_t channel;
  uint8_t tbuf[SC_TX_BUF_SIZE] = {0,};
  uint8_t rbuf[SC_RX_BUF_SIZE] = {0,};
  uint16_t tlength = 8;
  uint16_t rlength = 8;
  uint8_t index = 0;
  char c;
  Serial.println("Sub-test H - PZEM Command #2");

  Serial.print("Input Port (0~3): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }

  if(setAddress(c, &addr, &channel)) {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  //serial0.fifoClear();
#if 0
  // reset energy
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_REST;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tlength = 4;
  rlength = 5;
#elif 1
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ADDR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
  rlength = 7;
#elif 0
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RIR;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tbuf[4] = 0x00;
  tbuf[5] = 0x0a;
  rlength = 25;
#else
  // set power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_WSR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x55;
  tbuf[5] = 0xaa;
#endif
  setCRC(tbuf, tlength);
  
  while(tlength--) {
    Wire.beginTransmission(addr);
    Wire.write((SC16IS7XX_THR_REG<<3)|(channel<<1));
    Wire.write(tbuf[index]);
    Serial.print("tbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(tbuf[index], HEX);
    Wire.endTransmission();
    index++;
  }

  //Serial.print("Available right after Tx: "); Serial.println(Wire.available(), DEC);
  delay(200);

  index = 0;
  Wire.beginTransmission(addr);
  Wire.write((SC16IS7XX_RHR_REG<<3)|(channel<<1));
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) rlength);
  Serial.print("Available right after Request: "); Serial.println(Wire.available(), DEC);

  int rlength_tmp;
  while(rlength) {
    rlength_tmp = Wire.available();
    Serial.print("Available: "); Serial.println(rlength_tmp, DEC);
    if(rlength_tmp) {
    //if (Wire.available()) {
      while(rlength_tmp--) {
        rbuf[index] = Wire.read();
        Serial.print("rbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(rbuf[index], HEX);
        index++;
        rlength--;
        //delay(1);
      }
    }
  }

}

void sub_test_i(uint8_t iaddr) {

  uint8_t addr = SC16IS752_SADDR0;
  uint8_t channel;
  uint8_t tbuf[SC_TX_BUF_SIZE] = {0,};
  uint8_t rbuf[SC_RX_BUF_SIZE] = {0,};
  uint16_t tlength = 8;
  uint16_t rlength = 8;
  uint8_t index = 0;
  uint8_t reg = 0;
  char c;
  Serial.println("Sub-test I - PZEM Command #3 (Search)");

  if(iaddr) {
    reg = iaddr;
  }
  else {
    Serial.print("Input Port (0~3): ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }

    if(setAddress(c, &addr, &channel)) {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);
  }

  if(!reg || (reg > 0xf8)) {
    Serial.println("Invalid Reg Address (Should be 0x01~0xf8).");
  }

#if 0
  // reset energy
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_REST;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tlength = 4;
  rlength = 5;
#elif 0
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ADDR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
#elif 1
  // read address
  tbuf[0] = reg;
  tbuf[1] = CMD_RIR;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
  rlength = 7;
#else
  // set power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_WSR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x55;
  tbuf[5] = 0xaa;
#endif
  setCRC(tbuf, tlength);
  
  while(tlength--) {
    Wire.beginTransmission(addr);
    Wire.write((SC16IS7XX_THR_REG<<3)|(channel<<1));
    Wire.write(tbuf[index]);
    Serial.print("tbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(tbuf[index], HEX);
    Wire.endTransmission();
    index++;
  }

  delay(200);

  index = 0;
  Wire.beginTransmission(addr);
  Wire.write((SC16IS7XX_RHR_REG<<3)|(channel<<1));
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) rlength);

  while(rlength) {
    if (Wire.available()) {
      rbuf[index] = Wire.read();
      Serial.print("rbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(rbuf[index], HEX);
      index++;
      rlength--;
    }
  }
  if(rbuf[0] != rbuf[1]) Serial.println("Something received =============================================");

}

void sub_test_l(void) {

  uint8_t data[8] = {0,};
  int numBytes;
  char c;
  Serial.println("Sub-test L - RTC");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Read RTC 8 Bytes
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0)); // Access the first register
    sw.endTransmission();

    numBytes = sw.requestFrom(I2C_ADDR_RTC, (uint8_t)8);
    for (int i = 0; i < numBytes; ++i) {
      data[i] = sw.read();
      Serial.print("data["); Serial.print(i, DEC); Serial.print("]: "); Serial.println(data[i], HEX);
    }
    if (numBytes != 8) {
      Serial.print("Read wrong number of bytes: ");
      Serial.println((int)numBytes);
      return;
    }
  } else if (c == '1') {  // Enable RTC Clock
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    sw.write(uint8_t(0));
    sw.endTransmission();
  } else if (c == '2') {  // Disable RTC Clock
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    sw.write(uint8_t(0x80));
    sw.endTransmission();
  } else if (c == '3') {  // Set RTC Time
    data[0] = 0*0x10;   // 10 Seconds
    data[0] += 0;       // Seconds

    data[1] = 5*0x10;   // 10 Minutes
    data[1] += 5;       // Minutes

    //data[2] = 0x40;   // 12-Hour Mode
    data[2] = 0x00;     // 24-Hour Mode

    data[2] += 1*0x10;  // 10 Hours
    data[2] += 6;       // Hours

    data[3] = 2;        // Day (1~7), Monday first

    data[4] = 1*0x10;   // 10 Date
    data[4] += 7;       // Date

    data[5] = 1*0x10;   // 10 Month
    data[5] += 0;       // Month

    data[6] = 2*0x10;   // 10 Year
    data[6] += 3;       // Year
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    for (int i = 0; i < 8; ++i) {
      sw.write(data[i]);
    }
    sw.endTransmission();
  } else if (c == '4') {  // Read Time
    readTime();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_m(void) {

  uint16_t data;
  uint8_t rdata[2];
  uint8_t val[2];
  int numBytes;
  char c;
  uint8_t reg;
  Serial.println("Sub-test M - IO Expander");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Read IO
    gpio_exp_read(&data);
    Serial.print("data: "); Serial.println(data, HEX);

    for(int i=1; i<9; i++){
      if(get_swtich_val(i) == SWITCH_ON){
        Serial.print("Switch"); Serial.print(i); Serial.println(": ON");
      } else {
        Serial.print("Switch"); Serial.print(i); Serial.println(": OFF");
      }
    }

  } else if(c == '1') {  // LED On/Off
    for(int i=1; i<5; i++){
      led_pm(i, LED_OFF);
    }

    for(int i=1; i<5; i++){
      led_pm(i, LED_ON);
      delay(500);
      led_pm(i, LED_OFF);
      delay(500);
    }

    led_fail(LED_ON);
    delay(500);
    led_fail(LED_OFF);
    delay(500);

    led_tx(LED_ON);
    delay(500);
    led_tx(LED_OFF);
    delay(500);

    led_rx(LED_ON);
    delay(500);
    led_rx(LED_OFF);
    delay(500);

  } else if(c == '2') {  // Data Read
    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    i2c_read_sw(I2C_ADDR_IO, reg, rdata, 2);
    Serial.print("data[0]: "); Serial.println(rdata[0], HEX);
    Serial.print("data[1]: "); Serial.println(rdata[1], HEX);

  } else if(c == '3') {  // Data Write
    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[0] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      val[0] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[0] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      val[0] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input High Byte Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      val[1] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      val[1] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    i2c_write_sw(I2C_ADDR_IO, reg, val, 2);

  } else if(c == '4') {  // Dual Uart LED
    for(int i=2; i<6; i++){
      dual_uart_led_set(i, LED_OFF);
    }

    for(int i=2; i<6; i++){
      dual_uart_led_set(i, LED_ON);
      delay(500);
      dual_uart_led_set(i, LED_OFF);
      delay(500);
    }
  } else if(c == '5') {  // GPIO HIGH
    digitalWrite(PIN_GPIO, HIGH);
  } else if(c == '6') {  // GPIO LOW
    digitalWrite(PIN_GPIO, LOW);
  } else if(c == '7') {  // PCM_Control HIGH
    Serial.println("PCM Control to HIGH - AC Power-ON");
    digitalWrite(PIN_PCM_CONTROL, HIGH);
  } else if(c == '8') {  // PCM_Control LOW
    Serial.println("PCM Control to LOW - AC Power-OFF");
    digitalWrite(PIN_PCM_CONTROL, LOW);
  } else if(c == '9') {
    Serial.print("System will be restart in 10 seconds");
    for(int i=0; i<10; i++){
      Serial.print(".");
      delay(1000);
    }
    Serial.println();
    ESP.restart();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_n(void) {

  uint8_t data;
  int numBytes;
  char c;
  Serial.println("Sub-test N - W5500");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // 
    // W5500
    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    Ethernet.init(SS);

    Serial.print("w5500: "); Serial.println(w5500, DEC);
    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);

  } else if(c == '1') {  // 
    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      SPI.transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    SPI.endTransaction();

  } else if(c == '2') {  // VSPI Test

    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

    SPIClass* vspi = new SPIClass(VSPI);
    vspi->begin();
    vspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      vspi->transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    vspi->endTransaction();
    delete vspi;

  } else if(c == '3') {  // IP Set
    Ethernet.init(PIN_ETH_CS);
    pCUR_SPI = pspi;
    Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
    pspi->setFrequency(40000000);
    Ethernet._pinRST = PIN_W5500_RST;
    Ethernet._pinCS = PIN_ETH_CS;
    Ethernet.setHostname("PMS_001");
    Ethernet.setRetransmissionCount(3);
    Ethernet.setRetransmissionTimeout(4000);

    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

  } else if(c == '4') {  // Check LINK Status
    Serial.print("linkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); //LINK_ON, LINK_OFF
    Serial.print("PhyState: "); Serial.println(Ethernet.phyState(), HEX);
    Serial.print("HardwareStatus: "); Serial.println(Ethernet.hardwareStatus());
    Serial.print("speed: "); Serial.println(Ethernet.speed(), DEC);
    Serial.print("duplex: "); Serial.println(Ethernet.duplex(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());
    Serial.print("dnsServerIP: "); Serial.println(Ethernet.dnsServerIP());

  } else if(c == '5') {
    unsigned int localPort = 1883;
    char packetBuffer[255];
    int packetSize;
    int len;
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    while(1) {
      packetSize = Udp.parsePacket();

      if (packetSize)
      {
        Serial.print(F("Received packet of size "));
        Serial.println(packetSize);
        Serial.print(F("From "));
        IPAddress remoteIp = Udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(F(", port "));
        Serial.println(Udp.remotePort());

        // read the packet into packetBufffer
        len = Udp.read(packetBuffer, 255);

        if (len > 0)
        {
          packetBuffer[len] = 0;
        }

        Serial.println(F("Contents:"));
        Serial.println(packetBuffer);
      }

      if(Serial.available()) {
        c = Serial.read();
        if(c == 'q') {
          Serial.println("Exit Udp Receive");
          break;
        }
      }
      delay(100);
      //Serial.print(".");
    }
  } else if(c == '6') {
    unsigned int localPort = 8080;
    unsigned int serverPort = 1883;
    int ret;
    size_t wsize;
    IPAddress server_ip(192, 168, 1, 149);
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    ret = Udp.beginPacket(server_ip, serverPort);
    Serial.print("Return of beginPacket: "); Serial.println(ret, DEC);
    wsize = Udp.write("hello from esp");
    Serial.print("Return of write: "); Serial.println(wsize);
    ret = Udp.endPacket();
    Serial.print("Return of endPacket: "); Serial.println(ret, DEC);

  } else if(c == '7') {
    //DhcpClass* dhcp = new DhcpClass();
    dhcp->beginWithDHCP(nc_mac);
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
    Ethernet.setLocalIP(dhcp->getLocalIp());
    //delete dhcp;
  } else if(c == '8') {
    Serial.print("DhcpServerIp(): "); Serial.println(dhcp->getDhcpServerIp());
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
  } else if(c == '9') {
    //Ethernet.WoL(1);
    //Serial.print("WoL: "); Serial.println(Ethernet.WoL(), DEC);
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_o(void) {

  uint8_t uport = 1;
  uint8_t dtype = 0;
  char c;
  uint8_t data = 0;
  Serial.println("Sub-test O - UART TX");

  Serial.print("Select Port (1:RS232, 2:RS485): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    setRS485Dir(MAX485_DIR_SEND);
  } else {
    Serial.println("Invalid port number");
    return;
  }

  while(1) {
    Serial.println("Input data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c) || (c == '#')) break;
      }
      delay(100);
    }
    if(!isalnum(c)){
      Serial.println("Quit data input");
      break;
    }
    Serial.println(c);
    
    if(uport == 1) {
      Serial1.write(c);
    } else if(uport == 2) {
      Serial2.write(c);
    }

  }

}

void sub_test_p(void) {

  uint8_t uport = 1;
  uint8_t dtype = 0;
  char c;
  uint8_t data[128] = {0,};
  uint16_t length, rsize;
  Serial.println("Sub-test P - UART RX");

  Serial.print("Select Port (1:RS232, 2:RS485): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    setRS485Dir(MAX485_DIR_RECEIVE);
  } else {
    Serial.println("Invalid port number");
    return;
  }

  Serial.println("Received Data: ");

  while(1) {
    if(uport == 1) {
      length = Serial1.available();
      if(length) {
        if(length > 128) length = 128;
      }
      rsize = Serial1.read(data, length);
      for(int i=0; i < (int) rsize; i++) {
        Serial.println((char) data[i]);
      }
    } else if(uport == 2) {
      length = Serial2.available();
      if(length) {
        if(length > 128) length = 128;
      }
      rsize = Serial2.read(data, length);
      for(int i=0; i < (int) rsize; i++) {
        Serial.println((char) data[i]);
      }
    }

    if(Serial.available()) {
      c = Serial.read();
      if(c == 'q') {
        Serial.println("Exit Data Receive");
        break;
      }
    }
    delay(100);
  }

}

void sub_test_q(void) {
#ifndef DISABLE_IR_FUNCTION
  char c;

  Serial.println("Sub-test Q - IR");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // PIN_IR
    uint8_t ir_pin = 0;
    while(1) {
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)){
          Serial.println(c);
        } else {
          continue;
        }
      }
      if(c == 'q'){
        Serial.println("Quit loop");
        break;
      }
      Serial.printf("PIN_IR: %d\r\n", ir_pin);
      digitalWrite(PIN_IR, ir_pin);
      if(ir_pin){
        ir_pin = 0;
      } else {
        ir_pin = 1;
      }
      delay(1000);
    }
  } else if(c == '1') {

      Serial.println("Send NEC Raw 32bit Data");

      // Address: 0x00, Command: 0x07
      IrSender.sendNEC(0x00FFE01F);

  } else if(c == '2') {

      Serial.println("Send Raw Data");

      // F609FF00
      uint16_t irSignal1[] = {9000, 4450, 600, 550, 550, 550, 600, 550, 550, 550, 600, 500, 600, 550, 550, 600, 550, 550, 600, 1650, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1650, 600, 1650, 550, 1700, 550, 550, 600, 550, 550, 1700, 550, 550, 600, 500, 600, 550, 550, 600, 550, 550, 600, 1650, 550, 1700, 550, 550, 600, 1650, 550, 1700, 550, 1700, 550, 1700, 550}; // NEC FF906F
      // E21DFF00
      uint16_t irSignal2[] = {9000, 4500, 550, 550, 600, 550, 550, 550, 550, 600, 550, 550, 600, 550, 550, 550, 550, 550, 600, 1650, 600, 1700, 500, 1700, 600, 1700, 500, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 550, 550, 1700, 550, 1700, 550, 1700, 550, 550, 550, 550, 600, 550, 550, 550, 600, 1700, 500, 600, 550, 550, 600, 500, 600, 1650, 600, 1650, 600, 1650, 550}; // NEC FFB847
      // F807FF00
      uint16_t irSignal3[] = {9000, 4500, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550}; // NEC FFB847

      IrSender.sendRaw(irSignal1, 67, 38);
      delay(100);
      IrSender.sendRaw(irSignal2, 67, 38);
      delay(100);
      IrSender.sendRaw(irSignal3, 67, 38);

  } else {
    Serial.println("Invalid Test Number");
    return;
  }
#endif
}

void sub_test_r(void) {

  uint8_t data;
  char c;
  Serial.println("Sub-test R - EEPROM");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {
    for(int i=0; i<EEPROM_SIZE; i++){
      Serial.printf("Data[%d]: %02x\r\n", i, EEPROM.read(i));
    }
  } else if(c == '1') {
    for(int i=0; i<EEPROM_SIZE; i++){
      data = (uint8_t) (i % 256);
      EEPROM.write(i, data);
    }
    EEPROM.commit();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_s(void) {

  uint8_t data;
  char c;
  int r_data = 0;
  String strLog;
  Serial.println("Sub-test S - LiitleFS");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {
    LittleFS.begin();
    LittleFS.format();
  } else if(c == '1') {  // create log file
    writeFile(LittleFS, "/log_today.txt", (String((char*) str_cur_date)+ "\r\n").c_str());
    readFile(LittleFS, "/log_today.txt");
  } else if(c == '2') {  // add log message
    r_data = random(0, 1000);
    appendFile(LittleFS, "/log_today.txt", (String((char*) str_cur_time)+" - log data: "+String(r_data)+"\r\n").c_str());
    readFile(LittleFS, "/log_today.txt");
  } else if(c == '3') {
    File file = LittleFS.open("/log_today.txt");
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    int i=0;
    while(file.available()){
        Serial.printf("[%d]:%s\r\n", i++, file.readStringUntil(0x0d));
        file.read();
    }
    file.close();

  } else if(c == '4') {
    writeFile(LittleFS, "/init.txt", (String((char*) str_cur_date)+ "\r\n").c_str());
    readFile(LittleFS, "/init.txt");
  } else if(c == '5') {
    Serial.println("Enter data:");
    char cbuf[256] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          break;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
  } else if(c == '6') {
    Serial.printf("TotalBytes: %d(0x%x), UsedBytes: %d(0x%x)\r\n", 
      LittleFS.totalBytes(), LittleFS.totalBytes(), LittleFS.usedBytes(), LittleFS.usedBytes());
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_t(void) {

  uint8_t data;
  char c;
  int r_data = 0;
  Serial.println("Sub-test T - Web Server");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {
    // Connect to WiFi network
    if((gv_ssid[0] == 0x00) || (gv_passwd[0] == 0x00)){
      Serial.println("Invalid SSID or PASSWORD");
      return;
    }
    WiFi.begin(gv_ssid, gv_passwd);
    Serial.println("WiFi starting");
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(gv_ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else if(c == '1') {
    // use mdns for host name resolution
    if (!MDNS.begin(host))  // http://PMSMGR.local
      Serial.println("Error setting up MDNS responder!");   
    else
      Serial.printf("mDNS responder started. Hotstname = http://%s.local\n", host);
  } else if(c == '2') {
    server.on("/", HTTP_GET, handleMain);
  
    // upload file to FS. Three callbacks
    server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        // flashing firmware to ESP
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
 
    server.on("/delete", HTTP_GET, handleFileDelete);
    server.on("/main", HTTP_GET, handleMain); // JSON format used by /edit
    // second callback handles file uploads at that location
    server.on("/edit", HTTP_POST, []()
      {server.send(200, "text/html", "<meta http-equiv='refresh' content='1;url=/main'>File uploaded. <a href=/main>Back to list</a>"); }, handleFileUpload); 
       server.onNotFound([](){if(!handleFileRead(server.uri())) server.send(404, "text/plain", "404 FileNotFound");});
 
    server.begin();
  } else if(c == '3') {
    Serial.println("Web-server is running!");
    Serial.println("Press q to quit: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) Serial.println(c);
      }
      if(c == 'q') {
        Serial.println("Quit loop");
        break;
      }
      server.handleClient();
      delay(2);
    }
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_v(void) {

  uint8_t data;
  char c;
  uint8_t update_needed = 0;
  uint8_t str_tmp[128] = {0,};
  String str_in;
 
  Serial.println("Sub-test V - Settings");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // create init file
    fsFound = initFS(true, true);
    if(!fsFound) {
      Serial.println("Filesystem is not found!");
      return;
    }
    update_settings();

    readFile(LittleFS, "/init.txt");
  } else if(c == '1') {  // WLAN settings
    Serial.print("[WLAN] Enter SSID: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      memset(gv_ssid, 0x00, sizeof(gv_ssid));
      strcpy(gv_ssid, cbuf);
      update_needed = 1;
    }

    Serial.print("[WLAN] Enter PASSWD: ");
    memset(cbuf, 0x00, sizeof(cbuf));
    idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      memset(gv_passwd, 0x00, sizeof(gv_passwd));
      strcpy(gv_passwd, cbuf);
      update_needed = 1;
    }
  } else if(c == '2') {  // LAN settings
    Serial.print("[LAN] Select Static/DHCP: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    for(int i=0; i < sizeof(cbuf); i++){
      cbuf[i] = toLowerCase(cbuf[i]);
    }
    if(cbuf[0]){
      str_in = cbuf;
      if(!str_in.compareTo("static")){
        nc_ip_type = IP_TYPE_STATIC;
        update_needed = 1;
      } else if(!str_in.compareTo("dhcp")){
        nc_ip_type = IP_TYPE_DHCP;
        update_needed = 1;
      }
    }

    Serial.print("[LAN] Enter IP: ");
    memset(cbuf, 0x00, sizeof(cbuf));
    idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      str_in = cbuf;
      nc_ip.fromString(str_in);
      update_needed = 1;
    }

    Serial.print("[LAN] Enter Subnet: ");
    memset(cbuf, 0x00, sizeof(cbuf));
    idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      str_in = cbuf;
      nc_subnet.fromString(str_in);
      update_needed = 1;
    }

    Serial.print("[LAN] Enter Gateway: ");
    memset(cbuf, 0x00, sizeof(cbuf));
    idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      str_in = cbuf;
      nc_gateway.fromString(str_in);
      update_needed = 1;
    }

    Serial.print("[LAN] Enter DNS: ");
    memset(cbuf, 0x00, sizeof(cbuf));
    idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      str_in = cbuf;
      nc_dns.fromString(str_in);
      update_needed = 1;
    }
  } else if(c == '3') {  // Server settings
    Serial.print("[Server] Enter IP: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      str_in = cbuf;
      nc_server.fromString(str_in);
      update_needed = 1;
    }
  } else if(c == '4') {  // System settings
    Serial.print("[System] Enter SN: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(isalnum(cbuf[0])){
      update_serial_number(cbuf);
      update_needed = 1;
    }
  } else {
    Serial.println("Invalid Test Number");
    return;
  }
  if(update_needed){
    update_settings();
    readFile(LittleFS, "/init.txt");
  }
}

void sub_test_y(void) {
  uint8_t buf[SC_TX_BUF_SIZE] = {0,};
  uint8_t length = 0;
  char c;
  Serial.println("Sub-test Y - CRC check");

  Serial.print("Input Total Length (2 dec): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length = (c - '0')*10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length += (c - '0');
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  for(int i=0; i <(length-2); i++){

    Serial.print("Input Buffer Data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      buf[i] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'f')) {
      buf[i] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)) break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      buf[i] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
      buf[i] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

  }

  setCRC(buf, length);

  Serial.print("CRC_H: "); Serial.println(buf[length-2], HEX);
  Serial.print("CRC_L: "); Serial.println(buf[length-1], HEX);

}

void sub_test_z(void) {

  char c;
  int idx=0;
  uint8_t addr;
  PZEM004Tv30* ppzem[4];

#if (SYS_PMS_HW == SYS_PMS04)
  ppzem[0] = &pzem3;
  ppzem[1] = &pzem2;
  ppzem[2] = &pzem1;
  ppzem[3] = &pzem0;
#elif (SYS_PMS_HW == SYS_PMS01)
  ppzem[0] = &pzem0;
#endif

  Serial.println("Sub-test Z - PZEM Measurement Test");

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        Serial.println(c);
      } else {
        continue;
      }
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }

    // PZEM Address Check
    addr = ppzem[idx]->readAddress();
    Serial.print("Custom Address[PM-"); Serial.print(idx+1); Serial.print("]: ");
    Serial.println(addr, HEX);

    if(addr) {

      led_pm(idx+1, LED_ON);

      // Read the data from the sensor
      float voltage = ppzem[idx]->voltage();
      float current = ppzem[idx]->current();
      float power = ppzem[idx]->power();
      float energy = ppzem[idx]->energy();
      float frequency = ppzem[idx]->frequency();
      float pf = ppzem[idx]->pf();

      // Check if the data is valid
      if(isnan(voltage)){
          Serial.println("Error reading voltage");
      } else if (isnan(current)) {
          Serial.println("Error reading current");
      } else if (isnan(power)) {
          Serial.println("Error reading power");
      } else if (isnan(energy)) {
          Serial.println("Error reading energy");
      } else if (isnan(frequency)) {
          Serial.println("Error reading frequency");
      } else if (isnan(pf)) {
          Serial.println("Error reading power factor");
      } else {

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);

      }
      led_pm(idx+1, LED_OFF);
    }
    else{
      led_pm(idx+1, LED_OFF);
    }

    Serial.println();

#if (SYS_PMS_HW == SYS_PMS04)
   if(++idx > 3) idx = 0;
#endif
    delay(1000);
  }

}
