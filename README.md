# PMS04 (Arduino)

## Install Arduino IDE
- Download Arduino IDE

  참조: <https://www.arduino.cc/en/software>
- Install Additional Board Manager URLs for ESP32

  Arduino IDE : File --> Preferences --> Addtional board manager URLs에 하기의 json 파일 경로를 추가

  - Stable release link
  ```
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  ```

  참조: <https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html>

## Setup Board and Libraries
- Setup Board Type

  Arduino IDE : Tools --> Board --> esp32 --> ESP32 Dev Module 선택

- Standard Arduino Library 추가

  Arduino IDE : Tools --> Manage Libraries... --> Library Manager

  Install이 필요한 Library : AsyncDelay, Ethernet_Generic, SoftWire, Adafruit_SHT4x, ICM42670P, DFRobot_TMF8x01

- User Arduino Library (.ZIP) 추가

  Arduino IDE : Tools --> Sketch --> Include Library --> Add .ZIP Libraries...

  Install이 필요한 (.ZIP) Library : PZEM004Tv30-pms-x.x.x.zip, SC16IS752Serial-pms-x.x.x.zip, IRremoteESP8266-pms-x.x.x.zip (Optional)

## Definition Setup depends on Hardware
- Select PMS01/04

  사용하고자 하는 Hardware에 따라 pms04.ino 파일 상단에서 Definition 설정이 필요합니다.

  PMS04 사용의 경우
  ```
  //#define SYS_PMS_HW    SYS_PMS01
  #define SYS_PMS_HW   SYS_PMS04
  ```
  PMS01 사용의 경우
  ```
  #define SYS_PMS_HW    SYS_PMS01
  //#define SYS_PMS_HW   SYS_PMS04
  ```
- IR 송신 사용시

  IRremoteESP8266-pms-x.x.x.zip 설치
  ```
  //#define DISABLE_IR_FUNCTION
  ```
## Sample Source Code for Hardware Basic Operation
- 실행 방법

  시스템을 부팅하면, PZEM을 통하여 주기적으로 전력 데이터를 검출하는 기능이 Main Loop로 실행됩니다. 이후, '#'키를 입력하여, Sub Test 함수들을 시험할 수 있는 메뉴로 진입합니다.

  각 시험 항목들은 sub_test_*() 함수로 구분되어 작성되어 있고, Serial Monitor의 Message 창에 해당하는 함수의 명령어를 입력하여 실행할 수 있습니다. 예를 들어, sub_test_a( )는 'a'를 입력하고 'Enter'키를 입력함으로써, 실행됩니다.

  Sub Test 메뉴에서 다시 Main Loop로 돌아가고자 한다면, 'x'키를 눌러 Main Loop를 실행할 수 있습니다.
- Sub Test Function List
  - sub_test_a( ) : Read I2C - SC16IS752 channel별 register를 읽기 위한 함수
  - sub_test_b( ) : Write I2C - SC16IS752 channel별 register를 쓰기 위한 함수
  - sub_test_c( ) : Select clock setting - MAX14830 clock 설정 값을 계산하기 위한 함수 (미사용)
  - sub_test_d( ) : Access PZEM - PZEM Address를 읽기 위한 함수
  - sub_test_e( ) : PZEM command #1 - PZEM에 개별 Command를 전송하기 위한 함수
  - sub_test_f( ) : Write data to SC16IS752 UART - SC16IS752 UART에 임의의 데이터 블록을 전송하기 위한 함수
  - sub_test_g( ) : Read data from SC16IS752 UART - SC16IS752 UART에서 임의의 데이터 블록를 읽기 위한 함수
  - sub_test_h( ) : PZEM command #2 - PZEM에 개별 Command를 전송하기 위한 함수
  - sub_test_i( ) : PZEM command #3 - PZEM에 Search Command를 전송하기 위한 함수
  - sub_test_l( ) : RTC - RTC를 설정하고 시간 데이터를 읽어내기 위한 함수
  - sub_test_m( ) : IO Expander - IO Expander를 통해 GPIO를 읽고 쓰기 위한 함수
  - sub_test_n( ) : W5500 - W5500를 설정하고 Ethernet 통신을 시험하기 위한 함수
  - sub_test_o( ) : UART TX - RS232, RS485 송신 기능을 시험하기 위한 함수
  - sub_test_p( ) : UART RX - RS232, RS485 수신 기능을 시험하기 위한 함수
  - sub_test_r( ) : EEPROM에 데이터를 쓰고 읽는 기능을 시험하기 위한 함수
  - sub_test_s( ) : LittleFS 파일시스템을 적용하여 파일을 쓰고 읽는 기능을 시험하기 위한 함수
  - sub_test_t( ) : Wifi를 연결하고, Web Server를 기동하여 파일시스템과 연동 시험하기 위한 함수
  - sub_test_v( ) : 설정 파일을 이용하여, WLAN, LAN 설정을 저장하는 기능을 시험하기 위한 함수
  - sub_test_y( ) : CRC check - PZEM Packet의 CRC 계산을 위한 함수
  - sub_test_z( ) : PZEM Measurement - PZEM Power Measurement Test를 위한 함수
- PZEM Measurement Test
  
  4개(index 0~3)의 pzem객체 pointer를 통해 측정하고 표시함

  메시지 창에 ‘z’ 입력하고 ‘Enter’를 입력하면, 주기적으로 4개(index 0~3) port를 순차적으로 측정하여 각기 전압, 전류, 전력, 주파수, 역률 등을 표시함
  ```
  void sub_test_z(void) {
    ~~~ 중략 ~~~
    PZEM004Tv30* ppzem[4];

    ppzem[0] = &pzem0;
    ppzem[1] = &pzem1;
    ppzem[2] = &pzem2;
    ppzem[3] = &pzem3;
    ~~~ 중략 ~~~

    while(1) {
    ~~~ 중략 ~~~

      // PZEM Address Check
      addr = ppzem[idx]->readAddress();
      Serial.print("Custom Address["); Serial.print(idx); Serial.print("]: ");
      Serial.println(addr, HEX);
      ~~~ 중략 ~~~

      // Read the data from the sensor
      float voltage = ppzem[idx]->voltage();
      float current = ppzem[idx]->current();
      float power = ppzem[idx]->power();
      float energy = ppzem[idx]->energy();
      float frequency = ppzem[idx]->frequency();
      float pf = ppzem[idx]->pf();
      ~~~ 중략 ~~~

      // Print the values to the Serial console
      Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
      Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
      Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
      Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
      Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
      Serial.print("PF: ");           Serial.println(pf);
      ~~~ 중략 ~~~

      if(++idx > 3) idx = 0;
      delay(1000);
    }
  }
  ```
- RS232/RS485 Test

  Board #1에서는 송신 함수(sub_test_o), Board #2에서는 수신 함수(sub_test_p)를 실행하여 시험함. 기본적으로는 Serial1,2의 write()/read() 함수를 사용하여 데이터 송수신을 실행함.

  송신 Board #1에서 ‘o’, 수신 Board #2에서 ‘p’를 입력하여 각각 Sub- test 함수를 실행함.

  RS232 시험은, 두 대의 Board를 RS232단자로 연결하는데, TX와 RX를 서로 엇갈려 연결함. RS485 시험은, 두 대의 Board를 RS485단자로 연결하는데, TRX+는 TRX+와 TRX-는 TRX-와 각각 연결함.

  송신 Board에서 문자를 입력하여 수신 Board에서 동일한 문자가 표시됨을 확인할 수 있음



  ```
  void sub_test_o(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test O - UART TX");

    Serial.print("Select Port (1:RS232, 2:RS485): ");
    ~~~ 중략 ~~~

    while(1) {
      Serial.println("Input data: ");
      while(1){
        if(Serial.available()) {
          c = Serial.read();
          if(isalnum(c) || (c == '#')) break;
        }
        delay(100);
      }
    ~~~ 중략 ~~~
    
      if(uport == 1) {
        Serial1.write(c);
      } else if(uport == 2) {
        Serial2.write(c);
      }
    }
  }

  ```
  ```
  void sub_test_p(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test P - UART RX");

    Serial.print("Select Port (1:RS232, 2:RS485): ");
    ~~~ 중략 ~~~

    while(1) {
      if(uport == 1) {
        length = Serial1.available();
        ~~~ 중략 ~~~
        rsize = Serial1.read(data, length);
        for(int i=0; i < (int) rsize; i++) {
          Serial.println((char) data[i]);
        }
      } else if(uport == 2) {
        length = Serial2.available();
        ~~~ 중략 ~~~
        rsize = Serial2.read(data, length);
        for(int i=0; i < (int) rsize; i++) {
          Serial.println((char) data[i]);
        }
      }
      ~~~ 중략 ~~~
    }
  }
  ```
- LAN Test

  기 정의된 VSPI(pspi)를 이용하여 Ethernet 객체를 초기화함.

  MAC, IP 등을 설정하고 Link Status를 확인함

  이후, UDP Packet를 이용하여, Server, Client 통신이 가능함

  Test Command ‘n’를 입력하고, 세부 Command를 추가로 입력하여 W5500 (Ethernet Controller) 동작을 확인함

  (n+Enter+3+Enter)를 입력하여 Static IP를 설정하고, (n+Enter+4+Enter)를 입력하여 LINK Status를 확인함

  LINK ON(1)이 되었다면, UDP Server/Client 구성이 가능함.


  ```
  void sub_test_n(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test N - W5500");
    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~

    } else if(c == '3') {  // IP Set
      Ethernet.init(PIN_ETH_CS);
      pCUR_SPI = pspi;
      Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
      ~~~ 중략 ~~~
      Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
      Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

    } else if(c == '4') {
      Serial.print("linkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); 
      Serial.print("PhyState: "); Serial.println(Ethernet.phyState(), HEX);
      Serial.print("HardwareStatus: "); 
            Serial.println(Ethernet.hardwareStatus());
      Serial.print("speed: "); Serial.println(Ethernet.speed(), DEC);
      Serial.print("duplex: "); Serial.println(Ethernet.duplex(), DEC);
      Serial.print("dnsServerIP: "); Serial.println(Ethernet.dnsServerIP());
      ~~~ 중략 ~~~
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
      ~~~ 중략 ~~~
    }
  }
  ```
- RTC Test

  (l+Enter+3+Enter)를 입력하여 시간을 설정하고, (l+Enter+4+Enter)를 입력하여 현재 시간을 확인함

  ```
  void sub_test_l(void) {
    ~~~ 중략 ~~~

    Serial.println("Sub-test L - RTC");
    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~

    } else if (c == '3') {  // Set RTC Time
      ~~~ 중략 ~~~

      data[4] = 2*0x10;   // 10 Date
      data[4] += 2;       // Date

      data[5] = 0*0x10;   // 10 Month
      data[5] += 9;       // Month

      data[6] = 2*0x10;   // 10 Year
      data[6] += 3;       // Year
      sw.beginTransmission(I2C_ADDR_RTC);
      sw.write(uint8_t(0));
      for (int i = 0; i < 8; ++i) {
        sw.write(data[i]);
      }
      sw.endTransmission();
      ~~~ 중략 ~~~
    } else if (c == '4') {  // Read Time
      readTime();
      ~~~ 중략 ~~~
  }
  ```
- DIP SW Read Test

  (m+Enter+0+Enter)를 입력하여 현재 DIP SW의 상태를 확인함

  관련 함수, gpio_exp_conf(), gpio_exp_read(), gpio_exp_write(), get_switch_val() 참조

  ```
  void sub_test_m(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test M - IO Expander");

    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~

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
    ~~~ 중략 ~~~
  }
  ```
- LED Test 

  (m+Enter+1+Enter)를 입력하면, PM1, PM2, PM3, PM4, FAIL, TX, RX LED 순으로 점멸함. (m+Enter+4+Enter)를 입력하면, LED2, LED3, LED4, LED5의 순으로 점멸함.

  관련 함수, led_pm(), led_fail(), led_tx(), led_rx(), dual_uart_led_set() 참조

  ```
  void sub_test_m(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test M - IO Expander");

    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~

    } else if(c == '1'){ // LED On/Off
        for(int i=1; i<5; i++){
            led_pm(i, LED_OFF);
        }
        for(int i=1; i<5; i++){
          led_pm(i, LED_ON);
          delay(500);
          led_pm(i, LED_OFF);
          delay(500);
        }
        ~~~ 중략 ~~~
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
        ~~~ 중략 ~~~
  }
  ```
- LittleFS Filesystem Test

  (s+Enter+0+Enter)를 입력하면, LittleFS 파일 시스템을 시작하고 Format을 실행함. (s+Enter+1+Enter)를 입력하면, log_today.txt파일을 생성하고 날짜 데이터를 기록함. (s+Enter+2+Enter)를 입력하면, log_today.txt파일에 시간 데이터를 첨부한 로그 데이터를 추가하여 기록함.
  ```
  void sub_test_s(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test S - LiitleFS");

    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~
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
    ~~~ 중략 ~~~
  }
  ```

- Web Server Test

  (t+Enter+0+Enter)를 입력하면, WLAN 연결을 실행함. (t+Enter+1+Enter)를 입력하면, MDNS를 설정하여 'pmsmgr.local'로 Web server의 접근이 가능하도록 함. (t+Enter+2+Enter)를 입력하면, Web server의 handler를 설정하여, Web server 접근에 대해 응답하도록 설정함. (t+Enter+3+Enter)를 입력하면, 최종적으로 Web server를 기동 시킴. 이후, 'http://pmsmgr.local'를 통하여 Web server의 동작을 시험할 수 있음.
  ```
  void sub_test_t(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test T - Web Server");

    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~
    if(c == '0') {
      ~~~ 중략 ~~~
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
      ~~~ 중략 ~~~
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
      ~~~ 중략 ~~~
        server.handleClient();
        delay(2);
      }
      ~~~ 중략 ~~~
  }
  ```
- Settings Test

  (v+Enter+0+Enter)를 입력하면, Settings를 저장하고 관리할 수 있는 'init.txt' 파일을 생성하고 초기 데이터를 저장함. (v+Enter+1+Enter)를 입력하면, WLAN 설정을 입력하고 저장할 수 있음. (v+Enter+2+Enter)를 입력하면, LAN 설정을 입력하고 저장할 수 있음.
  ```
  void sub_test_v(void) {
    ~~~ 중략 ~~~
    Serial.println("Sub-test V - Settings");

    Serial.print("Input Test Number: ");
    ~~~ 중략 ~~~
    if(c == '0') {  // create init file
      fsFound = initFS(false, false);
      if(!fsFound) {
        Serial.println("Filesystem is not found!");
        return;
      }
      update_settings();

      readFile(LittleFS, "/init.txt");
    } else if(c == '1') {  // WLAN settings
      Serial.print("[WLAN] Enter SSID: ");
      ~~~ 중략 ~~~
      Serial.printf("Input Data String: %s\r\n", cbuf);
      if(cbuf[0]){
        memset(gv_ssid, 0x00, sizeof(gv_ssid));
        strcpy(gv_ssid, cbuf);
        update_needed = 1;
      }

      Serial.print("[WLAN] Enter PASSWD: ");
      ~~~ 중략 ~~~
      Serial.printf("Input Data String: %s\r\n", cbuf);
      if(cbuf[0]){
        memset(gv_passwd, 0x00, sizeof(gv_passwd));
        strcpy(gv_passwd, cbuf);
        update_needed = 1;
      }
    } else if(c == '2') {  // LAN settings
      Serial.print("[LAN] Select Static/DHCP: ");
      ~~~ 중략 ~~~
      if(!str_in.compareTo("static")){
        nc_ip_type = IP_TYPE_STATIC;
        update_needed = 1;
      } else if(!str_in.compareTo("dhcp")){
        nc_ip_type = IP_TYPE_DHCP;
        update_needed = 1;
      }
      ~~~ 중략 ~~~
      Serial.print("[LAN] Enter IP: ");
      ~~~ 중략 ~~~
      Serial.printf("Input Data String: %s\r\n", cbuf);
      if(cbuf[0]){
        str_in = cbuf;
        nc_ip.fromString(str_in);
        update_needed = 1;
      }
    ~~~ 중략 ~~~
    if(update_needed){
      update_settings();
      readFile(LittleFS, "/init.txt");
    }
  }
  ```