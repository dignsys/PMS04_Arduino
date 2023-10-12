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

  Install이 필요한 Library : AsyncDelay, Ethernet_Generic, SoftWire

- User Arduino Library (.ZIP) 추가

  Arduino IDE : Tools --> Sketch --> Include Library --> Add .ZIP Libraries...
## Sample Source Code for Hardware Basic Operation
- 실행 방법

  각각 sub_test_*() 함수로 구분되어 작성되어 있고, Serial Monitor의 Message 창에 해당하는 함수의 명령어를 입력하여 실행할 수 있음. 예를 들어, sub_test_a( )는 'a'를 입력하고 'Enter'키를 입력함으로써, 실행됨.
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