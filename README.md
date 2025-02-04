# ESP8266 Relay Controller

Проект **ESP8266 Relay Controller** предназначен для управления до 8 реле через MQTT и веб-интерфейс. Приложение поддерживает два режима работы:

- **Режим точки доступа (AP)** — используется для первоначальной настройки устройства (ввод параметров WiFi и MQTT) через страницу настроек.
- **Режим клиента (STA)** — после успешного подключения к WiFi открывается страница управления реле, где можно включать/выключать реле и видеть их текущее состояние.

## Особенности

- **Два режима работы:**
  - **AP-режим**: страница настроек для конфигурации WiFi и MQTT.
  - **STA-режим**: страница управления реле с визуальными индикаторами состояния.
- **Поддержка MQTT:** устройство подключается к MQTT-брокеру для обмена сообщениями и синхронизации состояния реле.
- **Веб-интерфейс:** две отдельные страницы (настройки и управление) с перекрёстными ссылками.
- **Локальное хранение настроек и состояний:** используется файловая система LittleFS для сохранения конфигурации и состояния реле.
- **Периодическое обновление состояния реле:** AJAX-запросы на сервер позволяют обновлять состояние реле на странице управления.



## Требования

- **Аппаратное обеспечение:** ESP8266
- **Программное обеспечение:** Arduino Core for ESP8266, PubSubClient, ESPAsyncTCP, ESPAsyncWebServer, ArduinoJson, DNSServer, LittleFS
- **Библиотеки:**  
  - [ESP8266WiFi](https://github.com/esp8266/Arduino)
  - [PubSubClient](https://github.com/knolleary/pubsubclient)
  - [ESPAsyncTCP](https://github.com/me-no-dev/ESPAsyncTCP)
  - [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
  - [ArduinoJson](https://arduinojson.org/)
  - [LittleFS](https://github.com/earlephilhower/arduino-esp8266littlefs)
  - [DNSServer](https://github.com/esp8266/Arduino/tree/master/libraries/DNSServer)

Использование
При включении устройства:
Если ESP8266 не подключается к заданной WiFi сети, автоматически включается режим AP. Перейдите по адресу 192.168.4.1 для настройки параметров WiFi и MQTT через страницу настроек.
После успешного подключения в режиме STA откройте страницу управления (например, по адресу, возвращаемому вашим маршрутизатором) для управления реле.
Управление реле:
На странице управления отображаются карточки с текущим состоянием реле. Нажимая кнопки "Включить" или "Выключить", вы отправляете HTTP‑запрос для переключения состояния реле.
Состояние реле обновляется автоматически через AJAX‑запросы.
Лицензия
Этот проект распространяется под лицензией MIT. Подробности см. в файле LICENSE.
