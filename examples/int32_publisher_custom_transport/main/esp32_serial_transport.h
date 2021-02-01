// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _MICROROS_CLIENT_ESP32_SERIAL_TRANSPORT_H_
#define _MICROROS_CLIENT_ESP32_SERIAL_TRANSPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

bool esp32_serial_open(struct uxrCustomTransport * transport);
bool esp32_serial_close(struct uxrCustomTransport * transport);
size_t esp32_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t esp32_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif //_MICROROS_CLIENT_ESP32_SERIAL_TRANSPORT_H_