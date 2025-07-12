# Getting Started

This is userguide on how to read `MTF-02` Optical Flow sensor from Micolink. Please follow below step on how to read data from this sensor.

## 1. Set the MTF-02 ID

Open `Miconlink assistant` and select the following
```
Change protocol to : MAV_APM
Change ID : 200 (very important) otherwise it wont work
Select `Write` to write the configuration
```

<img width="1464" height="871" alt="image" src="https://github.com/user-attachments/assets/86d92276-f18c-41eb-a130-4a136a723beb" />

## 2. Connection Pin

You've to configure the connection as follow
| ESP32-S3 Super Min | MTF-02       |
|--------------------|--------------|
| VDD (5V)           | 5V (Black)   |
| GND                | GND (Red)    |
| RX - 1             | SCL (Yellow) |
| TX - 2             | SDA (White)  |

## 3. Upload the code and observe the result as follow

```
MTF02 - FlowX: -0.5 FlowY: 0.0 Quality: 57 Distance: 22cm Ready: YES
MTF02 - FlowX: 0.0 FlowY: 0.5 Quality: 59 Distance: 22cm Ready: YES
MTF02 - FlowX: -5.0 FlowY: 0.5 Quality: 56 Distance: 22cm Ready: YES
```
