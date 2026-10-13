Checked acquisition preserves the ranges, units, and axis transform from
BolderFlight BMI088 Arduino v1.0.1, revision
5e839cbddf5eb540d9530e36385804ec0b47ed23 (MIT, BMI088_LICENSE):
https://github.com/bolderflight/bmi088-arduino/tree/5e839cbddf5eb540d9530e36385804ec0b47ed23

Vendor initialization is synchronous and uses finite sequences of register
transactions and Arduino delay(), without polling loops. ESP32 Wire timeouts
bound individual transactions. Runtime reads and orientation seeding use only
checked transactions; initialization/self-test samples never enter fusion.
The owner reapplies bus clock/timeout after begin(). Accelerometer ERR_REG and
gyro self-test failure flag are checked; gyro has no corresponding ERR_REG.
