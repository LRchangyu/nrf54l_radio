# nrf54l_radio
A demo for nRF54L radio test for 8K TX/RX

# 说明

SDK：NCS 3.0.2
硬件： 2 个 nRF54L15 DK。
需要：nrfutil

通过 UICR 写入特定值，区分 TX/RX

一个 DK 的 NRF_UICR->OTP[0] 写入 0xABABA，表示作为 TX。

可使用 rx_flash.bat/tx_flash.bat 。

