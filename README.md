# huiliuhan-control
使用stc8g1h1k作为mcu，ble模块作为手机控制接收器，k型电偶和max6675作为温度测量装置
传送给单片机0x55 0x02 0x11--0x12 value BCC_code\r\n0x11 is T1 S1 T2 S2 T3 S3 BCC_code，设置温度节点和时间节点
传送给单片机0x55 02 12 00 00 00 00 00 00 45 开始加热，执行回流焊程序。
