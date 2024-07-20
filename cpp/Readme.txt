
serial_driver.h为透传设备的驱动库，使用时只需引用头文件。

提供三个用户函数:

open_serial() 打开串口
send_serial()  发送数据
recieve_serial() 开启接收数据线程

用户需要自己定义并编写接收数据后处理函数，并作为回调函数传入recieve_serial() 。在收到正确数据时会执行。



具体使用方法见example_send()和example_recieve()

注：
usb-cdc协议在linux里面显示的是ACM*